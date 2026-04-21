#!/usr/bin/env python3
"""
Run 3-state relative EKFs on day_02 synced CSV.

Compares onboard sensor inputs vs mocap-derived inputs for all 6 directed
pairs:  0→1, 1→0, 0→2, 2→0, 1→2, 2→1

Drone 0 is a static ground beacon (zero yaw rate, velocity, height).
"""

from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt

from ekf_utils import EKF, wrapToPi

# ──────────────────────────────────────────────────────────────────────
# Configuration
# ──────────────────────────────────────────────────────────────────────

DATA_CSV   = Path("relative_data/day_02/synced_all.csv")
OUTPUT_DIR = Path("relative_data_plots/day02")

STATIC_DRONE = 0
FS = 50.0                  # synced CSV sample rate [Hz]
FILTER_CUTOFF_HZ = 3.0     # low-pass cutoff for gyro filtering
FILTER_ORDER = 2

PAIRS = [(0, 1), (1, 0), (0, 2), (2, 0), (1, 2), (2, 1)]

# EKF tuning
Q  = np.diag([1e-2, 1e-1, 1e-1])
R  = np.array([[0.10**2]])
P0 = np.diag([1.0**2, 2.0**2, 2.0**2])


# ──────────────────────────────────────────────────────────────────────
# Dynamics & measurement models (3-state: psi12, x12, y12)
# ──────────────────────────────────────────────────────────────────────

def f_3state(x, u):
    psi12, x12, y12 = x
    psi_dot_i, vxi, vyi, _vzi, psi_dot_j, vxj, vyj, _vzj, _z12 = u[:9]
    c, s = np.cos(psi12), np.sin(psi12)
    return np.array([
        psi_dot_j - psi_dot_i,
        (c * vxj - s * vyj) - vxi + psi_dot_i * y12,
        (s * vxj + c * vyj) - vyi - psi_dot_i * x12,
    ])


def h_3state(x, u):
    _, x12, y12 = x
    z12 = u[8]
    d = np.sqrt(x12**2 + y12**2 + z12**2)
    return np.array([max(d, 1e-8)])


# ──────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────

def lowpass(signal, cutoff=FILTER_CUTOFF_HZ, fs=FS, order=FILTER_ORDER):
    """Zero-phase Butterworth low-pass filter."""
    if np.all(signal == 0):
        return signal
    b, a = butter(order, cutoff / (fs / 2), btype="low")
    return filtfilt(b, a, signal)


def pair_label(i, j):
    names = {0: "Beacon", 1: "Drone 1", 2: "Drone 2"}
    return f"{names[i]} → {names[j]}"


def compute_gt(df, i, j):
    """Ground truth relative state of j w.r.t. i from mocap columns."""
    xi = df[f"mc{i}_cf_x"].values
    yi = df[f"mc{i}_cf_y"].values
    zi = df[f"mc{i}_cf_z"].values
    xj = df[f"mc{j}_cf_x"].values
    yj = df[f"mc{j}_cf_y"].values
    zj = df[f"mc{j}_cf_z"].values
    yaw_i = df[f"mc{i}_yaw"].values
    yaw_j = df[f"mc{j}_yaw"].values

    dx, dy, dz = xj - xi, yj - yi, zj - zi
    c, s = np.cos(yaw_i), np.sin(yaw_i)

    return {
        "x12":   c * dx + s * dy,
        "y12":  -s * dx + c * dy,
        "z12":   dz,
        "psi12": np.unwrap(yaw_j) - np.unwrap(yaw_i),
        "dist":  np.sqrt(dx**2 + dy**2 + dz**2),
    }


def mocap_body_vel_and_yawrate(df, drone_id, time):
    """Derive body-frame velocities and yaw rate from mocap position+yaw.

    Returns (vx_body, vy_body, vz_body, omega_yaw) at each timestep.
    Uses central differences (forward/backward at edges).
    """
    x = df[f"mc{drone_id}_cf_x"].values
    y = df[f"mc{drone_id}_cf_y"].values
    z = df[f"mc{drone_id}_cf_z"].values
    yaw = np.unwrap(df[f"mc{drone_id}_yaw"].values)

    dt = np.gradient(time)
    dx = np.gradient(x) / dt   # global-frame velocities
    dy = np.gradient(y) / dt
    dz = np.gradient(z) / dt
    omega = np.gradient(yaw) / dt  # yaw rate

    # Rotate global velocities into body frame
    c, s = np.cos(yaw), np.sin(yaw)
    vx_body =  c * dx + s * dy
    vy_body = -s * dx + c * dy

    return vx_body, vy_body, dz, omega


# ──────────────────────────────────────────────────────────────────────
# Data extraction
# ──────────────────────────────────────────────────────────────────────

def extract_pair_data(df, i, j):
    """Extract EKF inputs for pair i→j.

    - Own state for drone i: gyro yaw rate, Kalman velocity, Kalman height
    - Communicated state from j: yawR, v, height (from drone i's SD log)
    - UWB range from i to j
    - Drone 0 (beacon): forced to zero yaw rate, velocity, height

    Note: d{i}_height{i} is always 0 (firmware doesn't self-populate
    ranging height), so we use d{i}_kalman_stateZ for own height.
    """
    N = len(df)

    # Drone i (observer) — own state
    if i == STATIC_DRONE:
        omega_i = np.zeros(N)
        vxi, vyi, vzi = np.zeros(N), np.zeros(N), np.zeros(N)
        h_i = np.zeros(N)
    else:
        omega_i = df[f"d{i}_rateYaw"].values / 1000.0      # millirad/s → rad/s
        vxi = df[f"d{i}_kalman_statePX"].values
        vyi = df[f"d{i}_kalman_statePY"].values
        vzi = df[f"d{i}_kalman_statePZ"].values
        h_i = df[f"d{i}_kalman_stateZ"].values              # own height

    # Drone j (target) — communicated to i via UWB
    if j == STATIC_DRONE:
        omega_j = np.zeros(N)
        vxj, vyj, vzj = np.zeros(N), np.zeros(N), np.zeros(N)
        h_j = np.zeros(N)
    else:
        omega_j = np.deg2rad(df[f"d{i}_yawR{j}"].values)   # deg/s → rad/s
        vxj = df[f"d{i}_vx{j}"].values
        vyj = df[f"d{i}_vy{j}"].values
        vzj = df[f"d{i}_vz{j}"].values
        h_j = df[f"d{i}_height{j}"].values                  # communicated height

    d_meas = df[f"d{i}_distance{j}"].values / 1000.0        # mm → m
    z12 = h_j - h_i

    return {
        "omega_i": omega_i, "vxi": vxi, "vyi": vyi, "vzi": vzi,
        "omega_j": omega_j, "vxj": vxj, "vyj": vyj, "vzj": vzj,
        "d_meas": d_meas, "z12": z12,
        "gt": compute_gt(df, i, j),
    }


def extract_mocap_pair_data(df, i, j, time):
    """Extract EKF inputs for pair i→j using mocap as control inputs.

    Body-frame velocities and yaw rates are derived from mocap
    finite differences.  UWB range is still used as the measurement.
    """
    vxi, vyi, vzi, omega_i = mocap_body_vel_and_yawrate(df, i, time)
    vxj, vyj, vzj, omega_j = mocap_body_vel_and_yawrate(df, j, time)

    h_i = df[f"mc{i}_cf_z"].values
    h_j = df[f"mc{j}_cf_z"].values

    d_meas = df[f"d{i}_distance{j}"].values / 1000.0   # mm → m
    z12 = h_j - h_i

    return {
        "omega_i": omega_i, "vxi": vxi, "vyi": vyi, "vzi": vzi,
        "omega_j": omega_j, "vxj": vxj, "vyj": vyj, "vzj": vzj,
        "d_meas": d_meas, "z12": z12,
        "gt": compute_gt(df, i, j),
    }


# ──────────────────────────────────────────────────────────────────────
# EKF runner
# ──────────────────────────────────────────────────────────────────────

def run_3state_ekf(time, data, Q_override=None, R_override=None):
    """Run 3-state EKF on provided data dict."""
    N = len(time)
    dt_arr = np.diff(time)
    dt_med = float(np.median(dt_arr))

    Q_use = Q_override if Q_override is not None else Q
    R_use = R_override if R_override is not None else R

    omega_i = data["omega_i"]
    omega_j = data["omega_j"]
    vxi, vyi, vzi = data["vxi"], data["vyi"], data["vzi"]
    vxj, vyj, vzj = data["vxj"], data["vyj"], data["vzj"]
    d_meas = data["d_meas"]
    z12 = data["z12"]

    x0 = np.array([0.0, d_meas[0], 0.0])
    u0 = np.array([omega_i[0], vxi[0], vyi[0], vzi[0],
                   omega_j[0], vxj[0], vyj[0], vzj[0], z12[0]])

    ekf = EKF(f_3state, h_3state, x0, u0,
              P0.copy(), Q_use.copy(), R_use.copy(),
              dynamics_type="continuous",
              discretization_timestep=dt_med)

    for k in range(1, N):
        dt_k = dt_arr[k - 1] if k - 1 < len(dt_arr) else dt_med
        u_k = np.array([omega_i[k], vxi[k], vyi[k], vzi[k],
                        omega_j[k], vxj[k], vyj[k], vzj[k], z12[k]])
        ekf.forward_update(np.array([d_meas[k]]), u_k,
                           discretization_timestep=dt_k)

    X = np.array(ekf.history["X"])
    P = np.array(ekf.history["P_diags"])
    return X, P


# ──────────────────────────────────────────────────────────────────────
# Plotting
# ──────────────────────────────────────────────────────────────────────

def plot_comparison(time, X_onboard, P_onboard, X_mocap, P_mocap, data, i, j):
    gt = data["gt"]
    d_meas = data["d_meas"]
    z12 = data["z12"]
    label = pair_label(i, j)

    fig, axes = plt.subplots(3, 1, figsize=(16, 12), sharex=True)
    fig.suptitle(f"Day 02: 3-State EKF — {label}", fontsize=14, fontweight="bold")

    state_info = [
        (1, "x₁₂ [m]", "x12", False),
        (2, "y₁₂ [m]", "y12", False),
        (0, "ψ₁₂ [deg]", "psi12", True),
    ]

    for ax, (si, ylabel, gt_key, is_angle) in zip(axes, state_info):
        gt_vals = np.rad2deg(gt[gt_key]) if is_angle else gt[gt_key]
        onb = np.rad2deg(X_onboard[:, si]) if is_angle else X_onboard[:, si]
        mc  = np.rad2deg(X_mocap[:, si])   if is_angle else X_mocap[:, si]
        scale = np.rad2deg(1.0) if is_angle else 1.0

        ax.plot(time, gt_vals, "k-", lw=1.5, label="Mocap GT")
        ax.plot(time, onb, "b-", alpha=0.7, lw=0.8, label="Onboard inputs")
        ax.fill_between(time,
                        onb - 2*scale*np.sqrt(np.abs(P_onboard[:, si])),
                        onb + 2*scale*np.sqrt(np.abs(P_onboard[:, si])),
                        color="b", alpha=0.08)
        ax.plot(time, mc, "r-", alpha=0.7, lw=0.8, label="Mocap inputs")
        ax.fill_between(time,
                        mc - 2*scale*np.sqrt(np.abs(P_mocap[:, si])),
                        mc + 2*scale*np.sqrt(np.abs(P_mocap[:, si])),
                        color="r", alpha=0.08)
        ax.set_ylabel(ylabel); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time [s]")
    plt.tight_layout()
    fname = f"day02_ekf3_{i}{j}.png"
    fig.savefig(OUTPUT_DIR / fname, dpi=150)
    plt.close(fig)
    print(f"  Saved {fname}")


def plot_xy(time, X_onboard, X_mocap, data, i, j):
    gt = data["gt"]
    label = pair_label(i, j)

    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    ax.set_title(f"Day 02: 3-State XY — {label}", fontsize=13)
    ax.plot(gt["x12"], gt["y12"], "k-", lw=2, label="Mocap GT", alpha=0.7)
    ax.plot(X_onboard[:, 1], X_onboard[:, 2], "b-", lw=1,
            label="Onboard inputs", alpha=0.5)
    ax.plot(X_mocap[:, 1], X_mocap[:, 2], "r-", lw=1,
            label="Mocap inputs", alpha=0.5)
    ax.plot(gt["x12"][0], gt["y12"][0], "ko", ms=10, label="GT start")
    ax.set_xlabel("x₁₂ [m]"); ax.set_ylabel("y₁₂ [m]")
    ax.legend(); ax.grid(True, alpha=0.3); ax.set_aspect("equal")
    plt.tight_layout()
    fname = f"day02_ekf3_{i}{j}_xy.png"
    fig.savefig(OUTPUT_DIR / fname, dpi=150)
    plt.close(fig)
    print(f"  Saved {fname}")


# ──────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────

def plot_tuning_sweep(time, results, data, i, j, suffix=""):
    """Plot EKF results for multiple Q/R tuning configs on one figure."""
    gt = data["gt"]
    label = pair_label(i, j)

    fig, axes = plt.subplots(3, 1, figsize=(16, 12), sharex=True)
    fig.suptitle(f"Q/R Tuning Sweep — {label}", fontsize=14, fontweight="bold")

    state_info = [
        (1, "x₁₂ [m]", "x12", False),
        (2, "y₁₂ [m]", "y12", False),
        (0, "ψ₁₂ [deg]", "psi12", True),
    ]

    colors = plt.cm.tab10(np.linspace(0, 1, len(results)))

    for ax, (si, ylabel, gt_key, is_angle) in zip(axes, state_info):
        gt_vals = np.rad2deg(gt[gt_key]) if is_angle else gt[gt_key]
        ax.plot(time, gt_vals, "k-", lw=2, label="Mocap GT")

        for (cfg_label, X, P), color in zip(results, colors):
            vals = np.rad2deg(X[:, si]) if is_angle else X[:, si]
            ax.plot(time, vals, color=color, alpha=0.7, lw=0.8, label=cfg_label)

        ax.set_ylabel(ylabel); ax.legend(fontsize=7, ncol=2); ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time [s]")
    plt.tight_layout()
    fname = f"day02_tuning_{i}{j}{suffix}.png"
    fig.savefig(OUTPUT_DIR / fname, dpi=150)
    plt.close(fig)
    print(f"  Saved {fname}")


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    print(f"Loading synced data from {DATA_CSV}...")
    df = pd.read_csv(DATA_CSV)
    time = df["time"].values
    print(f"  {len(df)} rows, {len(df.columns)} cols")

    # ── Tuning sweep on mocap inputs (pair 0→1) ──────────────────────
    TUNING_CONFIGS = [
        ("Original Q, R=0.1²",
         np.diag([1e-2, 1e-1, 1e-1]), np.array([[0.10**2]])),
        ("Q×10, R=0.1²",
         np.diag([1e-1, 1.0, 1.0]),   np.array([[0.10**2]])),
        ("Q×100, R=0.1²",
         np.diag([1.0, 10.0, 10.0]),  np.array([[0.10**2]])),
        ("Q×10, R=0.05²",
         np.diag([1e-1, 1.0, 1.0]),   np.array([[0.05**2]])),
        ("Q×100, R=0.05²",
         np.diag([1.0, 10.0, 10.0]),  np.array([[0.05**2]])),
        ("Q×100, R=0.02²",
         np.diag([1.0, 10.0, 10.0]),  np.array([[0.02**2]])),
    ]

    for test_pair in [(0, 1), (0, 2)]:
        i, j = test_pair
        label = pair_label(i, j)
        print(f"\n{'='*60}")
        print(f"  Tuning sweep: {label} (mocap inputs)")
        print(f"{'='*60}")

        data_mocap = extract_mocap_pair_data(df, i, j, time)
        results = []
        for cfg_label, Q_cfg, R_cfg in TUNING_CONFIGS:
            print(f"  {cfg_label}...")
            X, P = run_3state_ekf(time, data_mocap,
                                  Q_override=Q_cfg, R_override=R_cfg)
            results.append((cfg_label, X, P))

        plot_tuning_sweep(time, results, data_mocap, i, j, suffix="_mocap")

    # ── Also sweep on onboard inputs for same pairs ──────────────────
    for test_pair in [(0, 1), (0, 2)]:
        i, j = test_pair
        label = pair_label(i, j)
        print(f"\n{'='*60}")
        print(f"  Tuning sweep: {label} (onboard inputs)")
        print(f"{'='*60}")

        data_onboard = extract_pair_data(df, i, j)
        results = []
        for cfg_label, Q_cfg, R_cfg in TUNING_CONFIGS:
            print(f"  {cfg_label}...")
            X, P = run_3state_ekf(time, data_onboard,
                                  Q_override=Q_cfg, R_override=R_cfg)
            results.append((cfg_label, X, P))

        plot_tuning_sweep(time, results, data_onboard, i, j, suffix="_onboard")

    print("\nDone!")


if __name__ == "__main__":
    main()
