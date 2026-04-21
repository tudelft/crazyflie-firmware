#!/usr/bin/env python3
"""
Run relative-position EKFs on day_02 synced data.

Uses synced_all.csv (50 Hz) produced by process_day02_data.py.
Drone 0 is a static ground beacon (zero yaw rate, velocity, height).

Pairs:
  - Beacon (0) → Drone 1
  - Beacon (0) → Drone 2
  - Drone 1 → Drone 2
"""

from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from ekf_utils import EKF, wrapToPi

# ──────────────────────────────────────────────────────────────────────
# Configuration
# ──────────────────────────────────────────────────────────────────────

DATA_CSV   = Path("relative_data/day_02/synced_all.csv")
OUTPUT_DIR = Path("relative_data_plots/day02")

STATIC_DRONE = 0

# EKF tuning (same as run_real_relative_ekf.py)
Q_4 = np.diag([1e-2, 1e-1, 1e-1, 5e-2])
Q_3 = np.diag([1e-2, 1e-1, 1e-1])
R   = np.array([[0.10**2]])

P0_4 = np.diag([1.0**2, 2.0**2, 2.0**2, 1.0**2])
P0_3 = np.diag([1.0**2, 2.0**2, 2.0**2])


# ──────────────────────────────────────────────────────────────────────
# Dynamics & measurement models
# ──────────────────────────────────────────────────────────────────────

def f_4state(x, u):
    psi12, x12, y12, z12 = x
    psi_dot_i, vxi, vyi, vzi, psi_dot_j, vxj, vyj, vzj = u[:8]
    c, s = np.cos(psi12), np.sin(psi12)
    return np.array([
        psi_dot_j - psi_dot_i,
        (c * vxj - s * vyj) - vxi + psi_dot_i * y12,
        (s * vxj + c * vyj) - vyi - psi_dot_i * x12,
        vzj - vzi,
    ])


def h_4state(x, _u):
    _, x12, y12, z12 = x
    d = np.sqrt(x12**2 + y12**2 + z12**2)
    return np.array([max(d, 1e-8)])


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
# Data extraction from synced CSV
# ──────────────────────────────────────────────────────────────────────

def extract_pair_data(df, i, j):
    """Extract EKF inputs for pair i→j from synced CSV.

    Returns dict with time, omega_i, vxi, vyi, vzi, omega_j, vxj, vyj, vzj,
    d_meas, z12, and ground truth arrays.

    Units in returned dict are all SI (rad/s, m/s, m).
    """
    time = df["time"].values

    if i == STATIC_DRONE:
        # Beacon: zero yaw rate, velocity, height
        N = len(time)
        omega_i = np.zeros(N)
        vxi = np.zeros(N)
        vyi = np.zeros(N)
        vzi = np.zeros(N)
        h_i = 0.0

        # Communicated data from j, as seen by drone i
        omega_j = np.deg2rad(df[f"d{i}_yawR{j}"].values)    # deg/s → rad/s
        vxj = df[f"d{i}_vx{j}"].values                       # m/s
        vyj = df[f"d{i}_vy{j}"].values
        vzj = df[f"d{i}_vz{j}"].values

        d_meas = df[f"d{i}_distance{j}"].values / 1000.0     # mm → m
        z12 = df[f"d{i}_height{j}"].values - h_i             # m

    else:
        # Flying drone i: own measurements
        omega_i = df[f"d{i}_rateYaw"].values / 1000.0         # millirad/s → rad/s
        vxi = df[f"d{i}_kalman_statePX"].values               # m/s
        vyi = df[f"d{i}_kalman_statePY"].values
        vzi = df[f"d{i}_kalman_statePZ"].values

        if j == STATIC_DRONE:
            # j is beacon: zero yaw rate, velocity, height
            N = len(time)
            omega_j = np.zeros(N)
            vxj = np.zeros(N)
            vyj = np.zeros(N)
            vzj = np.zeros(N)
            h_j = 0.0
            d_meas = df[f"d{i}_distance{j}"].values / 1000.0
            z12 = h_j - df[f"d{i}_height{i}"].values  # z_j - z_i, j is on ground
        else:
            # Both flying: communicated data from j
            omega_j = np.deg2rad(df[f"d{i}_yawR{j}"].values)
            vxj = df[f"d{i}_vx{j}"].values
            vyj = df[f"d{i}_vy{j}"].values
            vzj = df[f"d{i}_vz{j}"].values

            d_meas = df[f"d{i}_distance{j}"].values / 1000.0
            z12 = df[f"d{i}_height{j}"].values - df[f"d{i}_height{i}"].values

    # Ground truth (already computed in synced CSV)
    gt = {
        "x12":   df[f"mc_rel_{i}{j}_x"].values,
        "y12":   df[f"mc_rel_{i}{j}_y"].values,
        "z12":   df[f"mc_rel_{i}{j}_z"].values,
        "psi12": df[f"mc_rel_{i}{j}_yaw"].values,  # radians
        "dist":  df[f"mc_dist_{i}{j}"].values,
    }

    return {
        "time": time,
        "omega_i": omega_i, "vxi": vxi, "vyi": vyi, "vzi": vzi,
        "omega_j": omega_j, "vxj": vxj, "vyj": vyj, "vzj": vzj,
        "d_meas": d_meas, "z12": z12,
        "gt": gt,
    }


# ──────────────────────────────────────────────────────────────────────
# EKF runner
# ──────────────────────────────────────────────────────────────────────

def run_ekf_pair(data, label=""):
    """Run 4-state and 3-state EKF on extracted pair data."""
    time = data["time"]
    N = len(time)
    dt_arr = np.diff(time)
    dt_med = float(np.median(dt_arr))

    omega_i = data["omega_i"]
    vxi, vyi, vzi = data["vxi"], data["vyi"], data["vzi"]
    omega_j = data["omega_j"]
    vxj, vyj, vzj = data["vxj"], data["vyj"], data["vzj"]
    d_meas = data["d_meas"]
    z12 = data["z12"]

    d0 = d_meas[0]
    print(f"  [{label}] N={N}, dt_med={dt_med:.4f}s, d0={d0:.2f}m")

    # 4-state EKF
    x0_4 = np.array([0.0, d0, 0.0, z12[0]])
    u0_4 = np.array([omega_i[0], vxi[0], vyi[0], vzi[0],
                     omega_j[0], vxj[0], vyj[0], vzj[0]])

    ekf4 = EKF(f_4state, h_4state, x0_4, u0_4,
               P0_4.copy(), Q_4.copy(), R.copy(),
               dynamics_type="continuous",
               discretization_timestep=dt_med)

    for k in range(1, N):
        dt_k = dt_arr[k - 1] if k - 1 < len(dt_arr) else dt_med
        u_k = np.array([omega_i[k], vxi[k], vyi[k], vzi[k],
                        omega_j[k], vxj[k], vyj[k], vzj[k]])
        ekf4.forward_update(np.array([d_meas[k]]), u_k,
                            discretization_timestep=dt_k)

    X4 = np.array(ekf4.history["X"])
    P4 = np.array(ekf4.history["P_diags"])

    # 3-state EKF
    x0_3 = np.array([0.0, d0, 0.0])
    u0_3 = np.array([omega_i[0], vxi[0], vyi[0], vzi[0],
                     omega_j[0], vxj[0], vyj[0], vzj[0],
                     z12[0]])

    ekf3 = EKF(f_3state, h_3state, x0_3, u0_3,
               P0_3.copy(), Q_3.copy(), R.copy(),
               dynamics_type="continuous",
               discretization_timestep=dt_med)

    for k in range(1, N):
        dt_k = dt_arr[k - 1] if k - 1 < len(dt_arr) else dt_med
        u_k = np.array([omega_i[k], vxi[k], vyi[k], vzi[k],
                        omega_j[k], vxj[k], vyj[k], vzj[k],
                        z12[k]])
        ekf3.forward_update(np.array([d_meas[k]]), u_k,
                            discretization_timestep=dt_k)

    X3 = np.array(ekf3.history["X"])
    P3 = np.array(ekf3.history["P_diags"])

    return X4, P4, X3, P3


# ──────────────────────────────────────────────────────────────────────
# Plotting
# ──────────────────────────────────────────────────────────────────────

def plot_ekf_results(time, X4, P4, X3, P3, gt, d_meas, z12, label, fname):
    fig, axes = plt.subplots(5, 1, figsize=(16, 18), sharex=True)
    fig.suptitle(f"Day 02 Relative EKF — {label}", fontsize=14, fontweight="bold")

    # x12
    ax = axes[0]
    ax.plot(time, gt["x12"], "k-", lw=1.5, label="Mocap GT")
    ax.plot(time, X4[:, 1], "b-", alpha=0.8, label="4-state")
    ax.fill_between(time,
                    X4[:, 1] - 2*np.sqrt(np.abs(P4[:, 1])),
                    X4[:, 1] + 2*np.sqrt(np.abs(P4[:, 1])),
                    color="b", alpha=0.12)
    ax.plot(time, X3[:, 1], "r--", alpha=0.8, label="3-state")
    ax.fill_between(time,
                    X3[:, 1] - 2*np.sqrt(np.abs(P3[:, 1])),
                    X3[:, 1] + 2*np.sqrt(np.abs(P3[:, 1])),
                    color="r", alpha=0.12)
    ax.set_ylabel("x₁₂ [m]"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # y12
    ax = axes[1]
    ax.plot(time, gt["y12"], "k-", lw=1.5, label="Mocap GT")
    ax.plot(time, X4[:, 2], "b-", alpha=0.8, label="4-state")
    ax.fill_between(time,
                    X4[:, 2] - 2*np.sqrt(np.abs(P4[:, 2])),
                    X4[:, 2] + 2*np.sqrt(np.abs(P4[:, 2])),
                    color="b", alpha=0.12)
    ax.plot(time, X3[:, 2], "r--", alpha=0.8, label="3-state")
    ax.fill_between(time,
                    X3[:, 2] - 2*np.sqrt(np.abs(P3[:, 2])),
                    X3[:, 2] + 2*np.sqrt(np.abs(P3[:, 2])),
                    color="r", alpha=0.12)
    ax.set_ylabel("y₁₂ [m]"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # z12
    ax = axes[2]
    ax.plot(time, gt["z12"], "k-", lw=1.5, label="Mocap GT")
    ax.plot(time, X4[:, 3], "b-", alpha=0.8, label="4-state est.")
    ax.fill_between(time,
                    X4[:, 3] - 2*np.sqrt(np.abs(P4[:, 3])),
                    X4[:, 3] + 2*np.sqrt(np.abs(P4[:, 3])),
                    color="b", alpha=0.12)
    ax.plot(time, z12, "r--", alpha=0.8, label="3-state (known Δh)")
    ax.set_ylabel("z₁₂ [m]"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # psi12
    ax = axes[3]
    ax.plot(time, np.rad2deg(gt["psi12"]), "k-", lw=1.5, label="Mocap GT")
    ax.plot(time, np.rad2deg(X4[:, 0]), "b-", alpha=0.8, label="4-state")
    ax.fill_between(time,
                    np.rad2deg(X4[:, 0] - 2*np.sqrt(np.abs(P4[:, 0]))),
                    np.rad2deg(X4[:, 0] + 2*np.sqrt(np.abs(P4[:, 0]))),
                    color="b", alpha=0.12)
    ax.plot(time, np.rad2deg(X3[:, 0]), "r--", alpha=0.8, label="3-state")
    ax.fill_between(time,
                    np.rad2deg(X3[:, 0] - 2*np.sqrt(np.abs(P3[:, 0]))),
                    np.rad2deg(X3[:, 0] + 2*np.sqrt(np.abs(P3[:, 0]))),
                    color="r", alpha=0.12)
    ax.set_ylabel("ψ₁₂ [deg]"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # distance
    ax = axes[4]
    d_est4 = np.sqrt(X4[:, 1]**2 + X4[:, 2]**2 + X4[:, 3]**2)
    d_est3 = np.sqrt(X3[:, 1]**2 + X3[:, 2]**2 + z12**2)
    ax.plot(time, gt["dist"], "k-", lw=1.5, label="Mocap GT dist")
    ax.plot(time, d_meas, "g.", ms=1, alpha=0.15, label="UWB measured")
    ax.plot(time, d_est4, "b-", alpha=0.8, label="4-state")
    ax.plot(time, d_est3, "r--", alpha=0.8, label="3-state")
    ax.set_ylabel("Distance [m]"); ax.set_xlabel("Time [s]")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    fig.savefig(OUTPUT_DIR / fname, dpi=150)
    plt.close(fig)
    print(f"  Saved {fname}")


def plot_xy_trajectory(time, X4, X3, gt, label, fname):
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    ax.set_title(f"Day 02 Relative Position (XY) — {label}", fontsize=13)
    ax.plot(gt["x12"], gt["y12"], "k-", lw=2, label="Mocap GT", alpha=0.7)
    ax.plot(X4[:, 1], X4[:, 2], "b-", lw=1, label="4-state", alpha=0.7)
    ax.plot(X3[:, 1], X3[:, 2], "r--", lw=1, label="3-state", alpha=0.7)
    ax.plot(gt["x12"][0], gt["y12"][0], "ko", ms=10, label="GT start")
    ax.plot(X4[0, 1], X4[0, 2], "bs", ms=10, label="EKF start")
    ax.set_xlabel("x₁₂ [m]"); ax.set_ylabel("y₁₂ [m]")
    ax.legend(); ax.grid(True, alpha=0.3); ax.set_aspect("equal")
    plt.tight_layout()
    fig.savefig(OUTPUT_DIR / fname, dpi=150)
    plt.close(fig)
    print(f"  Saved {fname}")


# ──────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────

def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    print(f"Loading synced data from {DATA_CSV}...")
    df = pd.read_csv(DATA_CSV)
    print(f"  {len(df)} rows, {len(df.columns)} cols")

    pairs = [
        (0, 1, "Beacon → Drone 1"),
        (0, 2, "Beacon → Drone 2"),
        (1, 2, "Drone 1 → Drone 2"),
    ]

    for i, j, label in pairs:
        print(f"\n{'='*60}")
        print(f"  {label}")
        print(f"{'='*60}")

        data = extract_pair_data(df, i, j)

        # Print input diagnostics
        print(f"  d_meas range: [{data['d_meas'].min():.2f}, {data['d_meas'].max():.2f}] m")
        print(f"  omega_i range: [{data['omega_i'].min():.2f}, {data['omega_i'].max():.2f}] rad/s")
        print(f"  omega_j range: [{data['omega_j'].min():.2f}, {data['omega_j'].max():.2f}] rad/s")
        print(f"  z12 range: [{data['z12'].min():.2f}, {data['z12'].max():.2f}] m")

        X4, P4, X3, P3 = run_ekf_pair(data, label=label)

        tag = f"{i}{j}"
        plot_ekf_results(data["time"], X4, P4, X3, P3,
                         data["gt"], data["d_meas"], data["z12"],
                         label, f"day02_ekf_{tag}.png")
        plot_xy_trajectory(data["time"], X4, X3, data["gt"],
                           label, f"day02_ekf_{tag}_xy.png")

    print("\nDone!")


if __name__ == "__main__":
    main()
