#!/usr/bin/env python3
"""
Run relative-position EKFs on real Flapper drone data.

Uses:
  - Drone 0 (beacon/ground) SD log for UWB ranges, communicated velocities & yaw rates
  - OptiTrack mocap CSV for ground truth
  - Sync: mocap_time = drone0_relative_time + LAG

Pairs:
  - Beacon → Drone 1 (flapper_01)
  - Beacon → Drone 2 (flapper_02)
  - Drone 1 → Drone 2 (flapper_01 → flapper_02)
"""

from __future__ import annotations

import csv
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, "tools/usdlog")
import cfusdlog
from ekf_utils import EKF, wrapToPi

# ──────────────────────────────────────────────────────────────────────
# Configuration
# ──────────────────────────────────────────────────────────────────────
DATA_DIR   = Path("relative_data")
OUTPUT_DIR = Path("relative_data_plots")
OUTPUT_DIR.mkdir(exist_ok=True)

LAG = 11.58            # mocap_time = drone0_rel_time + LAG
DOWNSAMPLE = 10        # 500 Hz → 50 Hz

# Mapping: SD1 = flapper_01, SD2 = flapper_02
# Mocap column indices (0-indexed):
#   flapper_00: qx=2,qy=3,qz=4,qw=5, px=6,py=7,pz=8
#   flapper_01: qx=9,qy=10,qz=11,qw=12, px=13,py=14,pz=15
#   flapper_02: qx=16,qy=17,qz=18,qw=19, px=20,py=21,pz=22
# OT→CF frame: cf_x=ot_z, cf_y=ot_x, cf_z=ot_y
#              cf_qx=ot_qz, cf_qy=ot_qx, cf_qz=ot_qy, cf_qw=ot_qw

MOCAP_COLS = {
    0: {"px": 8, "py": 6, "pz": 7, "qx": 4, "qy": 2, "qz": 3, "qw": 5},
    1: {"px": 15, "py": 13, "pz": 14, "qx": 11, "qy": 9, "qz": 10, "qw": 12},
    2: {"px": 22, "py": 20, "pz": 21, "qx": 18, "qy": 16, "qz": 17, "qw": 19},
}

# EKF tuning
Q_4 = np.diag([1e-2, 1e-1, 1e-1, 5e-2])     # process noise (4-state)
Q_3 = np.diag([1e-2, 1e-1, 1e-1])            # process noise (3-state)
R   = np.array([[0.10**2]])                    # range measurement noise (UWB)

P0_4 = np.diag([1.0**2, 2.0**2, 2.0**2, 1.0**2])
P0_3 = np.diag([1.0**2, 2.0**2, 2.0**2])


# ──────────────────────────────────────────────────────────────────────
# Dynamics & measurement models (same as run_relative_ekf.py)
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
# Data loading
# ──────────────────────────────────────────────────────────────────────

def load_drone0():
    """Load beacon SD log. Returns dict with arrays on drone0's clock."""
    d = cfusdlog.decode(str(DATA_DIR / "flap0log00"))["fixedFrequency"]
    ts = np.array(d["timestamp"]) / 1000.0    # ms → s
    return {
        "time": ts,
        "time_rel": ts - ts[0],
        # Beacon inputs (stationary)
        "omega_beacon": np.deg2rad(np.array(d["stateEstimateZ.rateYaw"])),
        # Communicated data from drone 1
        "vx1": np.array(d["ranging.vx1"]),
        "vy1": np.array(d["ranging.vy1"]),
        "vz1": np.array(d["ranging.vz1"]),
        "omega1": np.deg2rad(np.array(d["ranging.yawR1"])),
        "height1": np.array(d["ranging.height1"]),
        # Communicated data from drone 2
        "vx2": np.array(d["ranging.vx2"]),
        "vy2": np.array(d["ranging.vy2"]),
        "vz2": np.array(d["ranging.vz2"]),
        "omega2": np.deg2rad(np.array(d["ranging.yawR2"])),
        "height2": np.array(d["ranging.height2"]),
        # UWB ranges
        "d01": np.array(d["ranging.distance1"]) / 1000.0,  # mm → m
        "d02": np.array(d["ranging.distance2"]) / 1000.0,
        "d12": np.array(d["ranging.inD12"]) / 1000.0,
    }


def load_mocap():
    """Load mocap CSV. Returns dict with time and per-body pos/yaw arrays."""
    rows = list(csv.reader(open(DATA_DIR / "good_sd.csv")))
    data = rows[7:]
    N = len(data)
    mt = np.zeros(N)
    bodies = {}
    for bid, cols in MOCAP_COLS.items():
        px = np.full(N, np.nan)
        py = np.full(N, np.nan)
        pz = np.full(N, np.nan)
        yaw = np.full(N, np.nan)
        for i, r in enumerate(data):
            mt[i] = float(r[1])
            try:
                px[i] = float(r[cols["px"]])
                py[i] = float(r[cols["py"]])
                pz[i] = float(r[cols["pz"]])
                qx = float(r[cols["qx"]])
                qy = float(r[cols["qy"]])
                qz = float(r[cols["qz"]])
                qw = float(r[cols["qw"]])
                # Yaw from quaternion (CF frame)
                yaw[i] = np.arctan2(2 * (qw * qz + qx * qy),
                                     1 - 2 * (qy**2 + qz**2))
            except (ValueError, IndexError):
                pass
        bodies[bid] = {"x": px, "y": py, "z": pz, "yaw": yaw}
    return {"time": mt, "bodies": bodies}


def interp_mocap_to_drone(mocap, body_id, drone_time_rel):
    """Interpolate mocap body to drone0's relative timestamps via LAG.

    Returns dict with x, y, z, yaw arrays (NaN where mocap unavailable).
    """
    mt = mocap["time"]
    b = mocap["bodies"][body_id]
    t_mocap = drone_time_rel + LAG

    result = {}
    for key in ["x", "y", "z"]:
        valid = ~np.isnan(b[key])
        result[key] = np.interp(t_mocap, mt[valid], b[key][valid],
                                left=np.nan, right=np.nan)

    # Yaw needs unwrap before interp
    valid_yaw = ~np.isnan(b["yaw"])
    yaw_unwrapped = np.unwrap(b["yaw"][valid_yaw])
    result["yaw"] = np.interp(t_mocap, mt[valid_yaw], yaw_unwrapped,
                               left=np.nan, right=np.nan)
    return result


def compute_ground_truth(mocap_i, mocap_j):
    """Compute body-frame relative state of j w.r.t. i from mocap.

    Returns psi12, x12, y12, z12, d12 arrays.
    """
    dx = mocap_j["x"] - mocap_i["x"]
    dy = mocap_j["y"] - mocap_i["y"]
    dz = mocap_j["z"] - mocap_i["z"]
    psi_i = mocap_i["yaw"]

    # World → body frame of drone i
    c = np.cos(psi_i)
    s = np.sin(psi_i)
    x12 =  c * dx + s * dy
    y12 = -s * dx + c * dy
    z12 = dz

    psi12 = wrapToPi(mocap_j["yaw"] - mocap_i["yaw"])
    d12 = np.sqrt(dx**2 + dy**2 + dz**2)

    return psi12, x12, y12, z12, d12


# ──────────────────────────────────────────────────────────────────────
# EKF runner
# ──────────────────────────────────────────────────────────────────────

def run_ekf_pair(time_rel, omega_i, vxi, vyi, vzi,
                 omega_j, vxj, vyj, vzj,
                 d_meas, z12_known, label=""):
    """Run 4-state and 3-state EKF on a drone pair.

    All inputs should be aligned arrays at DOWNSAMPLE'd rate.
    """
    N = len(time_rel)
    dt_arr = np.diff(time_rel)
    dt_med = float(np.median(dt_arr))

    # Initial guess from first range measurement
    d0 = d_meas[0]
    x0_4 = np.array([0.0, d0, 0.0, z12_known[0]])
    u0_4 = np.array([omega_i[0], vxi[0], vyi[0], vzi[0],
                     omega_j[0], vxj[0], vyj[0], vzj[0]])

    print(f"  [{label}] N={N}, dt_med={dt_med:.4f}s, d0={d0:.2f}m")

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

    # 3-state
    x0_3 = np.array([0.0, d0, 0.0])
    u0_3 = np.array([omega_i[0], vxi[0], vyi[0], vzi[0],
                     omega_j[0], vxj[0], vyj[0], vzj[0],
                     z12_known[0]])

    ekf3 = EKF(f_3state, h_3state, x0_3, u0_3,
               P0_3.copy(), Q_3.copy(), R.copy(),
               dynamics_type="continuous",
               discretization_timestep=dt_med)

    for k in range(1, N):
        dt_k = dt_arr[k - 1] if k - 1 < len(dt_arr) else dt_med
        u_k = np.array([omega_i[k], vxi[k], vyi[k], vzi[k],
                        omega_j[k], vxj[k], vyj[k], vzj[k],
                        z12_known[k]])
        ekf3.forward_update(np.array([d_meas[k]]), u_k,
                            discretization_timestep=dt_k)

    X3 = np.array(ekf3.history["X"])
    P3 = np.array(ekf3.history["P_diags"])

    return X4, P4, X3, P3


# ──────────────────────────────────────────────────────────────────────
# Plotting
# ──────────────────────────────────────────────────────────────────────

def plot_ekf_results(time, X4, P4, X3, P3, gt, d_meas, z12_known, label, fname):
    psi12_gt, x12_gt, y12_gt, z12_gt, d_gt = gt
    valid = ~np.isnan(x12_gt)

    fig, axes = plt.subplots(5, 1, figsize=(16, 18), sharex=True)
    fig.suptitle(f"Relative EKF — {label}", fontsize=14, fontweight="bold")

    # x12
    ax = axes[0]
    ax.plot(time[valid], x12_gt[valid], "k-", lw=1.5, label="Mocap GT")
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
    ax.plot(time[valid], y12_gt[valid], "k-", lw=1.5, label="Mocap GT")
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
    ax.plot(time[valid], z12_gt[valid], "k-", lw=1.5, label="Mocap GT")
    ax.plot(time, X4[:, 3], "b-", alpha=0.8, label="4-state est.")
    ax.fill_between(time,
                    X4[:, 3] - 2*np.sqrt(np.abs(P4[:, 3])),
                    X4[:, 3] + 2*np.sqrt(np.abs(P4[:, 3])),
                    color="b", alpha=0.12)
    ax.plot(time, z12_known, "r--", alpha=0.8, label="3-state (known)")
    ax.set_ylabel("z₁₂ [m]"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # psi12
    ax = axes[3]
    ax.plot(time[valid], np.rad2deg(psi12_gt[valid]), "k-", lw=1.5, label="Mocap GT")
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
    d_est3 = np.sqrt(X3[:, 1]**2 + X3[:, 2]**2 + z12_known**2)
    ax.plot(time[valid], d_gt[valid], "k-", lw=1.5, label="Mocap GT dist")
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
    """Top-down x-y plot of EKF estimates vs ground truth."""
    psi12_gt, x12_gt, y12_gt, z12_gt, d_gt = gt
    valid = ~np.isnan(x12_gt)

    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    ax.set_title(f"Relative Position (XY) — {label}", fontsize=13)
    ax.plot(x12_gt[valid], y12_gt[valid], "k-", lw=2, label="Mocap GT", alpha=0.7)
    ax.plot(X4[:, 1], X4[:, 2], "b-", lw=1, label="4-state", alpha=0.7)
    ax.plot(X3[:, 1], X3[:, 2], "r--", lw=1, label="3-state", alpha=0.7)
    # Mark start
    if valid.any():
        i0 = np.argmax(valid)
        ax.plot(x12_gt[i0], y12_gt[i0], "ko", ms=10, label="GT start")
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
    print("Loading data...")
    drone0 = load_drone0()
    mocap = load_mocap()

    tr = drone0["time_rel"]
    N_full = len(tr)

    # Downsample indices
    idx = np.arange(0, N_full, DOWNSAMPLE)
    tr_ds = tr[idx]
    N = len(tr_ds)
    print(f"Full: {N_full} samples, Downsampled: {N} samples")

    # Interpolate mocap for all 3 bodies
    mc = {}
    for bid in [0, 1, 2]:
        mc[bid] = interp_mocap_to_drone(mocap, bid, tr_ds)
        valid = np.sum(~np.isnan(mc[bid]["x"]))
        print(f"  Mocap body {bid}: {valid}/{N} valid after interp")

    # ================================================================
    # PAIR 1: Beacon (0) → Drone 2 (SD2=flapper_02) — best tracking
    # ================================================================
    print("\n=== Beacon → Drone 2 ===")
    omega_i = drone0["omega_beacon"][idx]
    vxi = np.zeros(N); vyi = np.zeros(N); vzi = np.zeros(N)  # beacon stationary
    omega_j = drone0["omega2"][idx]
    vxj = drone0["vx2"][idx]; vyj = drone0["vy2"][idx]; vzj = drone0["vz2"][idx]
    d_meas = drone0["d02"][idx]
    z12_known = drone0["height2"][idx]  # height of drone 2 ≈ z_12

    gt_02 = compute_ground_truth(mc[0], mc[2])

    X4, P4, X3, P3 = run_ekf_pair(
        tr_ds, omega_i, vxi, vyi, vzi,
        omega_j, vxj, vyj, vzj,
        d_meas, z12_known, label="Beacon→Drone2")

    plot_ekf_results(tr_ds, X4, P4, X3, P3, gt_02, d_meas, z12_known,
                     "Beacon → Drone 2 (flapper_02)", "ekf_beacon_drone2.png")
    plot_xy_trajectory(tr_ds, X4, X3, gt_02,
                       "Beacon → Drone 2", "ekf_beacon_drone2_xy.png")

    # ================================================================
    # PAIR 2: Beacon (0) → Drone 1 (SD1=flapper_01) — poor tracking
    # ================================================================
    print("\n=== Beacon → Drone 1 ===")
    omega_j = drone0["omega1"][idx]
    vxj = drone0["vx1"][idx]; vyj = drone0["vy1"][idx]; vzj = drone0["vz1"][idx]
    d_meas = drone0["d01"][idx]
    z12_known = drone0["height1"][idx]

    gt_01 = compute_ground_truth(mc[0], mc[1])

    X4_01, P4_01, X3_01, P3_01 = run_ekf_pair(
        tr_ds, omega_i, vxi, vyi, vzi,
        omega_j, vxj, vyj, vzj,
        d_meas, z12_known, label="Beacon→Drone1")

    plot_ekf_results(tr_ds, X4_01, P4_01, X3_01, P3_01, gt_01, d_meas, z12_known,
                     "Beacon → Drone 1 (flapper_01)", "ekf_beacon_drone1.png")
    plot_xy_trajectory(tr_ds, X4_01, X3_01, gt_01,
                       "Beacon → Drone 1", "ekf_beacon_drone1_xy.png")

    # ================================================================
    # PAIR 3: Drone 1 → Drone 2
    # ================================================================
    print("\n=== Drone 1 → Drone 2 ===")
    omega_i = drone0["omega1"][idx]
    vxi_12 = drone0["vx1"][idx]; vyi_12 = drone0["vy1"][idx]; vzi_12 = drone0["vz1"][idx]
    omega_j = drone0["omega2"][idx]
    vxj_12 = drone0["vx2"][idx]; vyj_12 = drone0["vy2"][idx]; vzj_12 = drone0["vz2"][idx]
    d_meas_12 = drone0["d12"][idx]
    z12_known_12 = drone0["height2"][idx] - drone0["height1"][idx]

    gt_12 = compute_ground_truth(mc[1], mc[2])

    X4_12, P4_12, X3_12, P3_12 = run_ekf_pair(
        tr_ds, omega_i, vxi_12, vyi_12, vzi_12,
        omega_j, vxj_12, vyj_12, vzj_12,
        d_meas_12, z12_known_12, label="Drone1→Drone2")

    plot_ekf_results(tr_ds, X4_12, P4_12, X3_12, P3_12, gt_12, d_meas_12, z12_known_12,
                     "Drone 1 → Drone 2 (flapper_01→02)", "ekf_drone1_drone2.png")
    plot_xy_trajectory(tr_ds, X4_12, X3_12, gt_12,
                       "Drone 1 → Drone 2", "ekf_drone1_drone2_xy.png")

    print("\nDone!")


if __name__ == "__main__":
    main()
