#!/usr/bin/env python3
"""
Run 4-state and 3-state relative-position EKFs on every dataset in
generated_data/ and compare against ground truth.

Models
------
4-state:  x = [ψ₁₂, x₁₂, y₁₂, z₁₂]
          z = d₁₂ (range measurement)

3-state:  x = [ψ₁₂, x₁₂, y₁₂]          (z₁₂ treated as known input)
          z = d₁₂

Both use continuous dynamics discretised with RK4.
"""

from __future__ import annotations

import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from ekf_utils import EKF, wrapToPi

# ──────────────────────────────────────────────────────────────────────
# Paths / constants
# ──────────────────────────────────────────────────────────────────────

GENERATED_DIR = Path("generated_data")
OUTPUT_DIR    = Path("relative_ekf_plots")
BEACON_TRUE   = np.array([1.0, 1.0, 0.0])   # beacon world position
DOWNSAMPLE    = 10                            # ~500 Hz → ~50 Hz

# ──────────────────────────────────────────────────────────────────────
# EKF tuning knobs
# ──────────────────────────────────────────────────────────────────────

Q_4 = np.diag([1e-4, 5e-4, 5e-4, 2e-4])     # process noise (4-state)
Q_3 = np.diag([1e-4, 5e-4, 5e-4])            # process noise (3-state)
R   = np.array([[0.05**2]])                   # range measurement noise

# Initial-guess offsets (deliberately wrong so we can see convergence)
INIT_POS_OFFSET = 0.5       # metres added to x₁₂, y₁₂, z₁₂
INIT_PSI_OFFSET = np.deg2rad(10.0)  # radians added to ψ₁₂

P0_4 = np.diag([0.5**2, 1.0**2, 1.0**2, 0.5**2])   # wider to match offsets
P0_3 = np.diag([0.5**2, 1.0**2, 1.0**2])


# ──────────────────────────────────────────────────────────────────────
# Dynamics & measurement models
# ──────────────────────────────────────────────────────────────────────

def f_4state(x, u):
    """Continuous-time dynamics for 4-state relative model.

    state  x = [ψ₁₂, x₁₂, y₁₂, z₁₂]
    input  u = [ψ̇_i, vx_i, vy_i, vz_i,
                ψ̇_2, vx_2, vy_2, vz_2]

    All velocities are **body-frame**.
    """
    psi12, x12, y12, z12 = x
    (psi_dot_i, vxi, vyi, vzi,
     psi_dot_2, vx2, vy2, vz2) = u[:8]

    c12 = np.cos(psi12)
    s12 = np.sin(psi12)

    psi12_dot = psi_dot_2 - psi_dot_i
    x12_dot   = (c12 * vx2 - s12 * vy2) - vxi + psi_dot_i * y12
    y12_dot   = (s12 * vx2 + c12 * vy2) - vyi - psi_dot_i * x12
    z12_dot   = vz2 - vzi

    return np.array([psi12_dot, x12_dot, y12_dot, z12_dot])


def h_4state(x, _u):
    """Measurement h = d₁₂ = ‖[x₁₂, y₁₂, z₁₂]‖."""
    _psi12, x12, y12, z12 = x
    d = np.sqrt(x12**2 + y12**2 + z12**2)
    return np.array([max(d, 1e-8)])


def f_3state(x, u):
    """Continuous-time dynamics for 3-state model (z₁₂ is known).

    state  x = [ψ₁₂, x₁₂, y₁₂]
    input  u = [ψ̇_i, vx_i, vy_i, vz_i,
                ψ̇_2, vx_2, vy_2, vz_2, z₁₂_known]
    """
    psi12, x12, y12 = x
    (psi_dot_i, vxi, vyi, _vzi,
     psi_dot_2, vx2, vy2, _vz2, _z12) = u[:9]

    c12 = np.cos(psi12)
    s12 = np.sin(psi12)

    psi12_dot = psi_dot_2 - psi_dot_i
    x12_dot   = (c12 * vx2 - s12 * vy2) - vxi + psi_dot_i * y12
    y12_dot   = (s12 * vx2 + c12 * vy2) - vyi - psi_dot_i * x12

    return np.array([psi12_dot, x12_dot, y12_dot])


def h_3state(x, u):
    """Measurement h = d₁₂ with z₁₂ taken from u."""
    _psi12, x12, y12 = x
    z12 = u[8]                          # last element of u
    d = np.sqrt(x12**2 + y12**2 + z12**2)
    return np.array([max(d, 1e-8)])


# ──────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────

def yaw_rate_from_deg(yaw_deg: np.ndarray, time: np.ndarray):
    """Return (ψ̇  [rad/s],  ψ  [rad])  from yaw in degrees."""
    yaw_rad  = np.deg2rad(yaw_deg)
    yaw_unwrap = np.unwrap(yaw_rad)
    psi_dot  = np.gradient(yaw_unwrap, time)
    return psi_dot, yaw_rad


def ground_truth_relative(drone_df: pd.DataFrame):
    """Compute body-frame relative pos of beacon (1,1,0) from drone EKF estimates."""
    dx = BEACON_TRUE[0] - drone_df["x"].values
    dy = BEACON_TRUE[1] - drone_df["y"].values
    dz = BEACON_TRUE[2] - drone_df["z"].values
    psi = np.deg2rad(drone_df["yaw"].values)

    # world → body rotation (yaw only, small roll/pitch)
    x12 =  np.cos(psi) * dx + np.sin(psi) * dy
    y12 = -np.sin(psi) * dx + np.cos(psi) * dy
    z12 = dz

    psi12 = wrapToPi(-psi)          # beacon true yaw ≈ 0
    d12   = np.sqrt(dx**2 + dy**2 + dz**2)

    return psi12, x12, y12, z12, d12


# ──────────────────────────────────────────────────────────────────────
# Core runner
# ──────────────────────────────────────────────────────────────────────

def run_one_dataset(name: str):
    """Run both relative EKFs on *name* and return a results dict."""
    # ---- load & downsample ----
    drone  = pd.read_csv(GENERATED_DIR / f"{name}_drone_estimates.csv")
    beacon = pd.read_csv(GENERATED_DIR / f"{name}_beacon_estimates.csv")
    dist   = pd.read_csv(GENERATED_DIR / f"{name}_distances.csv")

    idx = np.arange(0, len(drone), DOWNSAMPLE)
    drone  = drone.iloc[idx].reset_index(drop=True)
    beacon = beacon.iloc[idx].reset_index(drop=True)
    dist   = dist.iloc[idx].reset_index(drop=True)

    time = drone["time"].values
    N    = len(time)
    dt_arr = np.diff(time)
    dt_med = float(np.median(dt_arr))

    # ---- yaw rates ----
    psi_dot_d, yaw_d = yaw_rate_from_deg(drone["yaw"].values,  time)
    psi_dot_b, yaw_b = yaw_rate_from_deg(beacon["yaw"].values, time)

    # ---- body-frame velocities ----
    vxd = drone["vx_b"].values;  vyd = drone["vy_b"].values;  vzd = drone["vz_b"].values
    vxb = beacon["vx_b"].values; vyb = beacon["vy_b"].values; vzb = beacon["vz_b"].values

    # ---- measurements & known z12 ----
    d_meas   = dist["dist_drone_to_beacon"].values
    z12_known = beacon["z"].values - drone["z"].values

    # ---- ground truth ----
    psi12_gt, x12_gt, y12_gt, z12_gt, d_gt = ground_truth_relative(drone)

    # ---- initial state from estimates ----
    dx0 = beacon["x"].iloc[0] - drone["x"].iloc[0]
    dy0 = beacon["y"].iloc[0] - drone["y"].iloc[0]
    dz0 = beacon["z"].iloc[0] - drone["z"].iloc[0]
    c0, s0 = np.cos(yaw_d[0]), np.sin(yaw_d[0])
    x12_0 =  c0 * dx0 + s0 * dy0
    y12_0 = -s0 * dx0 + c0 * dy0
    z12_0 = dz0
    psi12_0 = float(wrapToPi(yaw_b[0] - yaw_d[0]))

    # ==================================================================
    # 4-state EKF  (offset initial guess)
    # ==================================================================
    x0_4 = np.array([psi12_0 + INIT_PSI_OFFSET,
                     x12_0  + INIT_POS_OFFSET,
                     y12_0  + INIT_POS_OFFSET,
                     z12_0  + INIT_POS_OFFSET])
    u0_4 = np.array([psi_dot_d[0], vxd[0], vyd[0], vzd[0],
                     psi_dot_b[0], vxb[0], vyb[0], vzb[0]])

    ekf4 = EKF(f_4state, h_4state, x0_4, u0_4,
               P0_4.copy(), Q_4.copy(), R.copy(),
               dynamics_type="continuous",
               discretization_timestep=dt_med)

    for k in range(1, N):
        dt_k = dt_arr[k - 1] if k - 1 < len(dt_arr) else dt_med
        u_k = np.array([psi_dot_d[k], vxd[k], vyd[k], vzd[k],
                        psi_dot_b[k], vxb[k], vyb[k], vzb[k]])
        ekf4.forward_update(np.array([d_meas[k]]), u_k,
                            discretization_timestep=dt_k)

    X4 = np.array(ekf4.history["X"])          # (N, 4)
    P4 = np.array(ekf4.history["P_diags"])    # (N, 4)

    # ==================================================================
    # 3-state EKF  (offset initial guess)
    # ==================================================================
    x0_3 = np.array([psi12_0 + INIT_PSI_OFFSET,
                     x12_0  + INIT_POS_OFFSET,
                     y12_0  + INIT_POS_OFFSET])
    u0_3 = np.array([psi_dot_d[0], vxd[0], vyd[0], vzd[0],
                     psi_dot_b[0], vxb[0], vyb[0], vzb[0],
                     z12_known[0]])

    ekf3 = EKF(f_3state, h_3state, x0_3, u0_3,
               P0_3.copy(), Q_3.copy(), R.copy(),
               dynamics_type="continuous",
               discretization_timestep=dt_med)

    for k in range(1, N):
        dt_k = dt_arr[k - 1] if k - 1 < len(dt_arr) else dt_med
        u_k = np.array([psi_dot_d[k], vxd[k], vyd[k], vzd[k],
                        psi_dot_b[k], vxb[k], vyb[k], vzb[k],
                        z12_known[k]])
        ekf3.forward_update(np.array([d_meas[k]]), u_k,
                            discretization_timestep=dt_k)

    X3 = np.array(ekf3.history["X"])          # (N, 3)
    P3 = np.array(ekf3.history["P_diags"])    # (N, 3)

    return dict(name=name, time=time,
                X4=X4, X3=X3, P4=P4, P3=P3,
                gt=dict(psi12=psi12_gt, x12=x12_gt, y12=y12_gt,
                        z12=z12_gt, d=d_gt),
                d_meas=d_meas, z12_known=z12_known)


# ──────────────────────────────────────────────────────────────────────
# Plotting
# ──────────────────────────────────────────────────────────────────────

def plot_one(res: dict, out_dir: Path):
    t   = res["time"]
    X4  = res["X4"];   P4  = res["P4"]
    X3  = res["X3"];   P3  = res["P3"]
    gt  = res["gt"]
    dm  = res["d_meas"]
    zk  = res["z12_known"]
    name = res["name"]

    fig, axes = plt.subplots(5, 1, figsize=(14, 16), sharex=True)
    fig.suptitle(f"Relative EKF  —  {name}", fontsize=14, fontweight="bold")

    # ---- x₁₂ ----
    ax = axes[0]
    ax.plot(t, gt["x12"], "k-", lw=1.5, label="Ground truth")
    ax.plot(t, X4[:, 1], "b-", alpha=.8, label="4-state")
    ax.fill_between(t,
                    X4[:, 1] - 2*np.sqrt(P4[:, 1]),
                    X4[:, 1] + 2*np.sqrt(P4[:, 1]),
                    color="b", alpha=.12)
    ax.plot(t, X3[:, 1], "r--", alpha=.8, label="3-state")
    ax.fill_between(t,
                    X3[:, 1] - 2*np.sqrt(P3[:, 1]),
                    X3[:, 1] + 2*np.sqrt(P3[:, 1]),
                    color="r", alpha=.12)
    ax.set_ylabel("x₁₂  [m]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=.3)

    # ---- y₁₂ ----
    ax = axes[1]
    ax.plot(t, gt["y12"], "k-", lw=1.5, label="Ground truth")
    ax.plot(t, X4[:, 2], "b-", alpha=.8, label="4-state")
    ax.fill_between(t,
                    X4[:, 2] - 2*np.sqrt(P4[:, 2]),
                    X4[:, 2] + 2*np.sqrt(P4[:, 2]),
                    color="b", alpha=.12)
    ax.plot(t, X3[:, 2], "r--", alpha=.8, label="3-state")
    ax.fill_between(t,
                    X3[:, 2] - 2*np.sqrt(P3[:, 2]),
                    X3[:, 2] + 2*np.sqrt(P3[:, 2]),
                    color="r", alpha=.12)
    ax.set_ylabel("y₁₂  [m]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=.3)

    # ---- z₁₂ ----
    ax = axes[2]
    ax.plot(t, gt["z12"], "k-", lw=1.5, label="Ground truth")
    ax.plot(t, X4[:, 3], "b-", alpha=.8, label="4-state est.")
    ax.fill_between(t,
                    X4[:, 3] - 2*np.sqrt(P4[:, 3]),
                    X4[:, 3] + 2*np.sqrt(P4[:, 3]),
                    color="b", alpha=.12)
    ax.plot(t, zk, "r--", alpha=.8, label="3-state (known)")
    ax.set_ylabel("z₁₂  [m]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=.3)

    # ---- ψ₁₂ ----
    ax = axes[3]
    ax.plot(t, np.rad2deg(gt["psi12"]), "k-", lw=1.5, label="Ground truth")
    ax.plot(t, np.rad2deg(X4[:, 0]), "b-", alpha=.8, label="4-state")
    ax.fill_between(t,
                    np.rad2deg(X4[:, 0] - 2*np.sqrt(P4[:, 0])),
                    np.rad2deg(X4[:, 0] + 2*np.sqrt(P4[:, 0])),
                    color="b", alpha=.12)
    ax.plot(t, np.rad2deg(X3[:, 0]), "r--", alpha=.8, label="3-state")
    ax.fill_between(t,
                    np.rad2deg(X3[:, 0] - 2*np.sqrt(P3[:, 0])),
                    np.rad2deg(X3[:, 0] + 2*np.sqrt(P3[:, 0])),
                    color="r", alpha=.12)
    ax.set_ylabel("ψ₁₂  [deg]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=.3)

    # ---- distance ----
    ax = axes[4]
    d_est4 = np.sqrt(X4[:, 1]**2 + X4[:, 2]**2 + X4[:, 3]**2)
    d_est3 = np.sqrt(X3[:, 1]**2 + X3[:, 2]**2 + zk**2)
    ax.plot(t, gt["d"], "k-", lw=1.5, label="True distance")
    ax.plot(t, dm, "g.", ms=1, alpha=.25, label="Measured")
    ax.plot(t, d_est4, "b-", alpha=.8, label="4-state")
    ax.plot(t, d_est3, "r--", alpha=.8, label="3-state")
    ax.set_ylabel("Distance  [m]")
    ax.set_xlabel("Time  [s]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=.3)

    plt.tight_layout()
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / f"{name}_relative_ekf.png", dpi=150)
    plt.close(fig)


# ──────────────────────────────────────────────────────────────────────
# Metrics
# ──────────────────────────────────────────────────────────────────────

def rmse(est, truth):
    return float(np.sqrt(np.mean((est - truth)**2)))


def compute_rmse(res):
    gt = res["gt"];  X4 = res["X4"];  X3 = res["X3"]
    r4 = dict(x12=rmse(X4[:, 1], gt["x12"]),
              y12=rmse(X4[:, 2], gt["y12"]),
              z12=rmse(X4[:, 3], gt["z12"]),
              psi=rmse(wrapToPi(X4[:, 0] - gt["psi12"]), 0.0))
    r3 = dict(x12=rmse(X3[:, 1], gt["x12"]),
              y12=rmse(X3[:, 2], gt["y12"]),
              psi=rmse(wrapToPi(X3[:, 0] - gt["psi12"]), 0.0))
    return r4, r3


# ──────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────

def main():
    datasets = sorted(
        p.stem.replace("_drone_estimates", "")
        for p in GENERATED_DIR.glob("*_drone_estimates.csv")
    )
    print(f"Found {len(datasets)} datasets\n")

    all_results = []
    for name in datasets:
        print(f"▸ {name} …", end="", flush=True)
        res = run_one_dataset(name)
        plot_one(res, OUTPUT_DIR)
        r4, r3 = compute_rmse(res)
        all_results.append((name, r4, r3))
        print(f"  done  (4-st RMSE  x={r4['x12']:.3f} y={r4['y12']:.3f} "
              f"z={r4['z12']:.3f}  ψ={np.rad2deg(r4['psi']):.1f}°)")

    # ---- summary table ----
    print("\n" + "=" * 100)
    hdr = (f"{'Dataset':<26}"
           f"{'4-x₁₂':>7} {'4-y₁₂':>7} {'4-z₁₂':>7} {'4-ψ°':>7}"
           f"  │  {'3-x₁₂':>7} {'3-y₁₂':>7} {'3-ψ°':>7}")
    print(hdr)
    print("-" * 100)
    for name, r4, r3 in all_results:
        print(f"{name[:25]:<26}"
              f"{r4['x12']:>7.4f} {r4['y12']:>7.4f} {r4['z12']:>7.4f} "
              f"{np.rad2deg(r4['psi']):>7.2f}"
              f"  │  {r3['x12']:>7.4f} {r3['y12']:>7.4f} "
              f"{np.rad2deg(r3['psi']):>7.2f}")

    # ---- averages ----
    avg4 = {k: np.mean([r[k] for _, r, _ in all_results])
            for k in ("x12", "y12", "z12", "psi")}
    avg3 = {k: np.mean([r[k] for _, _, r in all_results])
            for k in ("x12", "y12", "psi")}
    print("-" * 100)
    print(f"{'MEAN':<26}"
          f"{avg4['x12']:>7.4f} {avg4['y12']:>7.4f} {avg4['z12']:>7.4f} "
          f"{np.rad2deg(avg4['psi']):>7.2f}"
          f"  │  {avg3['x12']:>7.4f} {avg3['y12']:>7.4f} "
          f"{np.rad2deg(avg3['psi']):>7.2f}")
    print("=" * 100)
    print(f"\nPlots saved to  {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
