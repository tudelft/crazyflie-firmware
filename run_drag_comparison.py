#!/usr/bin/env python3
"""
Drag-degradation study for the relative-position ExcF.

For each of 3 drag conditions (tuned, 50 %-off, no-drag) this script:

  1. Re-runs the single-drone EKF  → degraded velocity / position estimates
  2. Regenerates beacon data + range measurements (beacon stays the same)
  3. Runs the 4-state and 3-state relative EKFs using those estimates
  4. Compares convergence and RMSE

Output
------
  drag_comparison_plots/{name}_{condition}.png   — per-dataset time series
  drag_comparison_plots/summary_bar.png          — bar chart across all datasets
  drag_comparison_plots/overlay_{name}.png       — 3 conditions overlaid per state
"""

from __future__ import annotations

import dataclasses
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt

from ekf_replay import EKFParams, run_ekf

# Reuse helpers from existing scripts
from run_relative_ekf import (
    f_4state, h_4state, f_3state, h_3state,
    yaw_rate_from_deg, ground_truth_relative,
    BEACON_TRUE, DOWNSAMPLE,
    Q_4, Q_3, R, P0_4, P0_3,
    INIT_POS_OFFSET, INIT_PSI_OFFSET,
    rmse, compute_rmse,
)
from ekf_utils import EKF, wrapToPi

# ──────────────────────────────────────────────────────────────────────
# Tuned parameters  (same as generate_beacon_data.py / ekf_replay.py)
# ──────────────────────────────────────────────────────────────────────

TUNED_DRAG = dict(drag_x=4.39468, drag_y=2.88896, drag_z=0.0611769)

TUNED_PARAMS_BASE = dict(
    proc_noise_acc_xy=1.05006,
    proc_noise_acc_z=0.604273,
    meas_noise_gyro_rp=0.0521776,
    meas_noise_gyro_yaw=0.116742,
    flow_std_fixed_x=1.07615,
    flow_std_fixed_y=5.41112,
    flow_resolution=0.22987,
    cop=np.array([0.0, 0.0, 0.03]),
    flowdeck_pos=np.array([0.0, 0.0, -0.12]),
)

# ──────────────────────────────────────────────────────────────────────
# Drag conditions to test
# ──────────────────────────────────────────────────────────────────────

DRAG_CONDITIONS = {
    "tuned": {**TUNED_DRAG},                          # 100 % of tuned
    "drag_50pct_off": {
        k: v * 0.5 for k, v in TUNED_DRAG.items()    # 50 % of tuned
    },
    "no_drag": {
        k: 0.0 for k in TUNED_DRAG                   # 0 drag
    },
}

CONDITION_LABELS = {
    "tuned":          "Tuned drag",
    "drag_50pct_off": "Drag 50 % off",
    "no_drag":        "No drag",
}
CONDITION_COLORS = {
    "tuned":          "C0",
    "drag_50pct_off": "C1",
    "no_drag":        "C3",
}

ALL_CSVS = sorted(str(p) for p in Path("DataFlapperEKF").glob("*.csv"))
OUTPUT_DIR = Path("drag_comparison_plots")

# ──────────────────────────────────────────────────────────────────────
# Beacon generation  (copied minimal version — beacon is deterministic)
# ──────────────────────────────────────────────────────────────────────

BEACON_POS = np.array([1.0, 1.0, 0.0])
BEACON_POS_NOISE_STD  = 0.005
BEACON_VEL_NOISE_STD  = 0.005
BEACON_ATT_NOISE_STD  = 0.3
BEACON_NOISE_LPF_HZ   = 2.0
RANGE_NOISE_STD        = 0.05


def _smooth_noise(n, std, dt, rng, cutoff_hz=BEACON_NOISE_LPF_HZ):
    raw = rng.normal(0.0, 1.0, n)
    fs = 1.0 / dt
    nyq = fs / 2.0
    if cutoff_hz >= nyq:
        cutoff_hz = nyq * 0.9
    b, a = butter(2, cutoff_hz / nyq, btype="low")
    filt = filtfilt(b, a, raw)
    s = filt.std()
    if s > 0:
        filt *= std / s
    return filt


def generate_beacon_estimates(time, rng):
    n = len(time)
    dt = float(np.median(np.diff(time))) if n > 1 else 0.002
    x = BEACON_POS[0] + _smooth_noise(n, BEACON_POS_NOISE_STD, dt, rng)
    y = BEACON_POS[1] + _smooth_noise(n, BEACON_POS_NOISE_STD, dt, rng)
    z_noise = _smooth_noise(n, BEACON_POS_NOISE_STD * 0.2, dt, rng)
    z = BEACON_POS[2] + np.abs(z_noise)
    vx_b = _smooth_noise(n, BEACON_VEL_NOISE_STD, dt, rng)
    vy_b = _smooth_noise(n, BEACON_VEL_NOISE_STD, dt, rng)
    vz_b = _smooth_noise(n, BEACON_VEL_NOISE_STD, dt, rng)
    roll  = _smooth_noise(n, BEACON_ATT_NOISE_STD, dt, rng)
    pitch = _smooth_noise(n, BEACON_ATT_NOISE_STD, dt, rng)
    yaw   = _smooth_noise(n, BEACON_ATT_NOISE_STD, dt, rng)
    return pd.DataFrame({
        "time": time, "x": x, "y": y, "z": z,
        "vx": vx_b, "vy": vy_b, "vz": vz_b,
        "vx_b": vx_b, "vy_b": vy_b, "vz_b": vz_b,
        "roll": roll, "pitch": pitch, "yaw": yaw,
    })


def generate_distances(time, drone_pos, rng):
    diff = drone_pos - np.tile(BEACON_POS, (len(time), 1))
    true_dist = np.sqrt(np.sum(diff ** 2, axis=1))
    d2b = np.maximum(true_dist + rng.normal(0, RANGE_NOISE_STD, len(time)), 0.0)
    b2d = np.maximum(true_dist + rng.normal(0, RANGE_NOISE_STD, len(time)), 0.0)
    return pd.DataFrame({
        "time": time, "true_distance": true_dist,
        "dist_drone_to_beacon": d2b, "dist_beacon_to_drone": b2d,
    })


# ──────────────────────────────────────────────────────────────────────
# Run relative EKF on a set of DataFrames  (mirror of run_relative_ekf)
# ──────────────────────────────────────────────────────────────────────

def run_relative_from_dfs(drone, beacon, dist):
    """Run 4-state & 3-state relative EKFs and return results dict."""
    idx = np.arange(0, len(drone), DOWNSAMPLE)
    drone  = drone.iloc[idx].reset_index(drop=True)
    beacon = beacon.iloc[idx].reset_index(drop=True)
    dist   = dist.iloc[idx].reset_index(drop=True)

    time = drone["time"].values
    N    = len(time)
    dt_arr = np.diff(time)
    dt_med = float(np.median(dt_arr))

    psi_dot_d, yaw_d = yaw_rate_from_deg(drone["yaw"].values,  time)
    psi_dot_b, yaw_b = yaw_rate_from_deg(beacon["yaw"].values, time)

    vxd = drone["vx_b"].values;  vyd = drone["vy_b"].values;  vzd = drone["vz_b"].values
    vxb = beacon["vx_b"].values; vyb = beacon["vy_b"].values; vzb = beacon["vz_b"].values

    d_meas    = dist["dist_drone_to_beacon"].values
    z12_known = beacon["z"].values - drone["z"].values

    psi12_gt, x12_gt, y12_gt, z12_gt, d_gt = ground_truth_relative(drone)

    # Initial state + offset
    dx0 = beacon["x"].iloc[0] - drone["x"].iloc[0]
    dy0 = beacon["y"].iloc[0] - drone["y"].iloc[0]
    dz0 = beacon["z"].iloc[0] - drone["z"].iloc[0]
    c0, s0 = np.cos(yaw_d[0]), np.sin(yaw_d[0])
    x12_0 =  c0 * dx0 + s0 * dy0
    y12_0 = -s0 * dx0 + c0 * dy0
    z12_0 = dz0
    psi12_0 = float(wrapToPi(yaw_b[0] - yaw_d[0]))

    # ---- 4-state ----
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
        dt_k = dt_arr[k-1] if k-1 < len(dt_arr) else dt_med
        u_k = np.array([psi_dot_d[k], vxd[k], vyd[k], vzd[k],
                        psi_dot_b[k], vxb[k], vyb[k], vzb[k]])
        ekf4.forward_update(np.array([d_meas[k]]), u_k,
                            discretization_timestep=dt_k)
    X4 = np.array(ekf4.history["X"])
    P4 = np.array(ekf4.history["P_diags"])

    # ---- 3-state ----
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
        dt_k = dt_arr[k-1] if k-1 < len(dt_arr) else dt_med
        u_k = np.array([psi_dot_d[k], vxd[k], vyd[k], vzd[k],
                        psi_dot_b[k], vxb[k], vyb[k], vzb[k],
                        z12_known[k]])
        ekf3.forward_update(np.array([d_meas[k]]), u_k,
                            discretization_timestep=dt_k)
    X3 = np.array(ekf3.history["X"])
    P3 = np.array(ekf3.history["P_diags"])

    return dict(time=time, X4=X4, X3=X3, P4=P4, P3=P3,
                gt=dict(psi12=psi12_gt, x12=x12_gt, y12=y12_gt,
                        z12=z12_gt, d=d_gt),
                d_meas=d_meas, z12_known=z12_known)


# ──────────────────────────────────────────────────────────────────────
# Full pipeline for one CSV × one drag condition
# ──────────────────────────────────────────────────────────────────────

def pipeline_one(csv_path: str, drag_overrides: dict):
    """EKF replay → beacon gen → relative EKF.  Returns relative results dict."""
    params = EKFParams(**{**TUNED_PARAMS_BASE, **drag_overrides})
    ekf_results = run_ekf(csv_path, params=params)
    time = ekf_results["time"].to_numpy()

    # Drone estimates
    est_cols = ["time", "x", "y", "z", "vx", "vy", "vz",
                "vx_b", "vy_b", "vz_b", "roll", "pitch", "yaw"]
    drone_est = ekf_results[[c for c in est_cols if c in ekf_results.columns]].copy()

    # Beacon (deterministic seed)
    rng = np.random.default_rng(seed=42)
    beacon_est = generate_beacon_estimates(time, rng)

    # Distances from mocap drone position (if available)
    has_mocap = "ls_x" in ekf_results.columns
    if has_mocap:
        drone_pos = ekf_results[["ls_x", "ls_y", "ls_z"]].to_numpy()
    else:
        drone_pos = ekf_results[["x", "y", "z"]].to_numpy()

    rng_d = np.random.default_rng(seed=42)
    dist_df = generate_distances(time, drone_pos, rng_d)

    # Relative EKF
    res = run_relative_from_dfs(drone_est, beacon_est, dist_df)
    return res


# ──────────────────────────────────────────────────────────────────────
# Overlay plot — one dataset, all 3 conditions side-by-side
# ──────────────────────────────────────────────────────────────────────

def plot_overlay(name: str, cond_results: dict, out_dir: Path, model="4"):
    """Plot x₁₂, y₁₂, z₁₂, distance for all 3 conditions overlaid."""
    fig, axes = plt.subplots(4, 1, figsize=(14, 14), sharex=True)
    m = model  # "4" or "3"
    fig.suptitle(f"Relative EKF ({m}-state)  —  {name}", fontsize=14, fontweight="bold")

    # Pick any condition for ground truth (same across all)
    gt = list(cond_results.values())[0]["gt"]
    t0 = list(cond_results.values())[0]["time"]

    # ── x₁₂ ──
    ax = axes[0]
    ax.plot(t0, gt["x12"], "k-", lw=1.5, label="Ground truth", zorder=10)
    for cond, res in cond_results.items():
        t = res["time"]
        ax.plot(t, res[f"X{m}"][:, 1], color=CONDITION_COLORS[cond],
                alpha=0.85, label=CONDITION_LABELS[cond])
    ax.set_ylabel("x₁₂  [m]")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)

    # ── y₁₂ ──
    ax = axes[1]
    ax.plot(t0, gt["y12"], "k-", lw=1.5, label="Ground truth", zorder=10)
    for cond, res in cond_results.items():
        t = res["time"]
        ax.plot(t, res[f"X{m}"][:, 2], color=CONDITION_COLORS[cond],
                alpha=0.85, label=CONDITION_LABELS[cond])
    ax.set_ylabel("y₁₂  [m]")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)

    # ── z₁₂  (estimated for 4-state, known input for 3-state) ──
    ax = axes[2]
    ax.plot(t0, gt["z12"], "k-", lw=1.5, label="Ground truth", zorder=10)
    for cond, res in cond_results.items():
        t = res["time"]
        col = CONDITION_COLORS[cond]
        if m == "4":
            z_vals = res["X4"][:, 3]
        else:
            z_vals = res["z12_known"]
        ax.plot(t, z_vals, color=col, alpha=0.85,
                label=CONDITION_LABELS[cond])
    z_label = "z₁₂  [m]" if m == "4" else "z₁₂  [m]  (known)"
    ax.set_ylabel(z_label)
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)

    # ── Distance ──
    ax = axes[3]
    ax.plot(t0, gt["d"], "k-", lw=1.5, label="True distance", zorder=10)
    for cond, res in cond_results.items():
        t = res["time"]
        Xm = res[f"X{m}"]
        col = CONDITION_COLORS[cond]
        if m == "4":
            d_est = np.sqrt(Xm[:, 1]**2 + Xm[:, 2]**2 + Xm[:, 3]**2)
        else:
            d_est = np.sqrt(Xm[:, 1]**2 + Xm[:, 2]**2 + res["z12_known"]**2)
        ax.plot(t, d_est, color=col, alpha=0.85,
                label=CONDITION_LABELS[cond])
    ax.set_ylabel("Distance  [m]")
    ax.set_xlabel("Time  [s]")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / f"overlay_{name}_{m}state.png", dpi=150)
    plt.close(fig)


# ──────────────────────────────────────────────────────────────────────
# Bar-chart summary across all datasets
# ──────────────────────────────────────────────────────────────────────

def plot_summary_bars(all_rmse: dict, out_dir: Path):
    """
    all_rmse[cond][name] = (r4_dict, r3_dict)
    """
    datasets = sorted(next(iter(all_rmse.values())).keys())
    conds = list(DRAG_CONDITIONS.keys())
    n_ds = len(datasets)
    x = np.arange(n_ds)
    w = 0.25

    for model, states in [("4-state", ["x12", "y12", "z12"]),
                           ("3-state", ["x12", "y12"])]:
        fig, axes = plt.subplots(len(states), 1,
                                 figsize=(max(14, n_ds * 1.1), 4 * len(states)),
                                 sharex=True)
        if len(states) == 1:
            axes = [axes]
        fig.suptitle(f"Relative Position RMSE  —  {model}  (offset init: "
                     f"+{INIT_POS_OFFSET} m, +{np.rad2deg(INIT_PSI_OFFSET):.0f}°)",
                     fontsize=13, fontweight="bold")

        mi = 0 if model == "4-state" else 1  # index into (r4, r3)
        for si, sname in enumerate(states):
            ax = axes[si]
            for ci, cond in enumerate(conds):
                vals = []
                for ds in datasets:
                    r = all_rmse[cond][ds][mi]
                    vals.append(r[sname])
                ax.bar(x + ci * w, vals, w, label=CONDITION_LABELS[cond],
                       color=CONDITION_COLORS[cond], alpha=0.85)
            ax.set_ylabel(f"{sname} RMSE  [m]")
            ax.set_xticks(x + w)
            ax.set_xticklabels([d.replace("_mtf", "").replace("flight_", "")
                                for d in datasets],
                               rotation=40, ha="right", fontsize=8)
            ax.legend(fontsize=8)
            ax.grid(True, axis="y", alpha=0.3)

        plt.tight_layout()
        out_dir.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_dir / f"summary_bar_{model.replace('-','')}.png", dpi=150)
        plt.close(fig)


# ──────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────

def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    datasets = [Path(p).stem for p in ALL_CSVS]
    conds = list(DRAG_CONDITIONS.keys())

    print(f"Datasets : {len(datasets)}")
    print(f"Conditions: {', '.join(CONDITION_LABELS[c] for c in conds)}")
    print(f"Initial offset: +{INIT_POS_OFFSET} m position, "
          f"+{np.rad2deg(INIT_PSI_OFFSET):.0f}° heading\n")

    all_rmse = {c: {} for c in conds}       # cond → name → (r4, r3)
    all_results = {c: {} for c in conds}     # cond → name → result dict

    for csv_path in ALL_CSVS:
        name = Path(csv_path).stem
        print(f"{'─' * 60}")
        print(f"  {name}")
        cond_results = {}

        for cond in conds:
            drag = DRAG_CONDITIONS[cond]
            label = CONDITION_LABELS[cond]
            print(f"    ▸ {label:20s}", end="", flush=True)

            res = pipeline_one(csv_path, drag)
            r4, r3 = compute_rmse(res)
            all_rmse[cond][name] = (r4, r3)
            all_results[cond][name] = res
            cond_results[cond] = res

            print(f"  4-st RMSE x={r4['x12']:.3f} y={r4['y12']:.3f} z={r4['z12']:.3f}"
                  f"  │  3-st x={r3['x12']:.3f} y={r3['y12']:.3f}")

        # Overlay plots (per-dataset, both models)
        for m in ("4", "3"):
            plot_overlay(name, cond_results, OUTPUT_DIR, model=m)

    # ── Summary table ────────────────────────────────────────────────
    print(f"\n{'═' * 110}")
    print(f"{'':26s}{'Tuned':^25s}│{'Drag 50 % off':^25s}│{'No drag':^25s}")
    print(f"{'Dataset':26s}{'4-x₁₂':>7s}{'4-y₁₂':>7s}{'4-z₁₂':>7s}  "
          f"│ {'4-x₁₂':>7s}{'4-y₁₂':>7s}{'4-z₁₂':>7s}  "
          f"│ {'4-x₁₂':>7s}{'4-y₁₂':>7s}{'4-z₁₂':>7s}")
    print(f"{'─' * 110}")
    for name in datasets:
        parts = []
        for cond in conds:
            r4, _ = all_rmse[cond][name]
            parts.append(f"{r4['x12']:>7.3f}{r4['y12']:>7.3f}{r4['z12']:>7.3f}")
        print(f"{name[:25]:<26s}" + "  │ ".join(parts))

    # Means
    print(f"{'─' * 110}")
    parts = []
    for cond in conds:
        mx = np.mean([all_rmse[cond][n][0]["x12"] for n in datasets])
        my = np.mean([all_rmse[cond][n][0]["y12"] for n in datasets])
        mz = np.mean([all_rmse[cond][n][0]["z12"] for n in datasets])
        parts.append(f"{mx:>7.3f}{my:>7.3f}{mz:>7.3f}")
    print(f"{'MEAN':26s}" + "  │ ".join(parts))
    print(f"{'═' * 110}")

    # Same for 3-state
    print(f"\n{'':26s}{'Tuned':^19s}│{'Drag 50 % off':^19s}│{'No drag':^19s}")
    print(f"{'Dataset':26s}{'3-x₁₂':>7s}{'3-y₁₂':>7s}  "
          f"│ {'3-x₁₂':>7s}{'3-y₁₂':>7s}  "
          f"│ {'3-x₁₂':>7s}{'3-y₁₂':>7s}")
    print(f"{'─' * 85}")
    for name in datasets:
        parts = []
        for cond in conds:
            _, r3 = all_rmse[cond][name]
            parts.append(f"{r3['x12']:>7.3f}{r3['y12']:>7.3f}")
        print(f"{name[:25]:<26s}" + "  │ ".join(parts))
    print(f"{'─' * 85}")
    parts = []
    for cond in conds:
        mx = np.mean([all_rmse[cond][n][1]["x12"] for n in datasets])
        my = np.mean([all_rmse[cond][n][1]["y12"] for n in datasets])
        parts.append(f"{mx:>7.3f}{my:>7.3f}")
    print(f"{'MEAN':26s}" + "  │ ".join(parts))
    print(f"{'═' * 85}")

    # ── Summary bar charts ───────────────────────────────────────────
    plot_summary_bars(all_rmse, OUTPUT_DIR)
    print(f"\nPlots saved to  {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
