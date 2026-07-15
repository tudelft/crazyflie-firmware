#!/usr/bin/env python3
"""
Generate companion beacon data and inter-agent distance measurements
for all flight CSVs in DataFlapperEKF/.

For each flight CSV this script produces three files and one plot:

  {name}_drone_estimates.csv   — EKF state estimates for the flying drone
  {name}_beacon_estimates.csv  — synthetic "EKF" estimates for a ground beacon
                                  sitting at (1, 1, 0) with small noise
  {name}_distances.csv         — simulated range measurements between
                                  drone (mocap position) ↔ beacon
  {name}_trajectory.png        — XYZ + distance plot

The beacon simulates a second Crazyflie on the ground that knows its own
position/velocity/attitude via its onboard EKF.  The range measurements
simulate a UWB-like inter-agent distance sensor.

Usage:
    python generate_beacon_data.py                        # all CSVs
    python generate_beacon_data.py --csv flight_sine_mtf.csv
    python generate_beacon_data.py --output-dir my_data
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

from ekf_replay import EKFParams, run_ekf

# ---------------------------------------------------------------------------
# Tuned EKF parameters  (must match ekf_replay.py main())
# ---------------------------------------------------------------------------

TUNED_PARAMS = dict(
    proc_noise_acc_xy=1.05006,
    proc_noise_acc_z=0.604273,
    meas_noise_gyro_rp=0.0521776,
    meas_noise_gyro_yaw=0.116742,
    drag_x=4.39468,
    drag_y=2.88896,
    drag_z=0.0611769,
    flow_std_fixed_x=1.07615,
    flow_std_fixed_y=5.41112,
    flow_resolution=0.22987,
    cop=np.array([0.0, 0.0, 0.03]),
    flowdeck_pos=np.array([0.0, 0.0, -0.12]),
)

# ---------------------------------------------------------------------------
# Beacon configuration
# ---------------------------------------------------------------------------

BEACON_POS = np.array([1.0, 1.0, 0.0])   # ground-truth position (m)

# Noise on the beacon's own state estimates (simulating its onboard EKF)
BEACON_POS_NOISE_STD  = 0.005    # m   — position jitter
BEACON_VEL_NOISE_STD  = 0.005    # m/s — velocity jitter
BEACON_ATT_NOISE_STD  = 0.3      # deg — attitude jitter

# Low-pass cutoff for beacon noise to look like EKF output (Hz)
BEACON_NOISE_LPF_HZ   = 2.0

# Inter-agent range measurement noise (UWB-like)
RANGE_NOISE_STD = 0.05            # m  (5 cm 1-σ)

# ---------------------------------------------------------------------------
# All available CSVs
# ---------------------------------------------------------------------------

ALL_CSVS = sorted(str(p) for p in Path("DataFlapperEKF").glob("*.csv"))


# ---------------------------------------------------------------------------
# Beacon data generation
# ---------------------------------------------------------------------------

def _smooth_noise(n: int, std: float, dt: float, rng: np.random.Generator,
                  cutoff_hz: float = BEACON_NOISE_LPF_HZ) -> np.ndarray:
    """
    Generate low-pass-filtered Gaussian noise that looks like slowly-varying
    EKF estimation error.  Returns array of length *n* with approximate std
    of *std* after filtering.
    """
    raw = rng.normal(0.0, 1.0, n)
    fs = 1.0 / dt
    nyq = fs / 2.0
    if cutoff_hz >= nyq:
        cutoff_hz = nyq * 0.9
    b, a = butter(2, cutoff_hz / nyq, btype="low")
    filt = filtfilt(b, a, raw)
    # Re-scale to desired std
    s = filt.std()
    if s > 0:
        filt *= std / s
    return filt


def generate_beacon_estimates(time: np.ndarray,
                              rng: np.random.Generator) -> pd.DataFrame:
    """
    Synthetic EKF estimates for a beacon sitting on the ground at BEACON_POS.
    Position, velocity, and attitude are near their true values with small,
    smoothly varying noise.
    """
    n = len(time)
    dt = float(np.median(np.diff(time))) if n > 1 else 0.002

    # Position: (1, 1, 0) + smooth noise
    x = BEACON_POS[0] + _smooth_noise(n, BEACON_POS_NOISE_STD, dt, rng)
    y = BEACON_POS[1] + _smooth_noise(n, BEACON_POS_NOISE_STD, dt, rng)
    z_noise = _smooth_noise(n, BEACON_POS_NOISE_STD * 0.2, dt, rng)
    z = BEACON_POS[2] + np.abs(z_noise)       # stays ≥ 0

    # Velocity: ~0 + smooth noise
    vx_b = _smooth_noise(n, BEACON_VEL_NOISE_STD, dt, rng)
    vy_b = _smooth_noise(n, BEACON_VEL_NOISE_STD, dt, rng)
    vz_b = _smooth_noise(n, BEACON_VEL_NOISE_STD, dt, rng)

    # World-frame velocity (≈ body-frame since attitude ≈ identity)
    vx = vx_b.copy()
    vy = vy_b.copy()
    vz = vz_b.copy()

    # Attitude: ~0 + smooth noise
    roll  = _smooth_noise(n, BEACON_ATT_NOISE_STD, dt, rng)
    pitch = _smooth_noise(n, BEACON_ATT_NOISE_STD, dt, rng)
    yaw   = _smooth_noise(n, BEACON_ATT_NOISE_STD, dt, rng)

    return pd.DataFrame({
        "time":  time,
        "x":     x,
        "y":     y,
        "z":     z,
        "vx":    vx,
        "vy":    vy,
        "vz":    vz,
        "vx_b":  vx_b,
        "vy_b":  vy_b,
        "vz_b":  vz_b,
        "roll":  roll,
        "pitch": pitch,
        "yaw":   yaw,
    })


# ---------------------------------------------------------------------------
# Distance measurement generation
# ---------------------------------------------------------------------------

def generate_distances(time: np.ndarray,
                       drone_pos: np.ndarray,
                       beacon_pos_noisy: np.ndarray,
                       rng: np.random.Generator) -> pd.DataFrame:
    """
    Simulated inter-agent range measurements.

    drone_pos        : (n, 3) — drone's mocap (ground-truth) position
    beacon_pos_noisy : (n, 3) — beacon's noisy estimated position
    """
    # True distance (mocap drone → true beacon position)
    diff_true = drone_pos - np.tile(BEACON_POS, (len(time), 1))
    true_dist = np.sqrt(np.sum(diff_true ** 2, axis=1))

    # Two independent noisy measurements
    noise_d2b = rng.normal(0, RANGE_NOISE_STD, len(time))
    noise_b2d = rng.normal(0, RANGE_NOISE_STD, len(time))
    dist_drone_to_beacon = np.maximum(true_dist + noise_d2b, 0.0)
    dist_beacon_to_drone = np.maximum(true_dist + noise_b2d, 0.0)

    return pd.DataFrame({
        "time":                time,
        "true_distance":       true_dist,
        "dist_drone_to_beacon": dist_drone_to_beacon,
        "dist_beacon_to_drone": dist_beacon_to_drone,
    })


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_trajectories(drone_est: pd.DataFrame,
                      beacon_est: pd.DataFrame,
                      dist_df: pd.DataFrame,
                      ekf_results: pd.DataFrame,
                      name: str,
                      save_path: str | None = None):
    """XYZ trajectory + distance plot for one flight."""
    has_mocap = "ls_x" in ekf_results.columns
    t = drone_est["time"]

    fig, axes = plt.subplots(4, 1, figsize=(16, 12), sharex=True)
    fig.suptitle(f"Drone + Beacon Trajectories: {name}", fontsize=14)

    # --- X, Y, Z ---
    for i, (col, ylabel) in enumerate([
        ("x", "X position (m)"),
        ("y", "Y position (m)"),
        ("z", "Z position (m)"),
    ]):
        ax = axes[i]
        ax.plot(t, drone_est[col], "C0", linewidth=1.2, label="Drone EKF")
        ax.plot(t, beacon_est[col], "C3", linewidth=1.2, label="Beacon EKF")
        if has_mocap:
            ax.plot(t, ekf_results[f"ls_{col}"], ":", color="C2",
                    alpha=0.7, linewidth=1.0, label="Drone mocap")
        ax.axhline(BEACON_POS[i], color="C3", linestyle="--", alpha=0.4,
                   label=f"Beacon truth ({BEACON_POS[i]:.0f} m)")
        ax.set_ylabel(ylabel)
        ax.legend(fontsize=7, loc="upper right", ncol=2)
        ax.grid(True, alpha=0.3)

    # --- Distance ---
    ax = axes[3]
    ax.plot(t, dist_df["true_distance"], "k-", linewidth=0.8,
            alpha=0.5, label="True distance")
    ax.plot(t, dist_df["dist_drone_to_beacon"], ".", color="C0",
            markersize=1.5, alpha=0.6, label="Drone → Beacon (noisy)")
    ax.plot(t, dist_df["dist_beacon_to_drone"], ".", color="C3",
            markersize=1.5, alpha=0.6, label="Beacon → Drone (noisy)")
    ax.set_ylabel("Range (m)")
    ax.set_xlabel("Time (s)")
    ax.legend(fontsize=7, loc="upper right")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.close(fig)


# ---------------------------------------------------------------------------
# Per-CSV processing
# ---------------------------------------------------------------------------

def process_csv(csv_path: str, output_dir: str):
    """Run EKF, generate beacon & distance data, save everything, and plot."""
    name = Path(csv_path).stem
    print(f"\n{'=' * 60}")
    print(f"  Processing: {name}")
    print(f"{'=' * 60}")

    # 1. Run tuned EKF
    params = EKFParams(**TUNED_PARAMS)
    ekf_results = run_ekf(csv_path, params=params)
    time = ekf_results["time"].to_numpy()

    rng = np.random.default_rng(seed=42)

    # 2. Save drone estimates
    est_cols = ["time", "x", "y", "z", "vx", "vy", "vz",
                "vx_b", "vy_b", "vz_b", "roll", "pitch", "yaw"]
    drone_est = ekf_results[[c for c in est_cols if c in ekf_results.columns]].copy()
    drone_path = os.path.join(output_dir, f"{name}_drone_estimates.csv")
    drone_est.to_csv(drone_path, index=False)
    print(f"  ✓ Drone estimates  → {drone_path}")

    # 3. Generate beacon estimates
    beacon_est = generate_beacon_estimates(time, rng)
    beacon_path = os.path.join(output_dir, f"{name}_beacon_estimates.csv")
    beacon_est.to_csv(beacon_path, index=False)
    print(f"  ✓ Beacon estimates → {beacon_path}")

    # 4. Generate distance measurements
    has_mocap = "ls_x" in ekf_results.columns
    if has_mocap:
        drone_pos = ekf_results[["ls_x", "ls_y", "ls_z"]].to_numpy()
    else:
        drone_pos = ekf_results[["x", "y", "z"]].to_numpy()
        print(f"  ⚠ No mocap data — using EKF positions for distances")

    beacon_pos_noisy = beacon_est[["x", "y", "z"]].to_numpy()
    dist_df = generate_distances(time, drone_pos, beacon_pos_noisy, rng)
    dist_path = os.path.join(output_dir, f"{name}_distances.csv")
    dist_df.to_csv(dist_path, index=False)
    print(f"  ✓ Distances        → {dist_path}")

    # 5. Plot
    plot_path = os.path.join(output_dir, f"{name}_trajectory.png")
    plot_trajectories(drone_est, beacon_est, dist_df, ekf_results,
                      name, save_path=plot_path)
    print(f"  ✓ Plot             → {plot_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Generate beacon companion data and distance measurements "
                    "for EKF flight logs"
    )
    parser.add_argument("--csv", type=str, nargs="+", default=None,
                        help="CSV file(s) to process (default: all in DataFlapperEKF/)")
    parser.add_argument("--output-dir", type=str, default="generated_data",
                        help="Output directory (default: generated_data)")
    args = parser.parse_args()

    csv_files = args.csv if args.csv else ALL_CSVS
    csv_files = [f for f in csv_files if Path(f).exists()]

    if not csv_files:
        print("Error: no CSV files found.")
        return

    os.makedirs(args.output_dir, exist_ok=True)
    print(f"Processing {len(csv_files)} CSV file(s)")
    print(f"Output directory: {args.output_dir}/")

    for csv_path in csv_files:
        process_csv(csv_path, args.output_dir)

    print(f"\n{'=' * 60}")
    print(f"  Done! All outputs saved to {args.output_dir}/")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
