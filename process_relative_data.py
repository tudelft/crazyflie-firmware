#!/usr/bin/env python3
"""
Process Flapper drone relative localization flight data.

1. Convert binary SD logs to CSVs
2. Parse OptiTrack mocap ground truth → Crazyflie frame
3. Synchronize drone/mocap time using height cross-correlation
4. Plot raw drone data + mocap ground truth
5. Run relative EKF (drone pairs) and compare to mocap relative positions
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import correlate
from scipy.spatial.transform import Rotation

try:
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    HAS_3D = True
except ImportError:
    HAS_3D = False

sys.path.insert(0, str(Path(__file__).parent / "tools" / "usdlog"))
import cfusdlog

from ekf_utils import EKF, wrapToPi

# ──────────────────────────────────────────────────────────────────────
# Paths
# ──────────────────────────────────────────────────────────────────────

DATA_DIR   = Path("relative_data")
OUTPUT_DIR = Path("relative_data_plots")

DRONE_FILES = {
    0: DATA_DIR / "flap0log00",
    1: DATA_DIR / "flap1log00",
    2: DATA_DIR / "flap2log00",
}
MOCAP_FILE = DATA_DIR / "good_sd.csv"

DOWNSAMPLE_EKF = 5  # drone logs ~500 Hz → ~100 Hz for EKF

# ──────────────────────────────────────────────────────────────────────
# EKF tuning
# ──────────────────────────────────────────────────────────────────────

Q_4 = np.diag([1e-3, 5e-3, 5e-3, 2e-3])
Q_3 = np.diag([1e-3, 5e-3, 5e-3])
R   = np.array([[0.10**2]])

P0_4 = np.diag([0.5**2, 1.0**2, 1.0**2, 0.5**2])
P0_3 = np.diag([0.5**2, 1.0**2, 1.0**2])


# ──────────────────────────────────────────────────────────────────────
# 1. Decode binary logs → CSVs
# ──────────────────────────────────────────────────────────────────────

def decode_drone_logs():
    """Decode binary SD logs and save as CSVs.  Returns dict of DataFrames."""
    import pandas as pd

    dfs = {}
    for drone_id, path in DRONE_FILES.items():
        csv_path = path.with_suffix(".csv")
        log_data = cfusdlog.decode(str(path))
        data = log_data["fixedFrequency"]
        df = pd.DataFrame(data)
        # Timestamp from ms → seconds (relative to boot)
        df["time_s"] = (df["timestamp"] - df["timestamp"].iloc[0]) / 1000.0
        df.to_csv(csv_path, index=False)
        print(f"Drone {drone_id}: {len(df)} rows → {csv_path}")
        dfs[drone_id] = df
    return dfs


# ──────────────────────────────────────────────────────────────────────
# 2. Parse OptiTrack mocap CSV → Crazyflie frame
# ──────────────────────────────────────────────────────────────────────

def parse_mocap():
    """Parse OptiTrack CSV and convert to Crazyflie XYZ FLU frame.

    OptiTrack global frame: Z-forward, X-left, Y-up  (ZXY)
    Crazyflie world frame:  X-forward, Y-left, Z-up  (XYZ FLU)

    Mapping (from send_extpose convention):
        cf_x = ot_z,  cf_y = ot_x,  cf_z = ot_y
        cf_qx = ot_qz, cf_qy = ot_qx, cf_qz = ot_qy, cf_qw = ot_qw
    """
    import pandas as pd

    # Parse the multi-header OptiTrack CSV manually
    rows = []
    with open(MOCAP_FILE, "r") as f:
        reader = csv.reader(f)
        for i, row in enumerate(reader):
            rows.append(row)

    # Row 0: metadata, Row 1: blank, Row 2: Type, Row 3: Name,
    # Row 4: ID, Row 5: Rotation/Position, Row 6: X/Y/Z/W, Row 7+: data

    # Extract capture rate from metadata
    meta = rows[0]
    frame_rate = float(meta[meta.index("Export Frame Rate") + 1])
    print(f"Mocap frame rate: {frame_rate} Hz")

    # Build column mapping from the header rows
    names = rows[3]       # '', 'Name', 'flapper_00', ...
    meas_type = rows[5]   # '', '', 'Rotation', 'Rotation', ...
    component = rows[6]   # 'Frame', 'Time (Seconds)', 'X', 'Y', ...

    # Per-drone data: columns are Rot(X,Y,Z,W), Pos(X,Y,Z) for each
    drones = {}
    col = 2  # skip Frame, Time
    while col < len(names):
        name = names[col]
        # Each drone has 7 columns: 4 rotation + 3 position
        # Rotation: X, Y, Z, W  then  Position: X, Y, Z
        drones[name] = {"rot_cols": list(range(col, col + 4)),
                        "pos_cols": list(range(col + 4, col + 7))}
        col += 7

    # Parse data rows
    data_rows = rows[7:]
    n = len(data_rows)
    time = np.zeros(n)
    mocap = {}
    for dname in drones:
        mocap[dname] = {
            "time": np.zeros(n),
            # CF frame positions and quaternions
            "x": np.zeros(n), "y": np.zeros(n), "z": np.zeros(n),
            "qx": np.zeros(n), "qy": np.zeros(n),
            "qz": np.zeros(n), "qw": np.zeros(n),
        }

    for i, row in enumerate(data_rows):
        if len(row) < 2 or row[1] == "":
            continue
        time[i] = float(row[1])
        for dname, cols in drones.items():
            # OptiTrack: rotation qx,qy,qz,qw then position x,y,z
            # Handle tracking dropouts (empty cells)
            try:
                ot_qx = float(row[cols["rot_cols"][0]])
                ot_qy = float(row[cols["rot_cols"][1]])
                ot_qz = float(row[cols["rot_cols"][2]])
                ot_qw = float(row[cols["rot_cols"][3]])
                ot_px = float(row[cols["pos_cols"][0]])
                ot_py = float(row[cols["pos_cols"][1]])
                ot_pz = float(row[cols["pos_cols"][2]])
            except (ValueError, IndexError):
                # Tracking dropout — leave as NaN (will be interpolated later)
                mocap[dname]["x"][i] = np.nan
                mocap[dname]["y"][i] = np.nan
                mocap[dname]["z"][i] = np.nan
                mocap[dname]["qx"][i] = np.nan
                mocap[dname]["qy"][i] = np.nan
                mocap[dname]["qz"][i] = np.nan
                mocap[dname]["qw"][i] = np.nan
                continue

            # Convert OptiTrack (ZXY) → Crazyflie (XYZ FLU)
            mocap[dname]["time"][i] = time[i]
            mocap[dname]["x"][i] = ot_pz    # cf_x = ot_z
            mocap[dname]["y"][i] = ot_px    # cf_y = ot_x
            mocap[dname]["z"][i] = ot_py    # cf_z = ot_y (height)
            mocap[dname]["qx"][i] = ot_qz   # cf_qx = ot_qz
            mocap[dname]["qy"][i] = ot_qx   # cf_qy = ot_qx
            mocap[dname]["qz"][i] = ot_qy   # cf_qz = ot_qy
            mocap[dname]["qw"][i] = ot_qw   # cf_qw = ot_qw

    # Interpolate NaN gaps from tracking dropouts
    for dname in mocap:
        m = mocap[dname]
        valid = ~np.isnan(m["x"])
        if not valid.all():
            n_bad = (~valid).sum()
            print(f"  {dname}: interpolating {n_bad} dropout frames")
            for key in ["x", "y", "z", "qx", "qy", "qz", "qw"]:
                m[key] = np.interp(time, time[valid], m[key][valid])

    # Compute yaw from quaternion (ZYX Euler: yaw = rotation about Z)
    for dname in mocap:
        m = mocap[dname]
        quats = np.column_stack([m["qx"], m["qy"], m["qz"], m["qw"]])
        # Normalize quaternions (may be slightly off after interpolation)
        norms = np.linalg.norm(quats, axis=1, keepdims=True)
        quats = quats / np.clip(norms, 1e-12, None)
        rot = Rotation.from_quat(quats)  # scipy uses [x,y,z,w]
        euler = rot.as_euler("ZYX", degrees=False)  # [yaw, pitch, roll]
        m["yaw"] = euler[:, 0]
        m["pitch"] = euler[:, 1]
        m["roll"] = euler[:, 2]

    # Map drone names to IDs
    sorted_names = sorted(drones.keys())
    mocap_by_id = {}
    for i, name in enumerate(sorted_names):
        mocap_by_id[i] = mocap[name]
        mocap_by_id[i]["name"] = name
        print(f"Mocap drone {i}: {name}, {n} frames, "
              f"x=[{mocap[name]['x'].min():.2f}, {mocap[name]['x'].max():.2f}], "
              f"z=[{mocap[name]['z'].min():.2f}, {mocap[name]['z'].max():.2f}]")

    return mocap_by_id


# ──────────────────────────────────────────────────────────────────────
# 3. Synchronize drone data with mocap via height cross-correlation
# ──────────────────────────────────────────────────────────────────────

def sync_time_offset(drone_time, drone_height, mocap_time, mocap_height):
    """Find time offset to align drone height signal with mocap height.

    Returns offset such that: mocap_time_aligned = mocap_time + offset
    maps mocap timestamps into drone time coordinates.

    Actually returns the offset to ADD to drone_time to get mocap_time:
        drone_time + offset ≈ mocap_time
    Or equivalently, the drone event at drone_time corresponds to
    mocap event at (drone_time + offset).
    """
    # Resample both to a common uniform grid
    dt_common = 0.01  # 100 Hz
    t0 = 0.0
    t1 = max(drone_time[-1] - drone_time[0], mocap_time[-1] - mocap_time[0])

    t_grid = np.arange(0, t1, dt_common)

    # Drone height on uniform grid (relative to its own start)
    drone_t_rel = drone_time - drone_time[0]
    h_drone = np.interp(t_grid, drone_t_rel, drone_height, left=0, right=0)

    # Mocap height on same grid
    mocap_t_rel = mocap_time - mocap_time[0]
    h_mocap = np.interp(t_grid, mocap_t_rel, mocap_height, left=0, right=0)

    # Normalize
    h_drone = (h_drone - h_drone.mean()) / (h_drone.std() + 1e-12)
    h_mocap = (h_mocap - h_mocap.mean()) / (h_mocap.std() + 1e-12)

    # Cross-correlate
    corr = correlate(h_drone, h_mocap, mode="full")
    lags = np.arange(-len(h_mocap) + 1, len(h_drone)) * dt_common
    best = np.argmax(corr)
    lag = lags[best]

    # lag > 0 means drone signal is delayed relative to mocap
    # drone_t_rel[drone_event] = mocap_t_rel[same_event] + lag
    # So: mocap_t_aligned = drone_time[0] + mocap_t_rel - lag
    #     offset to convert drone_time to mocap_time frame:
    #     mocap_time_abs = drone_time - drone_time[0] + lag + mocap_time[0]
    # Simplified: time_in_mocap = drone_time - drone_time[0] + lag + mocap_time[0]

    print(f"  Cross-corr lag: {lag:.3f}s (peak corr: {corr[best]:.1f})")
    return lag


def find_mocap_mapping(dfs, mocap):
    """Determine which mocap rigid body corresponds to which SD log drone.

    Strategy:
    1. Identify which mocap bodies are actually flying (height variance > threshold)
    2. Cross-correlate drone 0's height (kalman.statePZ) against each flying mocap body
    3. Cross-correlate communicated heights (ranging.heightN) against remaining bodies
    4. Return mapping: sd_drone_id → mocap_id
    """
    # Determine which mocap bodies are flying
    flying_mocap = {}
    static_mocap = {}
    for mid, m in mocap.items():
        z_range = np.nanmax(m["z"]) - np.nanmin(m["z"])
        if z_range > 0.3:  # at least 30cm height change → flying
            flying_mocap[mid] = m
            print(f"  Mocap {mid} ({m['name']}): FLYING (z range={z_range:.2f}m)")
        else:
            static_mocap[mid] = m
            print(f"  Mocap {mid} ({m['name']}): STATIC (z range={z_range:.2f}m)")

    if len(flying_mocap) < 2:
        print("  WARNING: Fewer than 2 flying mocap bodies found!")

    # Cross-correlate drone 0's own height against all flying bodies
    d0 = dfs[0]
    ts0 = (d0["timestamp"].values - d0["timestamp"].values[0]) / 1000.0
    h0 = d0["kalman.statePZ"].values

    dt_common = 0.01
    t_max = max(ts0[-1], max(m["time"][-1] - m["time"][0] for m in mocap.values()))
    t_grid = np.arange(0, t_max, dt_common)
    h0_grid = np.interp(t_grid, ts0, h0, left=0, right=0)
    h0_norm = (h0_grid - h0_grid.mean()) / (h0_grid.std() + 1e-12)

    best_match = {}
    all_scores = {}
    for mid, m in flying_mocap.items():
        mt_rel = m["time"] - m["time"][0]
        valid = ~np.isnan(m["z"])
        hm = np.interp(t_grid, mt_rel[valid], m["z"][valid], left=0, right=0)
        hm_norm = (hm - hm.mean()) / (hm.std() + 1e-12)
        corr = correlate(h0_norm, hm_norm, mode="full")
        lags = np.arange(-len(hm_norm) + 1, len(h0_norm)) * dt_common
        best_idx = np.argmax(corr)
        all_scores[(0, mid)] = (corr[best_idx], lags[best_idx])
        print(f"  Drone 0 vs mocap {mid}: peak={corr[best_idx]:.0f}, lag={lags[best_idx]:.2f}s")

    # Also cross-correlate communicated heights from drone 0's log
    for other_drone_id in [1, 2]:
        col = f"ranging.height{other_drone_id}"
        if col not in d0.columns:
            continue
        h_other = d0[col].values
        h_other_grid = np.interp(t_grid, ts0, h_other, left=0, right=0)
        h_other_norm = (h_other_grid - h_other_grid.mean()) / (h_other_grid.std() + 1e-12)

        for mid, m in flying_mocap.items():
            mt_rel = m["time"] - m["time"][0]
            valid = ~np.isnan(m["z"])
            hm = np.interp(t_grid, mt_rel[valid], m["z"][valid], left=0, right=0)
            hm_norm = (hm - hm.mean()) / (hm.std() + 1e-12)
            corr = correlate(h_other_norm, hm_norm, mode="full")
            lags = np.arange(-len(hm_norm) + 1, len(h_other_norm)) * dt_common
            best_idx = np.argmax(corr)
            all_scores[(other_drone_id, mid)] = (corr[best_idx], lags[best_idx])
            print(f"  Drone {other_drone_id} (comm'd height) vs mocap {mid}: "
                  f"peak={corr[best_idx]:.0f}, lag={lags[best_idx]:.2f}s")

    # Greedy assignment: for each SD drone, pick best mocap match
    # (only from flying bodies)
    flying_ids = set(flying_mocap.keys())
    mapping = {}  # sd_drone_id → mocap_id
    used = set()

    # Sort by correlation strength (descending) for greedy assignment
    score_list = [(score, lag, did, mid) for (did, mid), (score, lag) in all_scores.items()]
    score_list.sort(key=lambda x: -x[0])

    for score, lag, did, mid in score_list:
        if did not in mapping and mid not in used:
            mapping[did] = mid
            used.add(mid)
            print(f"  → Mapping: SD drone {did} = mocap {mid} "
                  f"({mocap[mid]['name']}, corr={score:.0f}, lag={lag:.2f}s)")

    # Unmapped drones without flying mocap data
    for did in dfs:
        if did not in mapping:
            print(f"  → SD drone {did}: no flying mocap body matched")

    return mapping, all_scores


def sync_all(dfs, mocap):
    """Synchronize all drone logs with mocap using height cross-correlation.

    Each drone has its own boot clock. We sync drone 0 against its matched
    mocap body, then use the boot-time offsets to place drones 1 and 2.
    """
    print("\nIdentifying mocap ↔ drone mapping:")
    mapping, scores = find_mocap_mapping(dfs, mocap)

    if 0 not in mapping:
        print("  ERROR: Could not match drone 0 to any mocap body!")
        print("  Falling back to mocap body 0")
        mapping[0] = sorted(mocap.keys())[0]

    # Sync drone 0 against its matched mocap body
    d0 = dfs[0]
    drone0_time = d0["timestamp"].values / 1000.0
    drone0_height = d0["kalman.statePZ"].values

    matched_mocap_id = mapping[0]
    mocap_matched = mocap[matched_mocap_id]
    mt = mocap_matched["time"]
    mh = mocap_matched["z"]

    print(f"\nSyncing drone 0 with mocap {matched_mocap_id} ({mocap_matched['name']}):")
    lag = sync_time_offset(drone0_time, drone0_height, mt, mh)

    # All drones in the SD log use their own boot clocks. But the communicated
    # heights in drone 0's log are all on drone 0's clock. So we can use
    # drone 0's clock as the reference for all.
    t0_drone0 = drone0_time[0]
    t0_mocap = mt[0]

    offsets = {}
    for drone_id in dfs:
        drone_time = dfs[drone_id]["timestamp"].values / 1000.0
        # Convert to mocap time:
        # For drone 0: t_mocap = (t_boot - t0_drone0) + lag + t0_mocap
        # For other drones: they have their OWN boot clocks.
        # But for EKF we primarily use drone 0's log (which has all heights/distances).
        # So we put everything in drone 0's clock → mocap time mapping.
        if drone_id == 0:
            mocap_aligned = drone_time - t0_drone0 + lag + t0_mocap
        else:
            # Other drones have their own clocks; we align them using
            # the communicated data in drone 0's log (already on drone 0's clock)
            mocap_aligned = drone_time  # placeholder, not directly usable
        offsets[drone_id] = {
            "drone_time_s": drone_time,
            "mocap_time_s": mocap_aligned if drone_id == 0 else drone_time,
        }
        if drone_id == 0:
            print(f"  Drone {drone_id}: boot=[{drone_time[0]:.1f}, {drone_time[-1]:.1f}]s → "
                  f"mocap=[{mocap_aligned[0]:.1f}, {mocap_aligned[-1]:.1f}]s")

    # Store the mapping and time conversion params for later use
    offsets["_mapping"] = mapping
    offsets["_t0_drone0"] = t0_drone0
    offsets["_lag"] = lag
    offsets["_t0_mocap"] = t0_mocap

    return offsets, lag


# ──────────────────────────────────────────────────────────────────────
# 4. Plotting: raw data
# ──────────────────────────────────────────────────────────────────────

def plot_raw_drone_data(dfs, out_dir):
    """Plot raw data for each drone."""
    out_dir.mkdir(parents=True, exist_ok=True)

    for drone_id, df in dfs.items():
        t = df["time_s"].values

        # Figure out which other drones this one logs
        other_ids = sorted(
            int(c.split("distance")[1])
            for c in df.columns if c.startswith("ranging.distance")
        )

        fig, axes = plt.subplots(5, 1, figsize=(14, 16), sharex=True)
        fig.suptitle(f"Drone {drone_id} — Raw SD Log Data", fontsize=14, fontweight="bold")

        # Heights
        ax = axes[0]
        for oid in range(3):
            col = f"ranging.height{oid}"
            if col in df.columns:
                h = df[col].values
                label = f"height{oid}" + (" (self)" if oid == drone_id else "")
                if oid == drone_id:
                    ax.plot(t, h, "--", alpha=0.5, label=label)
                else:
                    ax.plot(t, h, alpha=0.8, label=label)
        if "kalman.statePZ" in df.columns:
            ax.plot(t, df["kalman.statePZ"].values, "k-", lw=1.5,
                    label="kalman.statePZ")
        ax.set_ylabel("Height [m]")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # Distances (mm → m)
        ax = axes[1]
        for oid in other_ids:
            col = f"ranging.distance{oid}"
            ax.plot(t, df[col].values / 1000.0, alpha=0.8,
                    label=f"distance to {oid}")
        ax.set_ylabel("Distance [m]")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # Relative yaw
        ax = axes[2]
        for oid in other_ids:
            col = f"ranging.yawR{oid}"
            if col in df.columns:
                ax.plot(t, df[col].values, alpha=0.8, label=f"yawR{oid}")
        ax.set_ylabel("Relative Yaw [deg]")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # Velocities of others
        ax = axes[3]
        for oid in other_ids:
            for comp in ["vx", "vy", "vz"]:
                col = f"ranging.{comp}{oid}"
                if col in df.columns:
                    ax.plot(t, df[col].values, alpha=0.6,
                            label=f"{comp}{oid}")
        ax.set_ylabel("Velocity [m/s]")
        ax.legend(fontsize=7, ncol=3)
        ax.grid(True, alpha=0.3)

        # Own Kalman state + yaw rate
        ax = axes[4]
        ax.plot(t, df["kalman.statePX"].values, label="statePX")
        ax.plot(t, df["kalman.statePY"].values, label="statePY")
        ax.plot(t, df["kalman.statePZ"].values, label="statePZ")
        ax2 = ax.twinx()
        ax2.plot(t, df["stateEstimateZ.rateYaw"].values, "k-",
                 alpha=0.4, label="rateYaw")
        ax2.set_ylabel("Yaw rate [deg/s]")
        ax2.legend(loc="upper left", fontsize=8)
        ax.set_ylabel("Kalman state")
        ax.set_xlabel("Time [s]")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        fig.savefig(out_dir / f"drone{drone_id}_raw.png", dpi=150)
        plt.close(fig)
        print(f"  Saved drone{drone_id}_raw.png")


def plot_mocap_data(mocap, out_dir):
    """Plot mocap trajectories for all drones."""
    out_dir.mkdir(parents=True, exist_ok=True)
    colors = ["tab:blue", "tab:orange", "tab:green"]

    # 3D trajectory
    if HAS_3D:
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection="3d")
        for i in sorted(mocap.keys()):
            m = mocap[i]
            ax.plot(m["x"], m["y"], m["z"], color=colors[i],
                    alpha=0.7, label=f"Drone {i}")
            ax.scatter(m["x"][0], m["y"][0], m["z"][0], color=colors[i],
                       marker="o", s=60)
        ax.set_xlabel("X (forward) [m]")
        ax.set_ylabel("Y (left) [m]")
        ax.set_zlabel("Z (up) [m]")
        ax.set_title("Mocap 3D Trajectories (CF Frame)")
        ax.legend()
        fig.savefig(out_dir / "mocap_3d.png", dpi=150)
        plt.close(fig)
    else:
        # Fallback: 2D top-down view
        fig, ax = plt.subplots(figsize=(10, 8))
        for i in sorted(mocap.keys()):
            m = mocap[i]
            ax.plot(m["x"], m["y"], color=colors[i], alpha=0.7, label=f"Drone {i}")
            ax.scatter(m["x"][0], m["y"][0], color=colors[i], marker="o", s=60)
        ax.set_xlabel("X (forward) [m]")
        ax.set_ylabel("Y (left) [m]")
        ax.set_title("Mocap 2D Trajectories (CF Frame, top-down)")
        ax.legend(); ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
        fig.savefig(out_dir / "mocap_3d.png", dpi=150)
        plt.close(fig)

    # Position vs time
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle("Mocap Positions & Yaw (CF Frame)", fontsize=14, fontweight="bold")
    for i in sorted(mocap.keys()):
        m = mocap[i]
        t = m["time"]
        axes[0].plot(t, m["x"], color=colors[i], alpha=0.8, label=f"Drone {i}")
        axes[1].plot(t, m["y"], color=colors[i], alpha=0.8, label=f"Drone {i}")
        axes[2].plot(t, m["z"], color=colors[i], alpha=0.8, label=f"Drone {i}")
        axes[3].plot(t, np.rad2deg(m["yaw"]), color=colors[i], alpha=0.8,
                     label=f"Drone {i}")
    axes[0].set_ylabel("X [m]"); axes[0].legend(fontsize=8); axes[0].grid(True, alpha=0.3)
    axes[1].set_ylabel("Y [m]"); axes[1].legend(fontsize=8); axes[1].grid(True, alpha=0.3)
    axes[2].set_ylabel("Z [m]"); axes[2].legend(fontsize=8); axes[2].grid(True, alpha=0.3)
    axes[3].set_ylabel("Yaw [deg]"); axes[3].set_xlabel("Mocap time [s]")
    axes[3].legend(fontsize=8); axes[3].grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(out_dir / "mocap_pos_yaw.png", dpi=150)
    plt.close(fig)
    print("  Saved mocap_3d.png, mocap_pos_yaw.png")


def plot_sync_verification(dfs, mocap, offsets, out_dir):
    """Overlay drone and mocap heights to verify synchronization."""
    out_dir.mkdir(parents=True, exist_ok=True)
    mapping = offsets.get("_mapping", {})
    t0_drone0 = offsets["_t0_drone0"]
    lag = offsets["_lag"]
    t0_mocap = offsets["_t0_mocap"]

    # Convert drone 0's time to mocap time
    d0 = dfs[0]
    ts0 = d0["timestamp"].values / 1000.0
    ts0_mocap = ts0 - t0_drone0 + lag + t0_mocap

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle("Height Sync Verification: Drone vs Mocap", fontsize=14, fontweight="bold")

    # Panel 0: Drone 0's own height vs matched mocap body
    ax = axes[0]
    ax.plot(ts0_mocap, d0["kalman.statePZ"].values, "b-", alpha=0.7, lw=1,
            label="Drone 0 kalman.statePZ")
    if 0 in mapping:
        m = mocap[mapping[0]]
        ax.plot(m["time"], m["z"], "r-", lw=2, alpha=0.8,
                label=f"Mocap {mapping[0]} ({m['name']})")
    ax.set_ylabel("Drone 0\nHeight [m]")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)

    # Panel 1: Drone 1's communicated height vs matched mocap body
    ax = axes[1]
    col = "ranging.height1"
    if col in d0.columns:
        ax.plot(ts0_mocap, d0[col].values, "b-", alpha=0.7, lw=1,
                label="Drone 0 sees height1")
    if 1 in mapping:
        m = mocap[mapping[1]]
        ax.plot(m["time"], m["z"], "r-", lw=2, alpha=0.8,
                label=f"Mocap {mapping[1]} ({m['name']})")
    ax.set_ylabel("Drone 1\nHeight [m]")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)

    # Panel 2: Drone 2's communicated height vs matched mocap body
    ax = axes[2]
    col = "ranging.height2"
    if col in d0.columns:
        ax.plot(ts0_mocap, d0[col].values, "b-", alpha=0.7, lw=1,
                label="Drone 0 sees height2")
    if 2 in mapping:
        m = mocap[mapping[2]]
        ax.plot(m["time"], m["z"], "r-", lw=2, alpha=0.8,
                label=f"Mocap {mapping[2]} ({m['name']})")
    ax.set_ylabel("Drone 2\nHeight [m]")
    ax.set_xlabel("Mocap time [s]")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    fig.savefig(out_dir / "sync_verification.png", dpi=150)
    plt.close(fig)
    print("  Saved sync_verification.png")


def plot_mocap_distances(mocap, out_dir):
    """Plot inter-drone distances from mocap."""
    out_dir.mkdir(parents=True, exist_ok=True)
    t = mocap[0]["time"]
    pairs = [(0, 1), (0, 2), (1, 2)]
    colors = ["tab:blue", "tab:orange", "tab:green"]

    fig, ax = plt.subplots(figsize=(14, 5))
    for idx, (i, j) in enumerate(pairs):
        dx = mocap[j]["x"] - mocap[i]["x"]
        dy = mocap[j]["y"] - mocap[i]["y"]
        dz = mocap[j]["z"] - mocap[i]["z"]
        d = np.sqrt(dx**2 + dy**2 + dz**2)
        ax.plot(t, d, color=colors[idx], alpha=0.8, label=f"d({i}→{j})")
    ax.set_ylabel("Distance [m]")
    ax.set_xlabel("Mocap time [s]")
    ax.set_title("Mocap Inter-Drone Distances")
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(out_dir / "mocap_distances.png", dpi=150)
    plt.close(fig)
    print("  Saved mocap_distances.png")


def plot_drone_vs_mocap_distances(dfs, mocap, offsets, out_dir):
    """Overlay drone-measured distances with mocap ground truth distances."""
    out_dir.mkdir(parents=True, exist_ok=True)
    mapping = offsets.get("_mapping", {})
    t0_drone0 = offsets["_t0_drone0"]
    lag = offsets["_lag"]
    t0_mocap = offsets["_t0_mocap"]

    # Only run from drone 0's perspective (it has the best data)
    df = dfs[0]
    ts0 = df["timestamp"].values / 1000.0
    ts0_mocap = ts0 - t0_drone0 + lag + t0_mocap

    other_ids = sorted(
        int(c.split("distance")[1])
        for c in df.columns if c.startswith("ranging.distance")
    )

    fig, axes = plt.subplots(len(other_ids), 1, figsize=(14, 4 * len(other_ids)),
                             sharex=True)
    if len(other_ids) == 1:
        axes = [axes]
    fig.suptitle("Drone 0: UWB Range vs Mocap Distance", fontsize=14)

    for idx, oid in enumerate(other_ids):
        ax = axes[idx]
        # UWB measurement (mm → m)
        d_drone = df[f"ranging.distance{oid}"].values / 1000.0
        ax.plot(ts0_mocap, d_drone, "b-", alpha=0.5, lw=0.8,
                label=f"UWB distance to drone {oid}")

        # Mocap ground truth (if both drones have mocap mapping)
        if 0 in mapping and oid in mapping:
            mid_self = mapping[0]
            mid_other = mapping[oid]
            mt = mocap[mid_self]["time"]
            dx = mocap[mid_other]["x"] - mocap[mid_self]["x"]
            dy = mocap[mid_other]["y"] - mocap[mid_self]["y"]
            dz = mocap[mid_other]["z"] - mocap[mid_self]["z"]
            d_mocap = np.sqrt(dx**2 + dy**2 + dz**2)
            ax.plot(mt, d_mocap, "r-", lw=1.5, alpha=0.8,
                    label=f"Mocap GT ({mocap[mid_self]['name']}→{mocap[mid_other]['name']})")
        else:
            ax.text(0.5, 0.5, f"No mocap mapping for drone {oid}",
                    transform=ax.transAxes, ha="center", va="center", fontsize=12, color="red")

        ax.set_ylabel(f"Dist 0→{oid} [m]")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time [s]")
    plt.tight_layout()
    fig.savefig(out_dir / "drone0_dist_comparison.png", dpi=150)
    plt.close(fig)
    print(f"  Saved drone0_dist_comparison.png")


# ──────────────────────────────────────────────────────────────────────
# 5. Relative EKF
# ──────────────────────────────────────────────────────────────────────

def f_4state(x, u):
    """4-state relative dynamics: x = [ψ₁₂, x₁₂, y₁₂, z₁₂]"""
    psi12, x12, y12, z12 = x
    psi_dot_i, vxi, vyi, vzi, psi_dot_j, vxj, vyj, vzj = u[:8]

    c12, s12 = np.cos(psi12), np.sin(psi12)

    return np.array([
        psi_dot_j - psi_dot_i,
        (c12 * vxj - s12 * vyj) - vxi + psi_dot_i * y12,
        (s12 * vxj + c12 * vyj) - vyi - psi_dot_i * x12,
        vzj - vzi,
    ])


def h_4state(x, _u):
    """Measurement: range d₁₂ = ‖[x₁₂, y₁₂, z₁₂]‖"""
    _, x12, y12, z12 = x
    return np.array([max(np.sqrt(x12**2 + y12**2 + z12**2), 1e-8)])


def f_3state(x, u):
    """3-state relative dynamics (z₁₂ known): x = [ψ₁₂, x₁₂, y₁₂]"""
    psi12, x12, y12 = x
    psi_dot_i, vxi, vyi, _vzi, psi_dot_j, vxj, vyj, _vzj, _z12 = u[:9]

    c12, s12 = np.cos(psi12), np.sin(psi12)

    return np.array([
        psi_dot_j - psi_dot_i,
        (c12 * vxj - s12 * vyj) - vxi + psi_dot_i * y12,
        (s12 * vxj + c12 * vyj) - vyi - psi_dot_i * x12,
    ])


def h_3state(x, u):
    """Measurement: range with z₁₂ from input."""
    _, x12, y12 = x
    z12 = u[8]
    return np.array([max(np.sqrt(x12**2 + y12**2 + z12**2), 1e-8)])


def mocap_body_frame_velocity(mocap_drone, dt_smooth=0.05):
    """Compute body-frame velocity from mocap world-frame position + yaw.

    Returns vx_body, vy_body, vz_body (in body FLU frame).
    """
    t = mocap_drone["time"]
    x = mocap_drone["x"]
    y = mocap_drone["y"]
    z = mocap_drone["z"]
    yaw = mocap_drone["yaw"]

    # World-frame velocity via smooth numerical differentiation
    vx_w = np.gradient(x, t)
    vy_w = np.gradient(y, t)
    vz_w = np.gradient(z, t)

    # Rotate world → body (yaw only, assuming small pitch/roll)
    c = np.cos(yaw)
    s = np.sin(yaw)
    vx_b =  c * vx_w + s * vy_w
    vy_b = -s * vx_w + c * vy_w
    vz_b = vz_w

    return vx_b, vy_b, vz_b


def mocap_relative_state(mocap_i, mocap_j):
    """Compute ground-truth relative state of drone j in drone i's body frame.

    Returns psi12, x12, y12, z12, d12 (all at mocap timestamps).
    """
    dx = mocap_j["x"] - mocap_i["x"]
    dy = mocap_j["y"] - mocap_i["y"]
    dz = mocap_j["z"] - mocap_i["z"]
    yaw_i = mocap_i["yaw"]

    # World → body-i rotation (yaw only)
    c = np.cos(yaw_i)
    s = np.sin(yaw_i)
    x12 =  c * dx + s * dy
    y12 = -s * dx + c * dy
    z12 = dz

    psi12 = wrapToPi(mocap_j["yaw"] - mocap_i["yaw"])
    d12 = np.sqrt(dx**2 + dy**2 + dz**2)

    return psi12, x12, y12, z12, d12


def run_relative_ekf_pair(drone_id_i, drone_id_j, dfs, mocap, offsets):
    """Run 4-state and 3-state relative EKF for pair (i, j) using drone i's log.

    Uses:
    - Drone i's body-frame velocity from mocap (since log doesn't have own vel)
    - Drone j's body-frame velocity from ranging.vxJ/vyJ/vzJ in drone i's log
    - Drone i's yaw rate from mocap
    - Drone j's yaw rate from mocap
    - Range measurement from drone i's log
    """
    import pandas as pd

    mapping = offsets.get("_mapping", {})
    t0_drone0 = offsets["_t0_drone0"]
    lag = offsets["_lag"]
    t0_mocap = offsets["_t0_mocap"]

    if drone_id_i not in mapping or drone_id_j not in mapping:
        raise ValueError(f"No mocap mapping for drone {drone_id_i} or {drone_id_j}")

    mocap_i = mocap[mapping[drone_id_i]]
    mocap_j = mocap[mapping[drone_id_j]]

    df = dfs[drone_id_i]
    t_drone_boot = df["timestamp"].values / 1000.0
    # Convert to mocap time (all on drone 0's clock since we use drone 0's log)
    t_mocap_drone = t_drone_boot - t0_drone0 + lag + t0_mocap

    # Downsample
    idx = np.arange(0, len(df), DOWNSAMPLE_EKF)
    t_boot = t_drone_boot[idx]
    t_mocap_d = t_mocap_drone[idx]
    N = len(idx)

    # Range measurement (mm → m)
    d_meas = df[f"ranging.distance{drone_id_j}"].values[idx] / 1000.0

    # Drone j's body-frame velocity from i's log
    vxj = df[f"ranging.vx{drone_id_j}"].values[idx]
    vyj = df[f"ranging.vy{drone_id_j}"].values[idx]
    vzj = df[f"ranging.vz{drone_id_j}"].values[idx]

    # Drone i's body-frame velocity and yaw rate from mocap
    # Interpolate mocap onto drone timestamps
    mt = mocap_i["time"]
    vx_i_mocap, vy_i_mocap, vz_i_mocap = mocap_body_frame_velocity(mocap_i)
    vxi = np.interp(t_mocap_d, mt, vx_i_mocap)
    vyi = np.interp(t_mocap_d, mt, vy_i_mocap)
    vzi = np.interp(t_mocap_d, mt, vz_i_mocap)

    # Yaw rates from mocap (both drones)
    yaw_i = np.interp(t_mocap_d, mt, mocap_i["yaw"])
    yaw_j = np.interp(t_mocap_d, mocap_j["time"], mocap_j["yaw"])
    yaw_i_unwrap = np.unwrap(yaw_i)
    yaw_j_unwrap = np.unwrap(yaw_j)

    dt_arr = np.diff(t_boot)
    dt_med = float(np.median(dt_arr))
    psi_dot_i = np.gradient(yaw_i_unwrap, t_boot)
    psi_dot_j = np.gradient(yaw_j_unwrap, t_boot)

    # Known z12 from mocap
    z_i = np.interp(t_mocap_d, mocap_i["time"], mocap_i["z"])
    z_j = np.interp(t_mocap_d, mocap_j["time"], mocap_j["z"])
    z12_known = z_j - z_i

    # Ground truth relative state from mocap
    psi12_gt_full, x12_gt_full, y12_gt_full, z12_gt_full, d12_gt_full = \
        mocap_relative_state(mocap_i, mocap_j)
    psi12_gt = np.interp(t_mocap_d, mocap_i["time"], psi12_gt_full)
    x12_gt = np.interp(t_mocap_d, mocap_i["time"], x12_gt_full)
    y12_gt = np.interp(t_mocap_d, mocap_i["time"], y12_gt_full)
    z12_gt = np.interp(t_mocap_d, mocap_i["time"], z12_gt_full)
    d_gt = np.interp(t_mocap_d, mocap_i["time"], d12_gt_full)

    # Initial state from mocap ground truth (with small offset to test convergence)
    INIT_OFFSET = 0.3
    psi12_0 = float(psi12_gt[0])
    x12_0 = float(x12_gt[0])
    y12_0 = float(y12_gt[0])
    z12_0 = float(z12_gt[0])

    # ── 4-state EKF ──
    x0_4 = np.array([psi12_0 + np.deg2rad(5), x12_0 + INIT_OFFSET,
                      y12_0 + INIT_OFFSET, z12_0 + INIT_OFFSET])
    u0_4 = np.array([psi_dot_i[0], vxi[0], vyi[0], vzi[0],
                      psi_dot_j[0], vxj[0], vyj[0], vzj[0]])

    ekf4 = EKF(f_4state, h_4state, x0_4, u0_4,
               P0_4.copy(), Q_4.copy(), R.copy(),
               dynamics_type="continuous", discretization_timestep=dt_med)

    for k in range(1, N):
        dt_k = dt_arr[k - 1] if k - 1 < len(dt_arr) else dt_med
        u_k = np.array([psi_dot_i[k], vxi[k], vyi[k], vzi[k],
                        psi_dot_j[k], vxj[k], vyj[k], vzj[k]])
        ekf4.forward_update(np.array([d_meas[k]]), u_k,
                            discretization_timestep=dt_k)

    X4 = np.array(ekf4.history["X"])
    P4 = np.array(ekf4.history["P_diags"])

    # ── 3-state EKF ──
    x0_3 = np.array([psi12_0 + np.deg2rad(5), x12_0 + INIT_OFFSET,
                      y12_0 + INIT_OFFSET])
    u0_3 = np.array([psi_dot_i[0], vxi[0], vyi[0], vzi[0],
                      psi_dot_j[0], vxj[0], vyj[0], vzj[0],
                      z12_known[0]])

    ekf3 = EKF(f_3state, h_3state, x0_3, u0_3,
               P0_3.copy(), Q_3.copy(), R.copy(),
               dynamics_type="continuous", discretization_timestep=dt_med)

    for k in range(1, N):
        dt_k = dt_arr[k - 1] if k - 1 < len(dt_arr) else dt_med
        u_k = np.array([psi_dot_i[k], vxi[k], vyi[k], vzi[k],
                        psi_dot_j[k], vxj[k], vyj[k], vzj[k],
                        z12_known[k]])
        ekf3.forward_update(np.array([d_meas[k]]), u_k,
                            discretization_timestep=dt_k)

    X3 = np.array(ekf3.history["X"])
    P3 = np.array(ekf3.history["P_diags"])

    return dict(
        pair=(drone_id_i, drone_id_j),
        time=t_boot, time_mocap=t_mocap_d,
        X4=X4, X3=X3, P4=P4, P3=P3,
        gt=dict(psi12=psi12_gt, x12=x12_gt, y12=y12_gt,
                z12=z12_gt, d=d_gt),
        d_meas=d_meas, z12_known=z12_known,
    )


def plot_ekf_results(res, out_dir):
    """Plot relative EKF results vs mocap ground truth."""
    out_dir.mkdir(parents=True, exist_ok=True)
    i, j = res["pair"]
    t = res["time_mocap"]
    X4, X3 = res["X4"], res["X3"]
    P4, P3 = res["P4"], res["P3"]
    gt = res["gt"]
    dm = res["d_meas"]
    zk = res["z12_known"]

    fig, axes = plt.subplots(5, 1, figsize=(14, 16), sharex=True)
    fig.suptitle(f"Relative EKF: Drone {i} → Drone {j}", fontsize=14, fontweight="bold")

    # x₁₂
    ax = axes[0]
    ax.plot(t, gt["x12"], "k-", lw=1.5, label="Mocap GT")
    ax.plot(t, X4[:, 1], "b-", alpha=0.8, label="4-state")
    ax.fill_between(t, X4[:, 1] - 2*np.sqrt(P4[:, 1]),
                       X4[:, 1] + 2*np.sqrt(P4[:, 1]), color="b", alpha=0.12)
    ax.plot(t, X3[:, 1], "r--", alpha=0.8, label="3-state")
    ax.fill_between(t, X3[:, 1] - 2*np.sqrt(P3[:, 1]),
                       X3[:, 1] + 2*np.sqrt(P3[:, 1]), color="r", alpha=0.12)
    ax.set_ylabel("x₁₂ [m]"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # y₁₂
    ax = axes[1]
    ax.plot(t, gt["y12"], "k-", lw=1.5, label="Mocap GT")
    ax.plot(t, X4[:, 2], "b-", alpha=0.8, label="4-state")
    ax.fill_between(t, X4[:, 2] - 2*np.sqrt(P4[:, 2]),
                       X4[:, 2] + 2*np.sqrt(P4[:, 2]), color="b", alpha=0.12)
    ax.plot(t, X3[:, 2], "r--", alpha=0.8, label="3-state")
    ax.fill_between(t, X3[:, 2] - 2*np.sqrt(P3[:, 2]),
                       X3[:, 2] + 2*np.sqrt(P3[:, 2]), color="r", alpha=0.12)
    ax.set_ylabel("y₁₂ [m]"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # z₁₂
    ax = axes[2]
    ax.plot(t, gt["z12"], "k-", lw=1.5, label="Mocap GT")
    ax.plot(t, X4[:, 3], "b-", alpha=0.8, label="4-state est.")
    ax.fill_between(t, X4[:, 3] - 2*np.sqrt(P4[:, 3]),
                       X4[:, 3] + 2*np.sqrt(P4[:, 3]), color="b", alpha=0.12)
    ax.plot(t, zk, "r--", alpha=0.8, label="3-state (known z₁₂)")
    ax.set_ylabel("z₁₂ [m]"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # ψ₁₂
    ax = axes[3]
    ax.plot(t, np.rad2deg(gt["psi12"]), "k-", lw=1.5, label="Mocap GT")
    ax.plot(t, np.rad2deg(X4[:, 0]), "b-", alpha=0.8, label="4-state")
    ax.fill_between(t,
                    np.rad2deg(X4[:, 0] - 2*np.sqrt(P4[:, 0])),
                    np.rad2deg(X4[:, 0] + 2*np.sqrt(P4[:, 0])),
                    color="b", alpha=0.12)
    ax.plot(t, np.rad2deg(X3[:, 0]), "r--", alpha=0.8, label="3-state")
    ax.fill_between(t,
                    np.rad2deg(X3[:, 0] - 2*np.sqrt(P3[:, 0])),
                    np.rad2deg(X3[:, 0] + 2*np.sqrt(P3[:, 0])),
                    color="r", alpha=0.12)
    ax.set_ylabel("ψ₁₂ [deg]"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # Distance
    ax = axes[4]
    d_est4 = np.sqrt(X4[:, 1]**2 + X4[:, 2]**2 + X4[:, 3]**2)
    d_est3 = np.sqrt(X3[:, 1]**2 + X3[:, 2]**2 + zk**2)
    ax.plot(t, gt["d"], "k-", lw=1.5, label="Mocap GT distance")
    ax.plot(t, dm, "g.", ms=1, alpha=0.25, label="UWB measured")
    ax.plot(t, d_est4, "b-", alpha=0.8, label="4-state")
    ax.plot(t, d_est3, "r--", alpha=0.8, label="3-state")
    ax.set_ylabel("Distance [m]"); ax.set_xlabel("Time [s]")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    fig.savefig(out_dir / f"ekf_{i}_to_{j}.png", dpi=150)
    plt.close(fig)
    print(f"  Saved ekf_{i}_to_{j}.png")


def rmse(est, truth):
    return float(np.sqrt(np.mean((est - truth)**2)))


# ──────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────

def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("1. Decoding binary drone logs")
    print("=" * 60)
    dfs = decode_drone_logs()

    print("\n" + "=" * 60)
    print("2. Parsing mocap data")
    print("=" * 60)
    mocap = parse_mocap()

    print("\n" + "=" * 60)
    print("3. Synchronizing time")
    print("=" * 60)
    offsets, lag = sync_all(dfs, mocap)

    print("\n" + "=" * 60)
    print("4. Plotting raw data")
    print("=" * 60)
    plot_raw_drone_data(dfs, OUTPUT_DIR)
    plot_mocap_data(mocap, OUTPUT_DIR)
    plot_sync_verification(dfs, mocap, offsets, OUTPUT_DIR)
    plot_mocap_distances(mocap, OUTPUT_DIR)
    plot_drone_vs_mocap_distances(dfs, mocap, offsets, OUTPUT_DIR)

    print("\n" + "=" * 60)
    print("5. Running relative EKF")
    print("=" * 60)
    # Run from drone 0's perspective (it has the best data)
    mapping = offsets.get("_mapping", {})
    pairs = [(0, 1), (0, 2)]
    for i, j in pairs:
        if i not in mapping or j not in mapping:
            print(f"\n  Pair {i} → {j}: SKIPPED (no mocap mapping)")
            continue
        print(f"\n  Pair {i} → {j} (mocap {mapping[i]}→{mapping[j]}):")
        try:
            res = run_relative_ekf_pair(i, j, dfs, mocap, offsets)
            plot_ekf_results(res, OUTPUT_DIR)

            r4_x = rmse(res["X4"][:, 1], res["gt"]["x12"])
            r4_y = rmse(res["X4"][:, 2], res["gt"]["y12"])
            r4_z = rmse(res["X4"][:, 3], res["gt"]["z12"])
            r4_psi = rmse(wrapToPi(res["X4"][:, 0] - res["gt"]["psi12"]), 0.0)

            r3_x = rmse(res["X3"][:, 1], res["gt"]["x12"])
            r3_y = rmse(res["X3"][:, 2], res["gt"]["y12"])
            r3_psi = rmse(wrapToPi(res["X3"][:, 0] - res["gt"]["psi12"]), 0.0)

            print(f"    4-state RMSE: x={r4_x:.3f} y={r4_y:.3f} z={r4_z:.3f} "
                  f"ψ={np.rad2deg(r4_psi):.1f}°")
            print(f"    3-state RMSE: x={r3_x:.3f} y={r3_y:.3f} "
                  f"ψ={np.rad2deg(r3_psi):.1f}°")
        except Exception as e:
            print(f"    ERROR: {e}")
            import traceback
            traceback.print_exc()

    print(f"\nAll plots saved to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
