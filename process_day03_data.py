#!/usr/bin/env python3
"""
Process day_03 Flapper drone data — all takes.

Outputs synced_all_500hz.csv in each take directory.
Reuses sync/mocap utilities from process_day02_data.py.

Takes:
  no_drag_take1  — flap0log02, flap1log03, flap2log02  (note: drone1 got log03)
  no_drag_take2  — flap0log01, flap1log01, flap2log01
  drag_take1     — flap0log03, flap1log03, flap2log03

Drone 0 is stationary in all takes.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent / "tools" / "usdlog"))
import cfusdlog

# Import path-agnostic utilities from the day_02 script
sys.path.insert(0, str(Path(__file__).parent))
from process_day02_data import (
    STATIC_DRONE,
    _find_takeoff_time,
    _find_landing_time,
    _xcorr_offset_in_window,
    sync_all,
    drone_to_mocap_time,
    mocap_body_frame_velocity,
    mocap_global_velocity,
    mocap_yaw_rate,
    plot_velocity_alignment_debug,
    plot_yawrate_comparison,
)

# ──────────────────────────────────────────────────────────────────────
# Take definitions
# ──────────────────────────────────────────────────────────────────────

DAY03_DIR = Path("relative_data/day_03")

TAKES = {
    "no_drag_take1": {
        "drone_files": {
            0: DAY03_DIR / "no_drag_take1" / "flap0log02",
            1: DAY03_DIR / "no_drag_take1" / "flap1log03",
            2: DAY03_DIR / "no_drag_take1" / "flap2log02",
        },
        "mocap_file": DAY03_DIR / "no_drag_take1" / "REL_DATA_GODD_SD_NO_DRAG_002.csv",
        "out_dir":    DAY03_DIR / "no_drag_take1",
    },
    "no_drag_take2": {
        "drone_files": {
            0: DAY03_DIR / "no_drag_take2" / "flap0log01",
            1: DAY03_DIR / "no_drag_take2" / "flap1log01",
            2: DAY03_DIR / "no_drag_take2" / "flap2log01",
        },
        "mocap_file": DAY03_DIR / "no_drag_take2" / "REL_DATA_GOOD_SD_NO_DRAG_TAKE2_001.csv",
        "out_dir":    DAY03_DIR / "no_drag_take2",
    },
    "drag_take1": {
        "drone_files": {
            0: DAY03_DIR / "drag_take1" / "flap0log03",
            1: DAY03_DIR / "drag_take1" / "flap1log03",
            2: DAY03_DIR / "drag_take1" / "flap2log03",
        },
        "mocap_file": DAY03_DIR / "drag_take1" / "REL_DATA_GODD_SD_DRAG_003.csv",
        "out_dir":    DAY03_DIR / "drag_take1",
    },
}

OUTPUT_HZ = 500
OUTPUT_DT = 1.0 / OUTPUT_HZ


# ──────────────────────────────────────────────────────────────────────
# Decode
# ──────────────────────────────────────────────────────────────────────

def decode_drone_logs(drone_files):
    dfs = {}
    for drone_id, path in drone_files.items():
        log_data = cfusdlog.decode(str(path))
        data = log_data["fixedFrequency"]
        df = pd.DataFrame(data)
        df["time_s"] = (df["timestamp"] - df["timestamp"].iloc[0]) / 1000.0
        ts = df["timestamp"].values
        rate = 1000.0 / float(np.median(np.diff(ts)))
        print(f"  Drone {drone_id}: {len(df)} rows @ {rate:.0f} Hz  ← {path.name}")
        dfs[drone_id] = df
    return dfs


# ──────────────────────────────────────────────────────────────────────
# Parse mocap  (same logic as process_day02_data.parse_mocap, but takes
# the file path as an argument instead of using a module global)
# ──────────────────────────────────────────────────────────────────────

def parse_mocap(mocap_file):
    rows = []
    with open(mocap_file, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            rows.append(row)

    meta = rows[0]
    frame_rate = float(meta[meta.index("Export Frame Rate") + 1])
    print(f"  Mocap frame rate: {frame_rate} Hz")

    names = rows[3]
    drones = {}
    col = 2
    while col < len(names):
        name = names[col]
        drones[name] = {"rot_cols": list(range(col, col + 4)),
                        "pos_cols": list(range(col + 4, col + 7))}
        col += 7

    data_rows = rows[7:]
    n = len(data_rows)
    time = np.zeros(n)
    mocap = {}
    for dname in drones:
        mocap[dname] = {
            "time": np.zeros(n),
            "x": np.zeros(n), "y": np.zeros(n), "z": np.zeros(n),
            "qx": np.zeros(n), "qy": np.zeros(n),
            "qz": np.zeros(n), "qw": np.zeros(n),
        }

    for i, row in enumerate(data_rows):
        if len(row) < 2 or row[1] == "":
            continue
        time[i] = float(row[1])
        for dname, cols in drones.items():
            try:
                ot_qx = float(row[cols["rot_cols"][0]])
                ot_qy = float(row[cols["rot_cols"][1]])
                ot_qz = float(row[cols["rot_cols"][2]])
                ot_qw = float(row[cols["rot_cols"][3]])
                ot_px = float(row[cols["pos_cols"][0]])
                ot_py = float(row[cols["pos_cols"][1]])
                ot_pz = float(row[cols["pos_cols"][2]])
            except (ValueError, IndexError):
                for key in ["x", "y", "z", "qx", "qy", "qz", "qw"]:
                    mocap[dname][key][i] = np.nan
                continue

            mocap[dname]["time"][i] = time[i]
            mocap[dname]["x"][i] = ot_pz
            mocap[dname]["y"][i] = ot_px
            mocap[dname]["z"][i] = ot_py
            mocap[dname]["qx"][i] = ot_qz
            mocap[dname]["qy"][i] = ot_qx
            mocap[dname]["qz"][i] = ot_qy
            mocap[dname]["qw"][i] = ot_qw

    for dname in mocap:
        m = mocap[dname]
        valid = ~np.isnan(m["x"])
        if not valid.all():
            n_bad = (~valid).sum()
            print(f"  {dname}: interpolating {n_bad} dropout frames")
            for key in ["x", "y", "z", "qx", "qy", "qz", "qw"]:
                m[key] = np.interp(time, time[valid], m[key][valid])

    for dname in mocap:
        m = mocap[dname]
        quats = np.column_stack([m["qx"], m["qy"], m["qz"], m["qw"]])
        norms = np.linalg.norm(quats, axis=1, keepdims=True)
        quats = quats / np.clip(norms, 1e-12, None)
        rot = Rotation.from_quat(quats)
        euler = rot.as_euler("ZYX", degrees=False)
        m["yaw"] = euler[:, 0]
        m["pitch"] = euler[:, 1]
        m["roll"] = euler[:, 2]

    sorted_names = sorted(drones.keys())
    mocap_by_id = {}
    for i, name in enumerate(sorted_names):
        mocap_by_id[i] = mocap[name]
        mocap_by_id[i]["name"] = name
        z = mocap[name]["z"]
        z_range = np.nanmax(z) - np.nanmin(z)
        status = "FLYING" if z_range > 0.3 else "STATIC"
        print(f"  Mocap drone {i}: {name} [{status}], z=[{z.min():.2f}, {z.max():.2f}]")

    return mocap_by_id


# ──────────────────────────────────────────────────────────────────────
# Build synced CSV at target Hz
# ──────────────────────────────────────────────────────────────────────

def build_synced_csv(dfs, mocap, offsets, out_dir, dt=OUTPUT_DT):
    mapping = offsets["_mapping"]

    # Common time range: intersection of all drone spans in mocap time
    t_starts = []
    t_ends = []
    for did, df in dfs.items():
        ts = df["timestamp"].values / 1000.0
        t_starts.append(drone_to_mocap_time(ts[0],  offsets, did))
        t_ends.append(drone_to_mocap_time(ts[-1], offsets, did))
    t_start = max(t_starts)
    t_end   = min(t_ends)

    mc_t_start = min(m["time"][0]  for m in mocap.values())
    mc_t_end   = max(m["time"][-1] for m in mocap.values())
    t_start = max(t_start, mc_t_start)
    t_end   = min(t_end,   mc_t_end)

    t_common = np.arange(t_start, t_end, dt)
    out = {"time": t_common}

    # Drone data
    for did, df in dfs.items():
        ts      = df["timestamp"].values / 1000.0
        ts_mc   = drone_to_mocap_time(ts, offsets, did)
        data_cols = [c for c in df.columns if c not in ("timestamp", "time_s")]
        for col in data_cols:
            short   = (col.replace("ranging.", "")
                          .replace("kalman.", "kalman_")
                          .replace("stateEstimateZ.", ""))
            out[f"d{did}_{short}"] = np.interp(t_common, ts_mc, df[col].values)

    # Mocap data + body-frame velocities
    for did in sorted(mocap.keys()):
        m  = mocap[did]
        mt = m["time"]

        out[f"mc{did}_cf_x"]   = np.interp(t_common, mt, m["x"])
        out[f"mc{did}_cf_y"]   = np.interp(t_common, mt, m["y"])
        out[f"mc{did}_cf_z"]   = np.interp(t_common, mt, m["z"])
        out[f"mc{did}_roll"]   = np.interp(t_common, mt, m["roll"])
        out[f"mc{did}_pitch"]  = np.interp(t_common, mt, m["pitch"])
        out[f"mc{did}_yaw"]    = np.interp(t_common, mt, m["yaw"])

        vx_b, vy_b, vz_b = mocap_body_frame_velocity(m)
        out[f"mc{did}_vx_body"] = np.interp(t_common, mt, vx_b)
        out[f"mc{did}_vy_body"] = np.interp(t_common, mt, vy_b)
        out[f"mc{did}_vz_body"] = np.interp(t_common, mt, vz_b)

        yr = mocap_yaw_rate(m)
        out[f"mc{did}_yawrate"] = np.interp(t_common, mt, yr)

    # Ground-truth relative states
    for i, j in [(0, 1), (0, 2), (1, 2)]:
        if i not in mapping or j not in mapping:
            continue
        mi = mocap[mapping[i]]
        mj = mocap[mapping[j]]
        mt = mi["time"]

        dx = mj["x"] - mi["x"]
        dy = mj["y"] - mi["y"]
        dz = mj["z"] - mi["z"]
        d  = np.sqrt(dx**2 + dy**2 + dz**2)
        out[f"mc_dist_{i}{j}"] = np.interp(t_common, mt, d)

        yaw_i = mi["yaw"]
        c, s = np.cos(yaw_i), np.sin(yaw_i)
        out[f"mc_rel_{i}{j}_x"]   = np.interp(t_common, mt,  c*dx + s*dy)
        out[f"mc_rel_{i}{j}_y"]   = np.interp(t_common, mt, -s*dx + c*dy)
        out[f"mc_rel_{i}{j}_z"]   = np.interp(t_common, mt, dz)
        rel_yaw = np.unwrap(mj["yaw"]) - np.unwrap(mi["yaw"])
        out[f"mc_rel_{i}{j}_yaw"] = np.interp(t_common, mt, rel_yaw)

    synced_df = pd.DataFrame(out)
    csv_path  = out_dir / "synced_all_500hz.csv"
    synced_df.to_csv(csv_path, index=False)
    print(f"  → {OUTPUT_HZ} Hz CSV: {len(synced_df)} rows, "
          f"{len(synced_df.columns)} cols  →  {csv_path}")
    return synced_df


# ──────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────

def process_take(name, cfg):
    print(f"\n{'=' * 60}")
    print(f"TAKE: {name}")
    print(f"{'=' * 60}")

    print("\n[1] Decoding drone logs")
    dfs = decode_drone_logs(cfg["drone_files"])

    print("\n[2] Parsing mocap")
    mocap = parse_mocap(cfg["mocap_file"])

    print("\n[3] Synchronizing")
    offsets = sync_all(dfs, mocap)

    print("\n[4] Building synced CSV")
    build_synced_csv(dfs, mocap, offsets, cfg["out_dir"])

    plots_dir = Path("relative_data_plots") / "day03" / name
    plots_dir.mkdir(parents=True, exist_ok=True)

    file_prefix = f"day03_{name}"
    print("\n[5] Plotting velocity debug")
    plot_velocity_alignment_debug(dfs, mocap, offsets, plots_dir,
                                  title_prefix=f"Day 03 {name}",
                                  file_prefix=file_prefix)

    print("\n[6] Plotting yaw rate comparison")
    plot_yawrate_comparison(dfs, mocap, offsets, plots_dir,
                            title_prefix=f"Day 03 {name}",
                            file_prefix=file_prefix)


def main():
    for name, cfg in TAKES.items():
        process_take(name, cfg)
    print("\nAll takes done.")


if __name__ == "__main__":
    main()
