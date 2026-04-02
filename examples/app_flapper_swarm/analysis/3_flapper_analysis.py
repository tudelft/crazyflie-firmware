#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
3_flapper_analysis.py

Loads and plots data from three Crazyflie uSD log files (flap0log00, flap1log00, flap2log00)
with variables: ranging.distance0, ranging.distance1, ranging.distance2, acc.x, acc.y, acc.z,
gyro.x, gyro.y, gyro.z, range.zrange, mtf02.dpixelX, mtf02.dpixelY, mtf02.flowQual, etc.

Usage:
    python3 3_flapper_analysis.py

Edit the DATA_FILES list to point to your log files.
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

HERE = Path(__file__).resolve().parent
USDLOG_DIR = HERE.parents[2] / "tools" / "usdlog"
sys.path.insert(0, str(USDLOG_DIR))

import cfusdlog

# Edit these paths as needed
DATA_FILES = [
    "/home/austin/DATA/flapper_data/min_data/flap0log08",
    "/home/austin/DATA/flapper_data/min_data/flap1log08",
    "/home/austin/DATA/flapper_data/min_data/flap2log08",
]

LABELS = ["Drone 0", "Drone 1", "Drone 2"]


# Expanded variables to plot (including new states)
PLOT_VARS = [
    ["ranging.distance0", "ranging.distance1", "ranging.distance2"],
    ["acc.x", "acc.y", "acc.z"],
    ["gyro.x", "gyro.y", "gyro.z"],
    ["range.zrange"],
    ["mtf02.dpixelX", "mtf02.dpixelY"],
    ["mtf02.flowQual"],
    ["kalman.statePX", "kalman.statePY", "kalman.statePZ"],
    ["stateEstimateZ.rateYaw"],
    ["stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz"],
]

PLOT_TITLES = [
    "Direct Ranges [mm]",
    "Accelerometer [m/s^2]",
    "Gyroscope [rad/s]",
    "Z Range [mm]",
    "MTF02 Pixel Displacement",
    "MTF02 Flow Quality",
    "Kalman State Position (X/Y/Z)",
    "State Estimate Yaw Rate",
    "State Estimate Velocity (vx/vy/vz)",
]



def load_df(logfile):
    log_all = cfusdlog.decode(logfile)
    if "fixedFrequency" not in log_all:
        raise KeyError(f"'fixedFrequency' not found in {logfile}. Available: {list(log_all.keys())}")
    df = pd.DataFrame(log_all["fixedFrequency"])
    if "timestamp" in df.columns:
        df["t_s"] = (df["timestamp"] - df["timestamp"].iloc[0]) / 1000.0
    return df

def cross_correlate_shift(ref, target):
    # Remove NaNs for correlation
    ref = np.nan_to_num(ref)
    target = np.nan_to_num(target)
    corr = np.correlate(target - np.mean(target), ref - np.mean(ref), mode='full')
    shift = corr.argmax() - (len(ref) - 1)
    return shift

def align_dataframes_interdrone(dfs):
    # Align using inter-drone distances:
    # drone0.ranging.distance1 <-> drone1.ranging.distance2
    # drone0.ranging.distance2 <-> drone2.ranging.distance1
    # We'll use drone0 as reference, align drone1 and drone2 to it
    ref = dfs[0]
    aligned = [ref]
    shifts = [0]
    # Align drone1
    if "ranging.distance1" in ref.columns and "ranging.distance2" in dfs[1].columns:
        ref_signal = ref["ranging.distance1"].values
        target_signal = dfs[1]["ranging.distance2"].values
        shift1 = cross_correlate_shift(ref_signal, target_signal)
    else:
        shift1 = 0
    shifts.append(shift1)
    if shift1 > 0:
        df1_aligned = dfs[1].iloc[shift1:].reset_index(drop=True)
    elif shift1 < 0:
        pad = pd.DataFrame({col: [np.nan]*(-shift1) for col in dfs[1].columns})
        df1_aligned = pd.concat([pad, dfs[1]], ignore_index=True)
    else:
        df1_aligned = dfs[1].copy()
    aligned.append(df1_aligned)
    # Align drone2
    if "ranging.distance2" in ref.columns and "ranging.distance1" in dfs[2].columns:
        ref_signal = ref["ranging.distance2"].values
        target_signal = dfs[2]["ranging.distance1"].values
        shift2 = cross_correlate_shift(ref_signal, target_signal)
    else:
        shift2 = 0
    shifts.append(shift2)
    if shift2 > 0:
        df2_aligned = dfs[2].iloc[shift2:].reset_index(drop=True)
    elif shift2 < 0:
        pad = pd.DataFrame({col: [np.nan]*(-shift2) for col in dfs[2].columns})
        df2_aligned = pd.concat([pad, dfs[2]], ignore_index=True)
    else:
        df2_aligned = dfs[2].copy()
    aligned.append(df2_aligned)
    return aligned, shifts

def save_csv(df, src_path):
    csv_path = Path(src_path).with_suffix('.csv')
    df.to_csv(csv_path, index=False)
    print(f"Saved CSV: {csv_path}")


def plot_var_group(dfs, var_list, title, ylabel):
    plt.figure(figsize=(12, 6))
    for df, label in zip(dfs, LABELS):
        for var in var_list:
            if var in df.columns:
                plt.plot(df["t_s"] if "t_s" in df.columns else df.index, df[var], label=f"{label}: {var}")
    plt.title(title)
    plt.xlabel("Time [s]")
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()



def main():
    dfs = [load_df(f) for f in DATA_FILES]
    # Save original CSVs
    for df, f in zip(dfs, DATA_FILES):
        save_csv(df, f)

    # Align dataframes using inter-drone distance pairs
    aligned_dfs, shifts = align_dataframes_interdrone(dfs)
    print(f"Alignment shifts (samples): {shifts}")

    # Save aligned CSVs
    for i, (df, f) in enumerate(zip(aligned_dfs, DATA_FILES)):
        save_csv(df, Path(f).with_name(Path(f).stem + "_aligned.csv"))


    # Create unified dataframe (outer join on index, add drone id prefix)
    maxlen = max(len(df) for df in aligned_dfs)
    dfs_expanded = []
    for i, df in enumerate(aligned_dfs):
        df_exp = df.copy()
        df_exp = df_exp.reindex(range(maxlen))
        df_exp = df_exp.add_prefix(f"d{i}_")
        dfs_expanded.append(df_exp)
    unified = pd.concat(dfs_expanded, axis=1)
    unified.to_csv(HERE / "unified_flapper_data.csv", index=False)
    print(f"Saved unified CSV: {HERE / 'unified_flapper_data.csv'}")

    # Create unified_short: only rows where all drones have non-NaN for at least one key column (e.g., time or distance)
    # We'll use the first column of each drone (should be t_s or timestamp or similar)
    key_cols = [df.columns[0] for df in dfs_expanded]
    mask = ~dfs_expanded[0][key_cols[0]].isna()
    for i in range(1, len(dfs_expanded)):
        mask &= ~dfs_expanded[i][key_cols[i]].isna()
    unified_short = unified[mask].reset_index(drop=True)
    unified_short.to_csv(HERE / "unified_flapper_data_short.csv", index=False)
    print(f"Saved unified_short CSV: {HERE / 'unified_flapper_data_short.csv'}")

    # Plot using unified_short data
    unified_short = pd.read_csv(HERE / "unified_flapper_data_short.csv")
    # Split unified_short into per-drone dfs (by prefix)
    dfs_short = []
    for i in range(len(LABELS)):
        prefix = f"d{i}_"
        cols = [c for c in unified_short.columns if c.startswith(prefix)]
        df = unified_short[cols].copy()
        df.columns = [c[len(prefix):] for c in cols]
        dfs_short.append(df)

    for var_list, title in zip(PLOT_VARS, PLOT_TITLES):
        plot_var_group(dfs_short, var_list, title, title)

    plt.show()


if __name__ == "__main__":
    main()
