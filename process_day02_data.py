#!/usr/bin/env python3
"""
Process day_02 Flapper drone relative localization flight data.

Day 02 specifics:
- Drone 0 is STATIONARY (ground beacon, not flying)
- Drones 1 and 2 are flying
- Drone 0's Kalman estimates drift (no absolute position correction on ground)
  but its communicated velocities to other drones are ~0

Pipeline:
1. Decode binary SD logs → raw CSVs
2. Create zeroed-drone0 CSVs (drone 0 rates forced to 0)
3. Parse OptiTrack mocap ground truth → Crazyflie frame
4. Synchronize drone/mocap time using height cross-correlation
5. Compute mocap body-frame velocities (full rotation, not yaw-only)
6. Merge into synced CSV
7. Plot height alignment and body-frame velocity comparisons
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
from pynumdiff import smooth_finite_difference as pnd_sfd
from scipy.signal import correlate, savgol_filter
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent / "tools" / "usdlog"))
import cfusdlog

# ──────────────────────────────────────────────────────────────────────
# Paths
# ──────────────────────────────────────────────────────────────────────

DATA_DIR   = Path("relative_data/day_02")
OUTPUT_DIR = Path("relative_data_plots/day02")

DRONE_FILES = {
    0: DATA_DIR / "flap0log00",
    1: DATA_DIR / "flap1log00",
    2: DATA_DIR / "flap2log00",
}
MOCAP_FILE = DATA_DIR / "REL_DATA_GODD_SD.csv"

# Drone 0 is stationary — its own Kalman velocity/height estimates are unreliable
STATIC_DRONE = 0


# ──────────────────────────────────────────────────────────────────────
# 1. Decode binary logs → raw CSVs
# ──────────────────────────────────────────────────────────────────────

def decode_drone_logs():
    """Decode binary SD logs and save as raw CSVs. Returns dict of DataFrames."""
    dfs = {}
    for drone_id, path in DRONE_FILES.items():
        csv_path = path.with_name(path.name + "_raw.csv")
        log_data = cfusdlog.decode(str(path))
        data = log_data["fixedFrequency"]
        df = pd.DataFrame(data)
        df["time_s"] = (df["timestamp"] - df["timestamp"].iloc[0]) / 1000.0
        df.to_csv(csv_path, index=False)
        print(f"  Drone {drone_id}: {len(df)} rows, {len(df.columns)} cols → {csv_path}")
        dfs[drone_id] = df
    return dfs


# ──────────────────────────────────────────────────────────────────────
# 2. Create zeroed-drone0 CSVs
# ──────────────────────────────────────────────────────────────────────

def create_zeroed_csvs(dfs):
    """Zero out drone 0's velocity/rate columns in all logs."""
    zeroed = {}
    for drone_id, df in dfs.items():
        zdf = df.copy()

        if drone_id == STATIC_DRONE:
            # Zero drone 0's own Kalman velocity estimates and yaw rate
            for col in ["kalman.statePX", "kalman.statePY", "kalman.statePZ",
                        "stateEstimateZ.rateYaw"]:
                if col in zdf.columns:
                    zdf[col] = 0.0
        else:
            # Zero the communicated drone 0 data in other drones' logs
            for col in [f"ranging.vx{STATIC_DRONE}", f"ranging.vy{STATIC_DRONE}",
                        f"ranging.vz{STATIC_DRONE}", f"ranging.yawR{STATIC_DRONE}"]:
                if col in zdf.columns:
                    zdf[col] = 0.0

        csv_path = DRONE_FILES[drone_id].with_name(
            DRONE_FILES[drone_id].name + "_zeroed.csv")
        zdf.to_csv(csv_path, index=False)
        print(f"  Drone {drone_id}: zeroed → {csv_path}")
        zeroed[drone_id] = zdf

    return zeroed


# ──────────────────────────────────────────────────────────────────────
# 3. Parse OptiTrack mocap CSV → Crazyflie frame
# ──────────────────────────────────────────────────────────────────────

def parse_mocap():
    """Parse OptiTrack CSV and convert to Crazyflie XYZ FLU frame.

    OptiTrack global frame: Z-forward, X-left, Y-up  (ZXY)
    Crazyflie world frame:  X-forward, Y-left, Z-up  (XYZ FLU)

    Mapping:
        cf_x = ot_z,  cf_y = ot_x,  cf_z = ot_y
        cf_qx = ot_qz, cf_qy = ot_qx, cf_qz = ot_qy, cf_qw = ot_qw
    """
    rows = []
    with open(MOCAP_FILE, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            rows.append(row)

    # Row 0: metadata, Row 5: Rotation/Position, Row 6: X/Y/Z/W, Row 7+: data
    meta = rows[0]
    frame_rate = float(meta[meta.index("Export Frame Rate") + 1])
    print(f"  Mocap frame rate: {frame_rate} Hz")

    names = rows[3]       # drone names per column group

    # Per-drone columns: 4 rotation + 3 position = 7 each
    drones = {}
    col = 2  # skip Frame, Time
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
            mocap[dname]["x"][i] = ot_pz    # cf_x = ot_z
            mocap[dname]["y"][i] = ot_px    # cf_y = ot_x
            mocap[dname]["z"][i] = ot_py    # cf_z = ot_y (height)
            mocap[dname]["qx"][i] = ot_qz   # cf_qx = ot_qz
            mocap[dname]["qy"][i] = ot_qx   # cf_qy = ot_qx
            mocap[dname]["qz"][i] = ot_qy   # cf_qz = ot_qy
            mocap[dname]["qw"][i] = ot_qw   # cf_qw = ot_qw

    # Interpolate NaN gaps from tracking dropouts, but track which regions were interpolated
    for dname in mocap:
        m = mocap[dname]
        valid = ~np.isnan(m["x"])
        if not valid.all():
            n_bad = (~valid).sum()
            print(f"  {dname}: interpolating {n_bad} dropout frames")
            for key in ["x", "y", "z", "qx", "qy", "qz", "qw"]:
                m[key] = np.interp(time, time[valid], m[key][valid])
            # Track interpolated regions (where original data was NaN)
            m["valid_orig"] = valid

    # Fix time discontinuities (jumps backward in time) by interpolating time linearly
    # This happens when mocap software restarts between test runs
    for dname in mocap:
        m = mocap[dname]
        t = m["time"]
        dt = np.diff(t)
        # Check for backward jumps (discontinuities)
        bad_jumps = dt < -0.01  # threshold: backward jump > 10ms
        if bad_jumps.any():
            print(f"  {dname}: detected {bad_jumps.sum()} time discontinuities, fixing...")
            # Create evenly-spaced time by using frame indices
            # Expected timebase: 1.0 / 180.0 Hz
            expected_dt = 1.0 / frame_rate
            t_linear = np.arange(len(t)) * expected_dt
            # Preserve the original time offset at the start
            t_linear = t_linear + t[0]
            print(f"    Original time: [{t[0]:.2f}, {t[-1]:.2f}]")
            print(f"    Fixed time:    [{t_linear[0]:.2f}, {t_linear[-1]:.2f}]")
            m["time"] = t_linear
            m["time_original"] = t  # keep original for debugging
        else:
            m["time"] = t

    # Compute roll, pitch, yaw from quaternions
    for dname in mocap:
        m = mocap[dname]
        quats = np.column_stack([m["qx"], m["qy"], m["qz"], m["qw"]])
        norms = np.linalg.norm(quats, axis=1, keepdims=True)
        quats = quats / np.clip(norms, 1e-12, None)
        rot = Rotation.from_quat(quats)  # scipy uses [x,y,z,w]
        euler = rot.as_euler("ZYX", degrees=False)  # [yaw, pitch, roll]
        m["yaw"] = euler[:, 0]
        m["pitch"] = euler[:, 1]
        m["roll"] = euler[:, 2]

    # Map drone names to IDs (sorted alphabetically)
    sorted_names = sorted(drones.keys())
    mocap_by_id = {}
    for i, name in enumerate(sorted_names):
        mocap_by_id[i] = mocap[name]
        mocap_by_id[i]["name"] = name
        z = mocap[name]["z"]
        z_range = np.nanmax(z) - np.nanmin(z)
        status = "FLYING" if z_range > 0.3 else "STATIC"
        print(f"  Mocap drone {i}: {name} [{status}], {n} frames, "
              f"z=[{z.min():.2f}, {z.max():.2f}]")

    return mocap_by_id


# ──────────────────────────────────────────────────────────────────────
# 4. Synchronize drone time ↔ mocap time
# ──────────────────────────────────────────────────────────────────────

def sync_time_offset(signal_a_time, signal_a, signal_b_time, signal_b):
    """Find lag such that signal_a(t) ~ signal_b(t - lag).

    Returns lag in seconds. Positive lag means signal_a is delayed
    relative to signal_b.
    """
    dt = 0.01  # 100 Hz common grid
    t_max = max(signal_a_time[-1] - signal_a_time[0],
                signal_b_time[-1] - signal_b_time[0])
    t_grid = np.arange(0, t_max, dt)

    a_rel = signal_a_time - signal_a_time[0]
    b_rel = signal_b_time - signal_b_time[0]
    ha = np.interp(t_grid, a_rel, signal_a, left=0, right=0)
    hb = np.interp(t_grid, b_rel, signal_b, left=0, right=0)

    ha = (ha - ha.mean()) / (ha.std() + 1e-12)
    hb = (hb - hb.mean()) / (hb.std() + 1e-12)

    corr = correlate(ha, hb, mode="full")
    lags = np.arange(-len(hb) + 1, len(ha)) * dt
    best = np.argmax(corr)
    lag = lags[best]

    print(f"    Cross-corr lag: {lag:.3f}s (peak corr: {corr[best]:.1f})")
    return lag


def _enforce_monotonic_time(t, data):
    """Keep only samples where time is strictly increasing.

    Parameters
    ----------
    t    : 1-D array of timestamps
    data : dict of 1-D arrays (same length as t)

    Returns
    -------
    t_mono    : monotonically increasing time array
    data_mono : dict with same keys, filtered to matching indices
    """
    keep = np.concatenate(([True], np.diff(t) > 0))
    t_mono = t[keep]
    data_mono = {k: v[keep] for k, v in data.items()}
    return t_mono, data_mono


def _find_takeoff_time(signal, time, threshold=0.5):
    """Find the first time a signal crosses a threshold (rising edge)."""
    above = np.where(signal > threshold)[0]
    if len(above) == 0:
        return None
    return time[above[0]]


def _find_landing_time(signal, time, threshold=0.5):
    """Find the last time a signal drops below a threshold (falling edge)."""
    above = np.where(signal > threshold)[0]
    if len(above) == 0:
        return None
    return time[above[-1]]


def _xcorr_offset_in_window(drone_time, drone_sig, mocap_time, mocap_sig,
                             t_center_drone, rough_offset, window=8.0,
                             search_margin=3.0):
    """Cross-correlate drone and mocap signals in a time window.

    Returns the offset such that t_mocap ≈ t_drone + offset.
    Uses rough_offset to constrain the search range and avoid locking
    onto the wrong oscillation cycle.
    """
    dt = 0.005  # 200 Hz grid

    # Extract drone window
    d_mask = ((drone_time >= t_center_drone - window/2) &
              (drone_time <= t_center_drone + window/2))
    if d_mask.sum() < 20:
        return None
    d_t = drone_time[d_mask]
    d_s = drone_sig[d_mask]

    # Expected mocap center
    mc_center = t_center_drone + rough_offset

    # Extract mocap window with extra margin for search
    m_half = window/2 + search_margin
    m_mask = ((mocap_time >= mc_center - m_half) &
              (mocap_time <= mc_center + m_half))
    if m_mask.sum() < 20:
        return None
    m_t = mocap_time[m_mask]
    m_s = mocap_sig[m_mask]

    # Resample both onto regular grids
    t_grid_d = np.arange(0, d_t[-1] - d_t[0], dt)
    d_grid = np.interp(t_grid_d, d_t - d_t[0], d_s)
    d_grid = (d_grid - d_grid.mean()) / (d_grid.std() + 1e-12)

    t_grid_m = np.arange(0, m_t[-1] - m_t[0], dt)
    m_grid = np.interp(t_grid_m, m_t - m_t[0], m_s)
    m_grid = (m_grid - m_grid.mean()) / (m_grid.std() + 1e-12)

    corr = correlate(m_grid, d_grid, mode="full")
    lags_samples = np.arange(-len(d_grid) + 1, len(m_grid))
    lags_sec = lags_samples * dt

    best = np.argmax(corr)
    # d_grid[0] aligns with m_grid[lags_samples[best]]
    # In absolute time: d_t[0] matches m_t[0] + lags_sec[best]
    offset = (m_t[0] + lags_sec[best]) - d_t[0]
    return offset


def sync_all(dfs, mocap):
    """Synchronize drone logs with mocap using affine time mapping.

    Each drone has its own boot clock that may drift relative to the
    mocap PC clock. We compute per-drone affine parameters:
        t_mocap = scale * t_drone + offset
    using cross-correlation at two time points (early and late flight).
    """
    flying_ids = [did for did in dfs if did != STATIC_DRONE]

    # Mapping: drone_id → mocap_id (sorted alphabetically matches drone numbering)
    sorted_mocap_ids = sorted(mocap.keys())
    mapping = {did: sorted_mocap_ids[did] for did in dfs}
    for did in sorted(mapping.keys()):
        mid = mapping[did]
        status = "static" if did == STATIC_DRONE else "flying"
        print(f"  Drone {did} = mocap {mid} ({mocap[mid]['name']}, {status})")

    # Per-drone affine mapping: t_mocap = scale * t_drone + offset
    per_drone_affine = {}  # drone_id → (scale, offset)

    for did in flying_ids:
        df = dfs[did]
        ts = df["timestamp"].values / 1000.0
        h = df["kalman.stateZ"].values
        mid = mapping[did]
        m = mocap[mid]

        # Find flight window
        t_takeoff_d = _find_takeoff_time(h, ts, threshold=0.5)
        t_landing_d = _find_landing_time(h, ts, threshold=0.5)
        t_takeoff_mc = _find_takeoff_time(m["z"], m["time"], threshold=0.5)
        if t_takeoff_d is None or t_landing_d is None or t_takeoff_mc is None:
            continue
        flight_dur = t_landing_d - t_takeoff_d
        rough_offset = t_takeoff_mc - t_takeoff_d

        # Cross-correlate in multiple windows across the flight and fit a line
        n_windows = 6
        t_centers = np.linspace(t_takeoff_d + 5.0, t_landing_d - 5.0, n_windows)
        offsets_measured = []
        t_measured = []
        for tc in t_centers:
            off = _xcorr_offset_in_window(ts, h, m["time"], m["z"],
                                          tc, rough_offset, window=8.0,
                                          search_margin=1.5)
            if off is not None:
                offsets_measured.append(off)
                t_measured.append(tc)

        if len(offsets_measured) >= 2:
            t_arr = np.array(t_measured)
            off_arr = np.array(offsets_measured)
            # t_mocap = t_drone + offset, where offset varies linearly
            # offset(t) = a * t + b → t_mocap = (1+a)*t + b
            coeffs = np.polyfit(t_arr, off_arr, 1)
            scale = 1.0 + coeffs[0]
            offset = coeffs[1]
            drift_ppm = coeffs[0] * 1e6
            residuals = off_arr - np.polyval(coeffs, t_arr)
            print(f"  Drone {did}: scale={scale:.8f} ({drift_ppm:+.0f} ppm), "
                  f"offset={offset:.3f}s, residual_std={residuals.std()*1000:.1f}ms "
                  f"({len(offsets_measured)} windows)")
            per_drone_affine[did] = (scale, offset)
        elif len(offsets_measured) == 1:
            per_drone_affine[did] = (1.0, offsets_measured[0])
            print(f"  Drone {did}: offset-only={offsets_measured[0]:.3f}s")

    # Static drone 0: use communicated height of a flying drone
    # to find the affine mapping for drone 0's clock
    d0 = dfs[STATIC_DRONE]
    ts0 = d0["timestamp"].values / 1000.0
    ref_flying = flying_ids[0]
    ref_mid = mapping[ref_flying]
    col = f"ranging.height{ref_flying}"
    if col in d0.columns:
        h_comm = d0[col].values
        m = mocap[ref_mid]

        t_takeoff_d0 = _find_takeoff_time(h_comm, ts0, threshold=0.5)
        t_landing_d0 = _find_landing_time(h_comm, ts0, threshold=0.5)
        t_takeoff_mc = _find_takeoff_time(m["z"], m["time"], threshold=0.5)
        if t_takeoff_d0 is not None and t_landing_d0 is not None and t_takeoff_mc is not None:
            flight_dur_d0 = t_landing_d0 - t_takeoff_d0
            rough_offset_d0 = t_takeoff_mc - t_takeoff_d0

            n_windows = 6
            t_centers = np.linspace(t_takeoff_d0 + 5.0, t_landing_d0 - 5.0, n_windows)
            offsets_measured = []
            t_measured = []
            for tc in t_centers:
                off = _xcorr_offset_in_window(ts0, h_comm, m["time"], m["z"],
                                              tc, rough_offset_d0, window=8.0,
                                              search_margin=1.5)
                if off is not None:
                    offsets_measured.append(off)
                    t_measured.append(tc)

            if len(offsets_measured) >= 2:
                t_arr = np.array(t_measured)
                off_arr = np.array(offsets_measured)
                coeffs = np.polyfit(t_arr, off_arr, 1)
                scale = 1.0 + coeffs[0]
                offset = coeffs[1]
                drift_ppm = coeffs[0] * 1e6
                residuals = off_arr - np.polyval(coeffs, t_arr)
                print(f"  Drone {STATIC_DRONE}: scale={scale:.8f} ({drift_ppm:+.0f} ppm), "
                      f"offset={offset:.3f}s, residual_std={residuals.std()*1000:.1f}ms "
                      f"({len(offsets_measured)} windows)")
                per_drone_affine[STATIC_DRONE] = (scale, offset)
            elif len(offsets_measured) == 1:
                per_drone_affine[STATIC_DRONE] = (1.0, offsets_measured[0])

    # Outlier rejection: all drones share the same crystal hardware, so their
    # clock drift rates (scale) should agree within ~2000 ppm.  If one drone's
    # fitted scale is a clear outlier (e.g. due to mocap dropouts corrupting
    # the height signal), pin it to the consensus scale and refit only the
    # constant offset from the measured offset samples.
    if len(per_drone_affine) >= 2:
        scales = np.array([s for s, _ in per_drone_affine.values()])
        median_scale = float(np.median(scales))
        SCALE_OUTLIER_PPM = 5000  # tolerance: 5000 ppm ≈ 0.5%
        for did in list(per_drone_affine.keys()):
            s, b = per_drone_affine[did]
            deviation_ppm = abs(s - median_scale) * 1e6
            if deviation_ppm > SCALE_OUTLIER_PPM:
                # Refit offset with scale fixed to consensus
                # Model: t_mocap = median_scale * t_drone + const_offset
                # off_measured(t) = t_mocap - t_drone = (median_scale-1)*t + const_offset
                # So const_offset = mean(off_arr - (median_scale-1)*t_arr)
                # We need off_arr and t_arr for this drone — recompute from the
                # same windows used above, stored as an attribute isn't available
                # so we re-run the window loop with scale fixed.
                df = dfs[did]
                ts = df["timestamp"].values / 1000.0
                if did == STATIC_DRONE:
                    ref_did = flying_ids[0]
                    h = dfs[STATIC_DRONE][f"ranging.height{ref_did}"].values
                    mc_mid = mapping[ref_did]
                else:
                    h = df["kalman.stateZ"].values
                    mc_mid = mapping[did]
                m = mocap[mc_mid]
                t_takeoff_d = _find_takeoff_time(h, ts, threshold=0.5)
                t_landing_d = _find_landing_time(h, ts, threshold=0.5)
                t_takeoff_mc = _find_takeoff_time(m["z"], m["time"], threshold=0.5)
                if t_takeoff_d is None or t_landing_d is None or t_takeoff_mc is None:
                    per_drone_affine[did] = (median_scale, b)
                    print(f"  Drone {did}: scale outlier ({deviation_ppm:.0f} ppm) "
                          f"→ pinned scale to {median_scale:.8f}, kept offset={b:.3f}s")
                    continue
                rough_offset = t_takeoff_mc - t_takeoff_d
                t_centers = np.linspace(t_takeoff_d + 5.0, t_landing_d - 5.0, 6)
                off_list, t_list = [], []
                for tc in t_centers:
                    off = _xcorr_offset_in_window(ts, h, m["time"], m["z"],
                                                  tc, rough_offset,
                                                  window=8.0, search_margin=1.5)
                    if off is not None:
                        off_list.append(off)
                        t_list.append(tc)
                if off_list:
                    t_arr = np.array(t_list)
                    off_arr = np.array(off_list)
                    const_offset = float(np.mean(off_arr - (median_scale - 1.0) * t_arr))
                    residuals = off_arr - ((median_scale - 1.0) * t_arr + const_offset)
                    print(f"  Drone {did}: scale outlier ({deviation_ppm:.0f} ppm) "
                          f"→ pinned scale={median_scale:.8f}, "
                          f"refitted offset={const_offset:.3f}s, "
                          f"residual_std={residuals.std()*1000:.1f}ms")
                    per_drone_affine[did] = (median_scale, const_offset)
                else:
                    per_drone_affine[did] = (median_scale, b)
                    print(f"  Drone {did}: scale outlier → pinned scale, kept offset={b:.3f}s")

    offsets = {
        "_mapping": mapping,
        "_affine": per_drone_affine,
    }
    return offsets


def drone_to_mocap_time(t_drone_boot_s, offsets, drone_id):
    """Convert drone boot time (seconds) to mocap time using affine mapping."""
    scale, offset = offsets["_affine"][drone_id]
    return scale * t_drone_boot_s + offset


# ──────────────────────────────────────────────────────────────────────
# 5. Compute mocap body-frame velocities (full rotation)
# ──────────────────────────────────────────────────────────────────────

def _differentiate_signal_butter(t, x, cutoff_freq=0.06, filter_order=2):
    """Differentiate a signal using pynumdiff Butterworth smoothing.

    Returns derivative on the original time grid, handling duplicate timestamps
    by operating on unique times and interpolating back.
    """
    unique_idx = np.concatenate(([True], np.diff(t) > 1e-9))
    t_u = t[unique_idx]
    x_u = x[unique_idx]

    if len(t_u) < 5:
        dx_u = np.gradient(x_u, t_u)
        return np.interp(t, t_u, dx_u)

    dt_u = float(np.median(np.diff(t_u)))
    dt_u = max(dt_u, 1e-6)

    try:
        # Preferred API in recent pynumdiff versions.
        _, dx_u = pnd_sfd.butterdiff(
            x_u,
            dt_u,
            filter_order=filter_order,
            cutoff_freq=cutoff_freq,
        )
    except TypeError:
        # Fallback for API variants that require params=[order, cutoff].
        _, dx_u = pnd_sfd.butterdiff(x_u, dt_u, params=[filter_order, cutoff_freq])

    return np.interp(t, t_u, dx_u)

def mocap_body_frame_velocity(mocap_drone):
    """Compute body-frame velocity from mocap using full rotation.

    Differentiates position with a Butterworth smoother (pynumdiff), then
    rotates world-frame velocity into body frame using stored quaternions.
    """
    t = mocap_drone["time"]
    x = mocap_drone["x"]
    y = mocap_drone["y"]
    z = mocap_drone["z"]
    valid_orig = mocap_drone.get("valid_orig", np.ones(len(t), dtype=bool))

    vx_w = _differentiate_signal_butter(t, x)
    vy_w = _differentiate_signal_butter(t, y)
    vz_w = _differentiate_signal_butter(t, z)
    
    # Mask velocity as NaN where data was interpolated (dropout regions)
    # and a margin around those regions to avoid derivative spikes
    if valid_orig is not None:
        interp_regions = ~valid_orig
        if interp_regions.any():
            # Dilate interpolated regions by 1-2 samples to catch edge effects
            dilated = interp_regions.copy()
            dilated[:-1] |= interp_regions[1:]
            dilated[1:] |= interp_regions[:-1]
            vx_w[dilated] = np.nan
            vy_w[dilated] = np.nan
            vz_w[dilated] = np.nan

    # Use quaternions directly (already in CF frame from parse_mocap)
    quats = np.column_stack([
        mocap_drone["qx"], mocap_drone["qy"],
        mocap_drone["qz"], mocap_drone["qw"]
    ])
    norms = np.linalg.norm(quats, axis=1, keepdims=True)
    quats = quats / np.clip(norms, 1e-12, None)
    # scipy Rotation.from_quat expects [x, y, z, w]
    rotations = Rotation.from_quat(quats)

    # R rotates body→world, so R.inv() rotates world→body
    v_world = np.column_stack([vx_w, vy_w, vz_w])
    v_body = rotations.inv().apply(v_world)

    return v_body[:, 0], v_body[:, 1], v_body[:, 2]


def mocap_global_velocity(mocap_drone):
    """Compute world/global-frame velocity from mocap position."""
    t = mocap_drone["time"]
    x = mocap_drone["x"]
    y = mocap_drone["y"]
    z = mocap_drone["z"]
    valid_orig = mocap_drone.get("valid_orig", np.ones(len(t), dtype=bool))

    vx_w = _differentiate_signal_butter(t, x)
    vy_w = _differentiate_signal_butter(t, y)
    vz_w = _differentiate_signal_butter(t, z)
    
    # Mask velocity as NaN where data was interpolated (dropout regions)
    if valid_orig is not None:
        interp_regions = ~valid_orig
        if interp_regions.any():
            # Dilate interpolated regions by 1-2 samples to catch edge effects
            dilated = interp_regions.copy()
            dilated[:-1] |= interp_regions[1:]
            dilated[1:] |= interp_regions[:-1]
            vx_w[dilated] = np.nan
            vy_w[dilated] = np.nan
            vz_w[dilated] = np.nan
    
    return vx_w, vy_w, vz_w


# ──────────────────────────────────────────────────────────────────────
# 6. Merge into synced CSV
# ──────────────────────────────────────────────────────────────────────

def build_synced_csv(dfs, mocap, offsets, dt=0.02, output_name="synced_all.csv"):
    """Resample all data onto a common time grid and merge.

    Args:
        dfs: Drone log DataFrames keyed by drone id.
        mocap: Mocap data keyed by drone id.
        offsets: Synchronization metadata from sync_all().
        dt: Output timestep in seconds.
        output_name: Output CSV filename under DATA_DIR.
    """
    mapping = offsets["_mapping"]

    # Determine common time range in mocap time
    # Use the union of all drone time spans
    t_starts = []
    t_ends = []
    for did, df in dfs.items():
        ts = df["timestamp"].values / 1000.0
        t_starts.append(drone_to_mocap_time(ts[0], offsets, did))
        t_ends.append(drone_to_mocap_time(ts[-1], offsets, did))
    t_start = max(t_starts)
    t_end = min(t_ends)

    # Also bound by mocap time
    mc_t_start = min(m["time"][0] for m in mocap.values())
    mc_t_end = max(m["time"][-1] for m in mocap.values())
    t_start = max(t_start, mc_t_start)
    t_end = min(t_end, mc_t_end)

    t_common = np.arange(t_start, t_end, dt)
    out = {"time": t_common}

    # Drone data: resample each drone's log onto common time grid
    for did, df in dfs.items():
        ts = df["timestamp"].values / 1000.0
        ts_mc = drone_to_mocap_time(ts, offsets, did)

        # All columns except timestamp and time_s
        data_cols = [c for c in df.columns if c not in ("timestamp", "time_s")]
        for col in data_cols:
            # Rename: ranging.distance1 → d0_distance1, kalman.stateZ → d0_stateZ
            short = col.replace("ranging.", "").replace("kalman.", "kalman_") \
                       .replace("stateEstimateZ.", "")
            out_col = f"d{did}_{short}"
            out[out_col] = np.interp(t_common, ts_mc, df[col].values)

    # Mocap data
    for did in sorted(mocap.keys()):
        mid = None
        for d_id, m_id in mapping.items():
            if m_id == did:
                mid = did
                break
        if mid is None:
            mid = did

        m = mocap[did]
        mt = m["time"]

        out[f"mc{did}_cf_x"] = np.interp(t_common, mt, m["x"])
        out[f"mc{did}_cf_y"] = np.interp(t_common, mt, m["y"])
        out[f"mc{did}_cf_z"] = np.interp(t_common, mt, m["z"])
        out[f"mc{did}_roll"] = np.interp(t_common, mt, m["roll"])
        out[f"mc{did}_pitch"] = np.interp(t_common, mt, m["pitch"])
        out[f"mc{did}_yaw"] = np.interp(t_common, mt, m["yaw"])

        vx_b, vy_b, vz_b = mocap_body_frame_velocity(m)
        out[f"mc{did}_vx_body"] = np.interp(t_common, mt, vx_b)
        out[f"mc{did}_vy_body"] = np.interp(t_common, mt, vy_b)
        out[f"mc{did}_vz_body"] = np.interp(t_common, mt, vz_b)

        yr = mocap_yaw_rate(m)
        out[f"mc{did}_yawrate"] = np.interp(t_common, mt, yr)

    # Ground truth relative states
    pairs = [(0, 1), (0, 2), (1, 2)]
    for i, j in pairs:
        if i not in mapping or j not in mapping:
            continue
        mi = mocap[mapping[i]]
        mj = mocap[mapping[j]]
        mt = mi["time"]

        dx = mj["x"] - mi["x"]
        dy = mj["y"] - mi["y"]
        dz = mj["z"] - mi["z"]
        d = np.sqrt(dx**2 + dy**2 + dz**2)

        out[f"mc_dist_{i}{j}"] = np.interp(t_common, mt, d)

        # Body-frame relative position (drone j in drone i's frame)
        yaw_i = mi["yaw"]
        c = np.cos(yaw_i)
        s = np.sin(yaw_i)
        x12 = c * dx + s * dy
        y12 = -s * dx + c * dy
        z12 = dz

        out[f"mc_rel_{i}{j}_x"] = np.interp(t_common, mt, x12)
        out[f"mc_rel_{i}{j}_y"] = np.interp(t_common, mt, y12)
        out[f"mc_rel_{i}{j}_z"] = np.interp(t_common, mt, z12)

        rel_yaw = np.unwrap(mj["yaw"]) - np.unwrap(mi["yaw"])
        out[f"mc_rel_{i}{j}_yaw"] = np.interp(t_common, mt, rel_yaw)

    synced_df = pd.DataFrame(out)
    csv_path = DATA_DIR / output_name
    synced_df.to_csv(csv_path, index=False)
    print(
        f"  Synced CSV ({1.0 / dt:.1f} Hz): {len(synced_df)} rows, "
        f"{len(synced_df.columns)} cols → {csv_path}"
    )
    return synced_df


# ──────────────────────────────────────────────────────────────────────
# 7. Comparison plots
# ──────────────────────────────────────────────────────────────────────

def plot_height_sync(dfs, mocap, offsets, out_dir):
    """Overlay drone heights vs mocap Z to verify synchronization."""
    out_dir.mkdir(parents=True, exist_ok=True)
    mapping = offsets["_mapping"]
    colors_drone = ["tab:blue", "tab:orange", "tab:green"]
    colors_mocap = ["tab:red", "tab:purple", "tab:brown"]

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle("Day 02: Height Sync Verification", fontsize=14, fontweight="bold")

    for idx, did in enumerate(sorted(dfs.keys())):
        ax = axes[idx]
        df = dfs[did]
        ts = df["timestamp"].values / 1000.0
        ts_mc = drone_to_mocap_time(ts, offsets, did)

        # Drone height
        if did == STATIC_DRONE:
            # For static drone, show communicated heights of flying drones
            for other in [1, 2]:
                col = f"ranging.height{other}"
                if col in df.columns:
                    ax.plot(ts_mc, df[col].values, alpha=0.7, lw=1,
                            label=f"drone0 sees height{other}")
        else:
            ax.plot(ts_mc, df["kalman.stateZ"].values,
                    color=colors_drone[did], alpha=0.7, lw=1,
                    label=f"drone {did} kalman.stateZ")

        # Mocap height
        if did in mapping:
            m = mocap[mapping[did]]
            ax.plot(m["time"], m["z"], color=colors_mocap[idx], lw=2,
                    alpha=0.8, label=f"mocap {mapping[did]} ({m['name']})")

        ax.set_ylabel(f"Drone {did}\nHeight [m]")
        ax.legend(fontsize=8, loc="upper right")
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time [s]")
    plt.tight_layout()
    fig.savefig(out_dir / "day02_height_sync.png", dpi=150)
    plt.close(fig)
    print(f"  Saved day02_height_sync.png")


def plot_velocity_comparison(dfs, mocap, offsets, out_dir):
    """Plot drone body-frame velocities vs mocap-derived body-frame velocities."""
    out_dir.mkdir(parents=True, exist_ok=True)
    mapping = offsets["_mapping"]

    for did in sorted(dfs.keys()):
        if did == STATIC_DRONE:
            continue  # skip static drone

        if did not in mapping:
            continue

        df = dfs[did]
        ts = df["timestamp"].values / 1000.0
        ts_mc = drone_to_mocap_time(ts, offsets, did)

        m = mocap[mapping[did]]
        mt = m["time"]
        vx_b, vy_b, vz_b = mocap_body_frame_velocity(m)

        fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
        fig.suptitle(f"Day 02: Drone {did} Body-Frame Velocities",
                     fontsize=14, fontweight="bold")

        labels = [("statePX", "vx"), ("statePY", "vy"), ("statePZ", "vz")]
        mocap_vels = [vx_b, vy_b, vz_b]

        for i, (kalman_col, vel_name) in enumerate(labels):
            ax = axes[i]
            drone_col = f"kalman.{kalman_col}"
            ax.plot(ts_mc, df[drone_col].values, "b-", alpha=0.6, lw=0.8,
                    label=f"drone {did} {drone_col}")
            ax.plot(mt, mocap_vels[i], "r-", alpha=0.7, lw=1.2,
                    label=f"mocap body {vel_name}")
            ax.set_ylabel(f"{vel_name} [m/s]")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

        axes[-1].set_xlabel("Time [s]")
        plt.tight_layout()
        fig.savefig(out_dir / f"day02_velocities_drone{did}.png", dpi=150)
        plt.close(fig)
        print(f"  Saved day02_velocities_drone{did}.png")


def plot_velocity_alignment_debug(dfs, mocap, offsets, out_dir, title_prefix="Day 02", file_prefix="day02"):
    """Debug plot: compare drone velocity to mocap global and body-frame velocity."""
    out_dir.mkdir(parents=True, exist_ok=True)
    mapping = offsets["_mapping"]

    axis_meta = [
        ("X", "kalman.statePX"),
        ("Y", "kalman.statePY"),
        ("Z", "kalman.statePZ"),
    ]

    for did in sorted(dfs.keys()):
        if did == STATIC_DRONE:
            continue
        if did not in mapping:
            continue

        df = dfs[did]
        ts = df["timestamp"].values / 1000.0
        ts_mc = drone_to_mocap_time(ts, offsets, did)

        m = mocap[mapping[did]]
        mt = m["time"]
        
        # Check for mocap time discontinuities and fix
        mt_check = np.diff(mt)
        if (mt_check < -0.1).any():
            print(f"  Plot drone {did}: detected mocap time discontinuities, fixing...")
            # Extract all mocap data into dict for fixing
            mocap_data = {
                "x": m["x"],
                "y": m["y"],
                "z": m["z"],
                "qx": m["qx"],
                "qy": m["qy"],
                "qz": m["qz"],
                "qw": m["qw"],
                "yaw": m["yaw"],
                "pitch": m["pitch"],
                "roll": m["roll"],
            }
            if "valid_orig" in m:
                mocap_data["valid_orig"] = m["valid_orig"]
            mt_fixed, mocap_data_fixed = _enforce_monotonic_time(mt, mocap_data)
            # Update mocap dict with fixed data
            for key in mocap_data_fixed:
                m[key] = mocap_data_fixed[key]
            m["time"] = mt_fixed
            mt = mt_fixed
        
        # Recompute velocities with fixed (monotonic) time
        vx_w, vy_w, vz_w = mocap_global_velocity(m)
        vx_b, vy_b, vz_b = mocap_body_frame_velocity(m)
        mocap_world = [vx_w, vy_w, vz_w]
        mocap_body = [vx_b, vy_b, vz_b]

        fig, axes = plt.subplots(3, 2, figsize=(16, 10), sharex="col")
        fig.suptitle(
            f"{title_prefix} Velocity Debug Drone {did}: Global vs Body Frame",
            fontsize=14,
            fontweight="bold",
        )

        for row, (axis_name, kalman_col) in enumerate(axis_meta):
            drone_v = df[kalman_col].values
            # Resample drone velocity onto mocap time grid to avoid visual artifacts
            # from mixing different sampling rates (500 Hz drone vs 180 Hz mocap)
            drone_v_resampled = np.interp(mt, ts_mc, drone_v)

            ax = axes[row, 0]
            ax.plot(mt, mocap_world[row], color="tab:red", lw=1.1, alpha=0.8,
                    label=f"mocap global v{axis_name.lower()}")
            ax.plot(mt, drone_v_resampled, color="tab:blue", lw=0.8, alpha=0.6,
                    label=f"drone {kalman_col} (body, resampled)")
            ax.set_ylabel(f"v{axis_name.lower()} [m/s]")
            ax.set_ylim(-1.3, 1.3)
            ax.grid(True, alpha=0.3)
            if row == 0:
                ax.set_title("Mocap global velocity vs drone body velocity")
            ax.legend(fontsize=8)

            ax = axes[row, 1]
            ax.plot(mt, mocap_body[row], color="tab:green", lw=1.1, alpha=0.8,
                    label=f"mocap body v{axis_name.lower()}")
            ax.plot(mt, drone_v_resampled, color="tab:blue", lw=0.8, alpha=0.6,
                    label=f"drone {kalman_col} (resampled)")
            ax.set_ylim(-1.3, 1.3)
            ax.grid(True, alpha=0.3)
            if row == 0:
                ax.set_title("Mocap velocity rotated to body (roll+pitch+yaw)")
            ax.legend(fontsize=8)

        axes[-1, 0].set_xlabel("Mocap time [s]")
        axes[-1, 1].set_xlabel("Mocap time [s]")
        plt.tight_layout()
        fig.savefig(out_dir / f"{file_prefix}_velocity_debug_drone{did}.png", dpi=150)
        plt.close(fig)
        print(f"  Saved {file_prefix}_velocity_debug_drone{did}.png")


def mocap_yaw_rate(mocap_drone):
    """Compute yaw rate [deg/s] from mocap yaw angle via smoothed differentiation."""
    t = mocap_drone["time"]
    yaw = np.unwrap(mocap_drone["yaw"])  # radians, unwrapped
    valid_orig = mocap_drone.get("valid_orig", np.ones(len(t), dtype=bool))

    # Smooth before differentiation (same window as body-frame velocity)
    win = min(19, len(t) // 2 * 2 - 1)
    if win >= 5:
        yaw_s = savgol_filter(yaw, win, 3)
    else:
        yaw_s = yaw

    unique_idx = np.concatenate(([True], np.diff(t) > 1e-9))
    t_u = t[unique_idx]
    yaw_u = yaw_s[unique_idx]
    yaw_rate_u = np.gradient(yaw_u, t_u)  # rad/s
    yaw_rate_rad = np.interp(t, t_u, yaw_rate_u)

    # Mask yaw rate as NaN where data was interpolated (dropout regions)
    if valid_orig is not None:
        interp_regions = ~valid_orig
        if interp_regions.any():
            dilated = interp_regions.copy()
            dilated[:-1] |= interp_regions[1:]
            dilated[1:] |= interp_regions[:-1]
            yaw_rate_rad[dilated] = np.nan

    # Reject outlier spikes from marker tracking glitches:
    # replace samples > 5*median(|rate|) with interpolated neighbors
    yr_deg = np.degrees(yaw_rate_rad)
    valid_yr = ~np.isnan(yr_deg)
    if valid_yr.sum() > 0:
        med = np.median(np.abs(yr_deg[valid_yr][np.abs(yr_deg[valid_yr]) > 0.1]))  # avoid zero floor
        if med > 0:
            thresh = max(5 * med, 200.0)  # at least 200 deg/s
            bad = np.abs(yr_deg) > thresh
            if bad.any():
                good = ~bad & valid_yr
                if good.sum() > 0:
                    yr_deg[bad] = np.interp(t[bad], t[good], yr_deg[good])

    return yr_deg


def plot_yawrate_comparison(dfs, mocap, offsets, out_dir, title_prefix="Day 02", file_prefix="day02"):
    """Plot drone yaw rate vs mocap-derived yaw rate for flying drones."""
    out_dir.mkdir(parents=True, exist_ok=True)
    mapping = offsets["_mapping"]

    deg2millirad = np.pi * 1000.0 / 180.0

    for did in sorted(dfs.keys()):
        if did == STATIC_DRONE:
            continue
        if did not in mapping:
            continue

        df = dfs[did]
        ts = df["timestamp"].values / 1000.0
        ts_mc = drone_to_mocap_time(ts, offsets, did)

        # Drone's own yaw rate: stateEstimateZ.rateYaw (millirad/s) → deg/s
        drone_yawrate = df["stateEstimateZ.rateYaw"].values / deg2millirad

        # Mocap yaw rate
        m = mocap[mapping[did]]
        mt = m["time"]
        mc_yawrate = mocap_yaw_rate(m)

        # Low-pass filter both for trend comparison.
        # Drone is ~500 Hz, mocap is 180 Hz → resample drone to mocap grid first.
        drone_on_mc = np.interp(mt, ts_mc, drone_yawrate)

        # Savgol smooth: ~0.5 s window on mocap grid (180 Hz → 91 samples, odd)
        smooth_win = min(91, len(mt) // 2 * 2 - 1)
        if smooth_win >= 5:
            drone_smooth = savgol_filter(drone_on_mc, smooth_win, 3)
            mc_smooth = savgol_filter(mc_yawrate, smooth_win, 3)
        else:
            drone_smooth = drone_on_mc
            mc_smooth = mc_yawrate

        fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
        fig.suptitle(f"{title_prefix}: Drone {did} Yaw Rate Comparison",
                     fontsize=14, fontweight="bold")

        # Top: raw (resampled)
        ax = axes[0]
        ax.plot(mt, drone_on_mc, "b-", alpha=0.3, lw=0.5,
                label=f"drone {did} gyro.z (resampled)")
        ax.plot(mt, mc_yawrate, "r-", alpha=0.5, lw=0.8,
                label="mocap d(yaw)/dt")
        ax.set_ylabel("Yaw rate [deg/s]")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_title("Raw (resampled to mocap grid)")

        # Bottom: smoothed (~0.5 s window)
        ax = axes[1]
        ax.plot(mt, drone_smooth, "b-", alpha=0.8, lw=1.2,
                label=f"drone {did} gyro.z (smoothed)")
        ax.plot(mt, mc_smooth, "r-", alpha=0.8, lw=1.2,
                label="mocap d(yaw)/dt (smoothed)")
        ax.set_ylabel("Yaw rate [deg/s]")
        ax.set_xlabel("Mocap time [s]")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_title("Smoothed (~0.5 s window)")

        plt.tight_layout()
        fig.savefig(out_dir / f"{file_prefix}_yawrate_drone{did}.png", dpi=150)
        plt.close(fig)
        print(f"  Saved {file_prefix}_yawrate_drone{did}.png")


def plot_overview(mocap, offsets, out_dir):
    """XY trajectory and inter-drone distances from mocap."""
    out_dir.mkdir(parents=True, exist_ok=True)
    mapping = offsets["_mapping"]
    colors = ["tab:blue", "tab:orange", "tab:green"]

    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    fig.suptitle("Day 02: Overview", fontsize=14, fontweight="bold")

    # XY trajectory
    ax = axes[0]
    for did in sorted(mapping.keys()):
        mid = mapping[did]
        m = mocap[mid]
        label = f"Drone {did} ({m['name']})"
        if did == STATIC_DRONE:
            ax.scatter(m["x"].mean(), m["y"].mean(), color=colors[did],
                       marker="x", s=100, zorder=5, label=label + " [static]")
        else:
            ax.plot(m["x"], m["y"], color=colors[did], alpha=0.7, label=label)
            ax.scatter(m["x"][0], m["y"][0], color=colors[did],
                       marker="o", s=60, zorder=5)
    ax.set_xlabel("X (forward) [m]")
    ax.set_ylabel("Y (left) [m]")
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title("Mocap XY Trajectories (CF Frame)")

    # Inter-drone distances
    ax = axes[1]
    pairs = [(0, 1), (0, 2), (1, 2)]
    pair_colors = ["tab:blue", "tab:orange", "tab:green"]
    for idx, (i, j) in enumerate(pairs):
        if i not in mapping or j not in mapping:
            continue
        mi = mocap[mapping[i]]
        mj = mocap[mapping[j]]
        dx = mj["x"] - mi["x"]
        dy = mj["y"] - mi["y"]
        dz = mj["z"] - mi["z"]
        d = np.sqrt(dx**2 + dy**2 + dz**2)
        ax.plot(mi["time"], d, color=pair_colors[idx], alpha=0.8,
                label=f"d({i}→{j})")
    ax.set_ylabel("Distance [m]")
    ax.set_xlabel("Mocap time [s]")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title("Inter-Drone Distances")

    plt.tight_layout()
    fig.savefig(out_dir / "day02_overview.png", dpi=150)
    plt.close(fig)
    print(f"  Saved day02_overview.png")


def plot_pairwise_component_distances_global(mocap, offsets, out_dir):
    """Plot pairwise dx, dy, dz in global frame for all drone pairs."""
    out_dir.mkdir(parents=True, exist_ok=True)
    mapping = offsets["_mapping"]
    pairs = [(0, 1), (0, 2), (1, 2)]
    pair_colors = {
        (0, 1): "tab:blue",
        (0, 2): "tab:orange",
        (1, 2): "tab:green",
    }

    fig, axes = plt.subplots(3, 1, figsize=(15, 10), sharex=True)
    fig.suptitle("Day 02: Pairwise Relative Components in Global Frame",
                 fontsize=14, fontweight="bold")

    axis_names = ["x", "y", "z"]
    for (i, j) in pairs:
        if i not in mapping or j not in mapping:
            continue
        mi = mocap[mapping[i]]
        mj = mocap[mapping[j]]
        t = mi["time"]
        deltas = {
            "x": mj["x"] - mi["x"],
            "y": mj["y"] - mi["y"],
            "z": mj["z"] - mi["z"],
        }
        for idx, comp in enumerate(axis_names):
            axes[idx].plot(t, deltas[comp], color=pair_colors[(i, j)], lw=1.2,
                           alpha=0.8, label=f"d{comp}({i}->{j})")

    for idx, comp in enumerate(axis_names):
        axes[idx].set_ylabel(f"d{comp} [m]")
        axes[idx].grid(True, alpha=0.3)
        axes[idx].legend(fontsize=9, loc="upper right")
    axes[-1].set_xlabel("Mocap time [s]")

    plt.tight_layout()
    fig.savefig(out_dir / "day02_pairwise_components_global.png", dpi=150)
    plt.close(fig)
    print("  Saved day02_pairwise_components_global.png")


def plot_relative_components_in_drone0_body(mocap, offsets, out_dir):
    """Plot drone 1 and 2 position relative to drone 0 in drone 0 body frame."""
    out_dir.mkdir(parents=True, exist_ok=True)
    mapping = offsets["_mapping"]
    if 0 not in mapping or 1 not in mapping or 2 not in mapping:
        print("  Skipping drone 0 body-frame relative plot: missing mapping")
        return

    m0 = mocap[mapping[0]]
    m1 = mocap[mapping[1]]
    m2 = mocap[mapping[2]]
    t = m0["time"]

    q0 = np.column_stack([m0["qx"], m0["qy"], m0["qz"], m0["qw"]])
    q0 /= np.clip(np.linalg.norm(q0, axis=1, keepdims=True), 1e-12, None)
    r0 = Rotation.from_quat(q0)

    rel01_world = np.column_stack([
        m1["x"] - m0["x"],
        m1["y"] - m0["y"],
        m1["z"] - m0["z"],
    ])
    rel02_world = np.column_stack([
        m2["x"] - m0["x"],
        m2["y"] - m0["y"],
        m2["z"] - m0["z"],
    ])

    rel01_body = r0.inv().apply(rel01_world)
    rel02_body = r0.inv().apply(rel02_world)

    fig, axes = plt.subplots(3, 1, figsize=(15, 10), sharex=True)
    fig.suptitle("Day 02: Relative Components in Drone 0 Body Frame",
                 fontsize=14, fontweight="bold")

    for idx, axis_name in enumerate(["x", "y", "z"]):
        axes[idx].plot(t, rel01_body[:, idx], color="tab:blue", lw=1.2,
                       alpha=0.8, label=f"d{axis_name}(0->1) in body0")
        axes[idx].plot(t, rel02_body[:, idx], color="tab:orange", lw=1.2,
                       alpha=0.8, label=f"d{axis_name}(0->2) in body0")
        axes[idx].set_ylabel(f"d{axis_name} [m]")
        axes[idx].grid(True, alpha=0.3)
        axes[idx].legend(fontsize=9, loc="upper right")

    axes[-1].set_xlabel("Mocap time [s]")
    plt.tight_layout()
    fig.savefig(out_dir / "day02_relative_components_body0.png", dpi=150)
    plt.close(fig)
    print("  Saved day02_relative_components_body0.png")


# ──────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────

def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("1. Decoding binary drone logs → raw CSVs")
    print("=" * 60)
    dfs = decode_drone_logs()

    print("\n" + "=" * 60)
    print("2. Creating zeroed-drone0 CSVs")
    print("=" * 60)
    zeroed = create_zeroed_csvs(dfs)

    print("\n" + "=" * 60)
    print("3. Parsing mocap data")
    print("=" * 60)
    mocap = parse_mocap()

    print("\n" + "=" * 60)
    print("4. Synchronizing time")
    print("=" * 60)
    offsets = sync_all(dfs, mocap)

    print("\n" + "=" * 60)
    print("5. Building synced CSVs")
    print("=" * 60)
    synced_df = build_synced_csv(
        dfs,
        mocap,
        offsets,
        dt=0.02,
        output_name="synced_all.csv",
    )
    synced_df_500hz = build_synced_csv(
        dfs,
        mocap,
        offsets,
        dt=0.002,
        output_name="synced_all_500hz.csv",
    )

    print("\n" + "=" * 60)
    print("6. Plotting")
    print("=" * 60)
    plot_height_sync(dfs, mocap, offsets, OUTPUT_DIR)
    plot_velocity_comparison(dfs, mocap, offsets, OUTPUT_DIR)
    plot_velocity_alignment_debug(dfs, mocap, offsets, OUTPUT_DIR)
    plot_yawrate_comparison(dfs, mocap, offsets, OUTPUT_DIR)
    plot_overview(mocap, offsets, OUTPUT_DIR)
    plot_pairwise_component_distances_global(mocap, offsets, OUTPUT_DIR)
    plot_relative_components_in_drone0_body(mocap, offsets, OUTPUT_DIR)

    print(f"\nDone! Outputs in {DATA_DIR}/ and {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
