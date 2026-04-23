#!/usr/bin/env python3
"""
EKF Ablation & Sensitivity Study.

Runs ekf_replay.py with systematic feature toggling and parameter sweeps
to quantify the contribution of each modelling feature.

Tests:
  1. Ablation (2^3 = 8 combos): drag on/off × CoP offset on/off × flowdeck offset on/off
  2. Sensitivity: with all features on, scale each feature ±10%, ±20%, … ±50%

Usage:
    python ekf_ablation.py                                          # small (6 CSVs), default
    python ekf_ablation.py --large                                  # all CSVs in DataFlapperEKF/
    python ekf_ablation.py --csv DataFlapperEKF/flight_sine_mtf.csv # single CSV
    python ekf_ablation.py --no-sensitivity                         # ablation only
    python ekf_ablation.py --no-ablation                            # sensitivity only
    python ekf_ablation.py --save ekf_study --save-plots plots      # save data + PNGs
"""

from __future__ import annotations

import argparse
import contextlib
import itertools
import os
import warnings
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from ekf_replay import EKFParams, run_ekf

warnings.filterwarnings("ignore", category=FutureWarning)

# ---------------------------------------------------------------------------
# Baseline (tuned) parameters — must match what main() in ekf_replay.py uses
# ---------------------------------------------------------------------------

BASELINE_PARAMS = dict(
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

# CSVs to test — "small" is the curated 6 used for tuning; "large" is every file
SMALL_CSVS = [
    "DataFlapperEKF/flight_diag_lr_mtf.csv",
    "DataFlapperEKF/flight_sine_mtf.csv",
    "DataFlapperEKF/flight_y_mtf.csv",
    "DataFlapperEKF/flight_z_mtf.csv",
    "DataFlapperEKF/square_60cm_mtf.csv",
    "DataFlapperEKF/square_120cm_mtf.csv",
]

ALL_CSVS = sorted(
    str(p) for p in Path("DataFlapperEKF").glob("*.csv")
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _rmse(a: np.ndarray, b: np.ndarray) -> float:
    d = a - b
    return float(np.sqrt(np.nanmean(d * d)))


def _compute_rmse(results: pd.DataFrame) -> dict[str, float]:
    """Return a dict of RMSE metrics from a run_ekf result DataFrame."""
    out = {}
    if "ls_x" in results.columns:
        out["pos_x"] = _rmse(results["x"].to_numpy(), results["ls_x"].to_numpy())
        out["pos_y"] = _rmse(results["y"].to_numpy(), results["ls_y"].to_numpy())
        out["pos_z"] = _rmse(results["z"].to_numpy(), results["ls_z"].to_numpy())
        out["pos_total"] = np.sqrt(out["pos_x"]**2 + out["pos_y"]**2 + out["pos_z"]**2)
    if "ls_vx_b" in results.columns:
        out["vel_x"] = _rmse(results["vx_b"].to_numpy(), results["ls_vx_b"].to_numpy())
        out["vel_y"] = _rmse(results["vy_b"].to_numpy(), results["ls_vy_b"].to_numpy())
        out["vel_z"] = _rmse(results["vz_b"].to_numpy(), results["ls_vz_b"].to_numpy())
        out["vel_total"] = np.sqrt(out["vel_x"]**2 + out["vel_y"]**2 + out["vel_z"]**2)
    if "ls_roll" in results.columns:
        out["att_roll"]  = _rmse(results["roll"].to_numpy(),  results["ls_roll"].to_numpy())
        out["att_pitch"] = _rmse(results["pitch"].to_numpy(), results["ls_pitch"].to_numpy())
        out["att_yaw"]   = _rmse(results["yaw"].to_numpy(),   results["ls_yaw"].to_numpy())
    return out


def _make_params(**overrides) -> EKFParams:
    """Build EKFParams from BASELINE_PARAMS with overrides."""
    merged = {**BASELINE_PARAMS, **overrides}
    return EKFParams(**merged)


def _run_silent(
    csv_path: str,
    params: EKFParams,
    use_flow: bool = True,
    use_tof: bool = True,
) -> pd.DataFrame:
    """Run EKF with stdout suppressed."""
    with open(os.devnull, "w") as devnull, contextlib.redirect_stdout(devnull):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            return run_ekf(csv_path, params=params, use_flow=use_flow, use_tof=use_tof)


def run_velocity_measurement_comparison(csv_files: list[str]) -> pd.DataFrame:
    """
    Compare two full-model scenarios on velocity RMSE:
      1) full measurements (ToF + optical flow)
      2) no flow-sensor measurements (no ToF + no optical flow)
    """
    scenarios = [
        ("full measurements", True, True),
        ("no flow-sensor measurements", False, False),
    ]

    rows = []
    params = _make_params()
    for scenario_name, use_flow, use_tof in scenarios:
        for csv_path in csv_files:
            if not Path(csv_path).exists():
                continue
            print(f"  Scenario [{scenario_name}] on {Path(csv_path).name} ...")
            try:
                results = _run_silent(csv_path, params, use_flow=use_flow, use_tof=use_tof)
                rmses = _compute_rmse(results)
            except Exception as e:
                print(f"    ⚠ failed: {e}")
                continue

            rows.append({
                "scenario": scenario_name,
                "use_flow": use_flow,
                "use_tof": use_tof,
                "csv": Path(csv_path).stem,
                "vel_total": rmses.get("vel_total", np.nan),
                "vel_x": rmses.get("vel_x", np.nan),
                "vel_y": rmses.get("vel_y", np.nan),
                "vel_z": rmses.get("vel_z", np.nan),
            })

    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# 1. Ablation study
# ---------------------------------------------------------------------------

@dataclass
class AblationCase:
    name: str
    use_drag: bool
    use_cop: bool
    use_flowdeck_pos: bool


def build_ablation_cases() -> list[AblationCase]:
    """All 8 on/off combinations of the three features."""
    cases = []
    for drag, cop, fdp in itertools.product([True, False], repeat=3):
        parts = []
        parts.append("drag" if drag else "no-drag")
        parts.append("cop" if cop else "no-cop")
        parts.append("fdp" if fdp else "no-fdp")
        name = " | ".join(parts)
        cases.append(AblationCase(name=name, use_drag=drag,
                                  use_cop=cop, use_flowdeck_pos=fdp))
    return cases


def run_ablation(csv_files: list[str]) -> pd.DataFrame:
    """Run all ablation cases and return a tidy DataFrame of RMSE results."""
    cases = build_ablation_cases()
    rows = []

    for case in cases:
        overrides = {}
        if not case.use_drag:
            overrides.update(drag_x=0.0, drag_y=0.0, drag_z=0.0)
        if not case.use_cop:
            overrides["cop"] = np.zeros(3)
        if not case.use_flowdeck_pos:
            overrides["flowdeck_pos"] = np.zeros(3)

        params = _make_params(**overrides)

        for csv_path in csv_files:
            if not Path(csv_path).exists():
                continue
            print(f"  Ablation [{case.name}] on {Path(csv_path).name} ...")
            try:
                results = _run_silent(csv_path, params)
                rmses = _compute_rmse(results)
            except Exception as e:
                print(f"    ⚠ failed: {e}")
                continue

            rows.append({
                "case": case.name,
                "drag": case.use_drag,
                "cop": case.use_cop,
                "fdp": case.use_flowdeck_pos,
                "csv": Path(csv_path).stem,
                **rmses,
            })

    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# 2. Sensitivity study
# ---------------------------------------------------------------------------

SENSITIVITY_FEATURES = {
    "drag": ["drag_x", "drag_y", "drag_z"],
    "cop_z": ["cop"],
    "flowdeck_pos_z": ["flowdeck_pos"],
}

SCALE_PERCENTS = [-50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50]


def _scale_param(name: str, scale_frac: float) -> dict:
    """Return an override dict with the named feature scaled by (1 + scale_frac)."""
    factor = 1.0 + scale_frac
    if name == "drag":
        return dict(
            drag_x=BASELINE_PARAMS["drag_x"] * factor,
            drag_y=BASELINE_PARAMS["drag_y"] * factor,
            drag_z=BASELINE_PARAMS["drag_z"] * factor,
        )
    elif name == "cop_z":
        base_z = BASELINE_PARAMS["cop"][2]
        return dict(cop=np.array([0.0, 0.0, base_z * factor]))
    elif name == "flowdeck_pos_z":
        base_z = BASELINE_PARAMS["flowdeck_pos"][2]
        return dict(flowdeck_pos=np.array([0.0, 0.0, base_z * factor]))
    else:
        raise ValueError(f"Unknown feature: {name}")


def run_sensitivity(csv_files: list[str]) -> pd.DataFrame:
    """Sweep each feature ±10%…±50% and return a tidy DataFrame."""
    rows = []

    for feat_name in SENSITIVITY_FEATURES:
        for pct in SCALE_PERCENTS:
            scale_frac = pct / 100.0
            overrides = _scale_param(feat_name, scale_frac)
            params = _make_params(**overrides)

            for csv_path in csv_files:
                if not Path(csv_path).exists():
                    continue
                print(f"  Sensitivity [{feat_name} {pct:+d}%] on {Path(csv_path).name} ...")
                try:
                    results = _run_silent(csv_path, params)
                    rmses = _compute_rmse(results)
                except Exception as e:
                    print(f"    ⚠ failed: {e}")
                    continue

                rows.append({
                    "feature": feat_name,
                    "scale_pct": pct,
                    "csv": Path(csv_path).stem,
                    **rmses,
                })

    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# Plotting helpers
# ---------------------------------------------------------------------------

def _get_csv_colormap(df: pd.DataFrame) -> dict[str, str]:
    """Assign a unique colour to each CSV/trajectory in the DataFrame."""
    csvs = sorted(df["csv"].unique())
    cmap = plt.get_cmap("tab10") if len(csvs) <= 10 else plt.get_cmap("tab20")
    return {name: cmap(i) for i, name in enumerate(csvs)}


def _boxplot_with_points(ax, df: pd.DataFrame, group_col: str, metric: str,
                         csv_colors: dict[str, str]):
    """
    Draw a box-plot of *metric* grouped by *group_col*, then overlay
    individual trajectory points coloured by CSV name.

    Each trajectory always gets the same deterministic x-offset within
    every box so the dot pattern is consistent across all plots.
    """
    groups = df[group_col].unique()           # preserve original order
    group_data = [df.loc[df[group_col] == g, metric].dropna().values for g in groups]

    # Sorted trajectory names — defines the fixed left-to-right order
    csv_names = sorted(csv_colors.keys())
    n_csv = len(csv_names)
    # Evenly spaced offsets within the box width, centred on 0
    spread = 0.30                             # total width of the point band
    if n_csv > 1:
        offsets = np.linspace(-spread / 2, spread / 2, n_csv)
    else:
        offsets = np.array([0.0])
    csv_offset = {name: offsets[j] for j, name in enumerate(csv_names)}

    bp = ax.boxplot(group_data, positions=np.arange(len(groups)),
                    widths=0.45, patch_artist=True, showfliers=False,
                    boxprops=dict(facecolor="lightsteelblue", alpha=0.55),
                    medianprops=dict(color="black", linewidth=1.5),
                    whiskerprops=dict(linewidth=1),
                    capprops=dict(linewidth=1))

    # Scatter individual points with per-trajectory colour & fixed offset
    plotted_labels: set[str] = set()
    for i, g in enumerate(groups):
        sub = df[df[group_col] == g]
        for _, row in sub.iterrows():
            csv_name = row["csv"]
            colour = csv_colors.get(csv_name, "gray")
            label = csv_name if csv_name not in plotted_labels else None
            if label:
                plotted_labels.add(csv_name)
            ax.scatter(i + csv_offset.get(csv_name, 0.0),
                       row[metric], color=colour, edgecolors="black",
                       linewidths=0.4, s=48, zorder=5, label=label)

    ax.set_xticks(np.arange(len(groups)))
    ax.set_xticklabels(groups, rotation=45, ha="right", fontsize=8)
    ax.grid(axis="y", alpha=0.3)


# ---------------------------------------------------------------------------
# Plotting – ablation
# ---------------------------------------------------------------------------

def plot_ablation(df: pd.DataFrame, save_dir: str | None = None):
    """Box-plot + scatter for each velocity axis and total across ablation cases."""
    csv_colors = _get_csv_colormap(df)
    n_traj = len(csv_colors)
    fig_w = max(14, 1.6 * n_traj + 4)

    vel_metrics = [
        ("vel_x",     "Body-frame vx RMSE (m/s)"),
        ("vel_y",     "Body-frame vy RMSE (m/s)"),
        ("vel_z",     "Body-frame vz RMSE (m/s)"),
        ("vel_total", "Body-frame velocity total RMSE (m/s)"),
    ]

    att_metrics = [
        ("att_roll",  "Roll RMSE (deg)"),
        ("att_pitch", "Pitch RMSE (deg)"),
        ("att_yaw",   "Yaw RMSE (deg)"),
    ]

    for metric, ylabel in vel_metrics + att_metrics:
        if metric not in df.columns:
            continue

        fig, ax = plt.subplots(figsize=(fig_w, 6))
        fig.suptitle(f"Ablation: {ylabel}", fontsize=13)

        _boxplot_with_points(ax, df, "case", metric, csv_colors)
        ax.set_ylabel(ylabel)

        # De-duplicate legend entries, place outside
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(),
                  fontsize=8, title="Trajectory", title_fontsize=9,
                  loc="center left", bbox_to_anchor=(1.01, 0.5),
                  framealpha=0.8)
        fig.subplots_adjust(right=0.78)

        if save_dir:
            tag = metric.replace("_", "")
            fig.savefig(f"{save_dir}/ablation_{tag}.png", dpi=150,
                        bbox_inches="tight")
            plt.close(fig)

    # --- Console summary: which feature removal hurts most ---
    agg = df.groupby("case", sort=False).mean(numeric_only=True).reset_index()
    if "vel_total" in agg.columns:
        baseline_row = agg[agg["case"].str.contains("drag.*cop.*fdp") &
                           ~agg["case"].str.contains("no-")]
        if not baseline_row.empty:
            baseline_vel = baseline_row["vel_total"].values[0]
            print("\n===== Feature Impact (velocity RMSE increase vs full model) =====")
            for _, row in agg.iterrows():
                delta = row["vel_total"] - baseline_vel
                pct = (delta / baseline_vel) * 100 if baseline_vel > 0 else 0
                marker = " ← BASELINE" if abs(delta) < 1e-6 else ""
                print(f"  {row['case']:40s}  vel_total={row['vel_total']:.4f}  "
                      f"Δ={delta:+.4f} ({pct:+.1f}%){marker}")
            print("=" * 65)


def plot_ablation_by_trajectory(df: pd.DataFrame, save_dir: str | None = None):
    """
    Trajectory-centric ablation view.
    X-axis = trajectory name.  Each ablation case uses a unique marker
    symbol, while the dot colour matches the trajectory's colour from
    the box-plots (consistent across all plots).
    """
    csv_colors = _get_csv_colormap(df)
    cases = list(df["case"].unique())
    csvs = sorted(df["csv"].unique())
    fig_w = max(14, 1.6 * len(csvs) + 4)

    # One unique marker per ablation case
    _MARKERS = ["o", "s", "^", "D", "v", "P", "X", "*", "h", "<", ">", "p"]
    case_marker = {c: _MARKERS[i % len(_MARKERS)] for i, c in enumerate(cases)}

    vel_metrics = [
        ("vel_x",     "Body-frame vx RMSE (m/s)"),
        ("vel_y",     "Body-frame vy RMSE (m/s)"),
        ("vel_z",     "Body-frame vz RMSE (m/s)"),
        ("vel_total", "Body-frame velocity total RMSE (m/s)"),
    ]

    att_metrics = [
        ("att_roll",  "Roll RMSE (deg)"),
        ("att_pitch", "Pitch RMSE (deg)"),
        ("att_yaw",   "Yaw RMSE (deg)"),
    ]

    n_cases = len(cases)
    spread = 0.6
    if n_cases > 1:
        case_offsets = np.linspace(-spread / 2, spread / 2, n_cases)
    else:
        case_offsets = np.array([0.0])
    case_offset = {c: case_offsets[j] for j, c in enumerate(cases)}

    for metric, ylabel in vel_metrics + att_metrics:
        if metric not in df.columns:
            continue

        fig, ax = plt.subplots(figsize=(fig_w, 6))
        fig.suptitle(f"Ablation by Trajectory: {ylabel}", fontsize=13)

        # Plot points: colour = trajectory, marker = case
        for c in cases:
            marker = case_marker[c]
            sub = df[df["case"] == c]
            for _, row in sub.iterrows():
                csv_idx = csvs.index(row["csv"])
                colour = csv_colors.get(row["csv"], "gray")
                ax.scatter(csv_idx + case_offset[c],
                           row[metric], color=colour, marker=marker,
                           edgecolors="black", linewidths=0.4, s=55, zorder=5)

        # Legend: one entry per case (gray marker to show shape)
        for c in cases:
            ax.scatter([], [], color="dimgray", marker=case_marker[c],
                       edgecolors="black", linewidths=0.4, s=55, label=c)

        ax.set_xticks(np.arange(len(csvs)))
        ax.set_xticklabels(csvs, rotation=30, ha="right", fontsize=9)
        ax.set_ylabel(ylabel)
        ax.set_xlabel("Trajectory")
        ax.grid(axis="y", alpha=0.3)
        ax.legend(fontsize=7, title="Ablation case (marker = config)",
                  title_fontsize=8,
                  loc="center left", bbox_to_anchor=(1.01, 0.5),
                  framealpha=0.8)
        fig.subplots_adjust(right=0.72)

        if save_dir:
            tag = metric.replace("_", "")
            fig.savefig(f"{save_dir}/ablation_traj_{tag}.png", dpi=150,
                        bbox_inches="tight")
            plt.close(fig)


# ---------------------------------------------------------------------------
# Plotting – sensitivity
# ---------------------------------------------------------------------------

def plot_sensitivity(df: pd.DataFrame, save_dir: str | None = None):
    """
    For each feature (drag, cop_z, flowdeck_pos_z) and each velocity axis,
    draw a box-plot + scatter at every scale-% tick.
    """
    csv_colors = _get_csv_colormap(df)
    n_traj = len(csv_colors)
    fig_w = max(12, 1.6 * n_traj + 4)
    features = df["feature"].unique()

    vel_metrics = [
        ("vel_x",     "Body-frame vx RMSE (m/s)"),
        ("vel_y",     "Body-frame vy RMSE (m/s)"),
        ("vel_z",     "Body-frame vz RMSE (m/s)"),
        ("vel_total", "Body-frame velocity total RMSE (m/s)"),
    ]

    att_metrics = [
        ("att_roll",  "Roll RMSE (deg)"),
        ("att_pitch", "Pitch RMSE (deg)"),
        ("att_yaw",   "Yaw RMSE (deg)"),
    ]

    for feat in features:
        feat_df = df[df["feature"] == feat].copy()
        feat_df["scale_label"] = feat_df["scale_pct"].apply(lambda p: f"{p:+d}%")
        sorted_labels = (feat_df[["scale_pct", "scale_label"]]
                         .drop_duplicates()
                         .sort_values("scale_pct")["scale_label"]
                         .tolist())
        feat_df["scale_label"] = pd.Categorical(
            feat_df["scale_label"], categories=sorted_labels, ordered=True)

        for metric, ylabel in vel_metrics + att_metrics:
            if metric not in feat_df.columns:
                continue

            fig, ax = plt.subplots(figsize=(fig_w, 6))
            fig.suptitle(f"Sensitivity: {feat} — {ylabel}", fontsize=13)

            _boxplot_with_points(ax, feat_df, "scale_label", metric, csv_colors)
            ax.set_xlabel(f"{feat} scale")
            ax.set_ylabel(ylabel)

            zero_idx = sorted_labels.index("+0%") if "+0%" in sorted_labels else None
            if zero_idx is not None:
                ax.axvline(zero_idx, color="gray", linestyle="--", alpha=0.5,
                           label="baseline (0%)")

            # De-duplicate legend, place outside
            handles, labels = ax.get_legend_handles_labels()
            by_label = dict(zip(labels, handles))
            ax.legend(by_label.values(), by_label.keys(),
                      fontsize=8, title="Trajectory", title_fontsize=9,
                      loc="center left", bbox_to_anchor=(1.01, 0.5),
                      framealpha=0.8)
            fig.subplots_adjust(right=0.78)

            if save_dir:
                tag = metric.replace("_", "")
                fig.savefig(f"{save_dir}/sensitivity_{feat}_{tag}.png",
                            dpi=150, bbox_inches="tight")
                plt.close(fig)


def plot_sensitivity_by_trajectory(df: pd.DataFrame, save_dir: str | None = None):
    """
    Trajectory-centric sensitivity view.
    X-axis = trajectory name.  Each scale-% is a dot at a fixed x-offset.
    Dot colour matches the trajectory's colour from the box-plots, while
    alpha encodes how far the scale is from baseline (0%).
    Baseline dots are fully opaque; ±50% most transparent.
    """
    csv_colors = _get_csv_colormap(df)
    features = df["feature"].unique()
    csvs = sorted(df["csv"].unique())
    fig_w = max(14, 1.6 * len(csvs) + 4)

    vel_metrics = [
        ("vel_x",     "Body-frame vx RMSE (m/s)"),
        ("vel_y",     "Body-frame vy RMSE (m/s)"),
        ("vel_z",     "Body-frame vz RMSE (m/s)"),
        ("vel_total", "Body-frame velocity total RMSE (m/s)"),
    ]

    att_metrics = [
        ("att_roll",  "Roll RMSE (deg)"),
        ("att_pitch", "Pitch RMSE (deg)"),
        ("att_yaw",   "Yaw RMSE (deg)"),
    ]

    for feat in features:
        feat_df = df[df["feature"] == feat].copy()
        pcts = sorted(feat_df["scale_pct"].unique())
        n_pcts = len(pcts)

        max_abs = max(abs(p) for p in pcts) if pcts else 1

        # alpha: baseline opaque, further away → more transparent
        def _pct_alpha(p):
            frac = abs(p) / max_abs if max_abs > 0 else 0
            return max(1.0 - 0.7 * frac, 0.2)

        spread = 0.6
        if n_pcts > 1:
            pct_offsets = np.linspace(-spread / 2, spread / 2, n_pcts)
        else:
            pct_offsets = np.array([0.0])
        pct_offset = {p: pct_offsets[j] for j, p in enumerate(pcts)}

        for metric, ylabel in vel_metrics + att_metrics:
            if metric not in feat_df.columns:
                continue

            fig, ax = plt.subplots(figsize=(fig_w, 6))
            fig.suptitle(f"Sensitivity by Trajectory: {feat} — {ylabel}", fontsize=13)

            # Plot dots: colour = trajectory, alpha = scale-%
            for p in pcts:
                alpha = _pct_alpha(p)
                sub = feat_df[feat_df["scale_pct"] == p]
                for _, row in sub.iterrows():
                    csv_idx = csvs.index(row["csv"])
                    colour = csv_colors.get(row["csv"], "gray")
                    ax.scatter(csv_idx + pct_offset[p],
                               row[metric], color=colour, alpha=alpha,
                               edgecolors="black", linewidths=0.3, s=50, zorder=5)

            # Legend: one entry per scale-% (use gray to show alpha encoding)
            for p in pcts:
                alpha = _pct_alpha(p)
                ax.scatter([], [], color="dimgray", alpha=alpha, edgecolors="black",
                           linewidths=0.3, s=50, label=f"{p:+d}%")

            ax.set_xticks(np.arange(len(csvs)))
            ax.set_xticklabels(csvs, rotation=30, ha="right", fontsize=9)
            ax.set_ylabel(ylabel)
            ax.set_xlabel("Trajectory")
            ax.grid(axis="y", alpha=0.3)
            ax.legend(fontsize=7, title=f"{feat} scale (α encodes distance from 0%)",
                      title_fontsize=8,
                      loc="center left", bbox_to_anchor=(1.01, 0.5),
                      framealpha=0.8)
            fig.subplots_adjust(right=0.82)

            if save_dir:
                tag = metric.replace("_", "")
                fig.savefig(f"{save_dir}/sensitivity_traj_{feat}_{tag}.png",
                            dpi=150, bbox_inches="tight")
                plt.close(fig)


def plot_velocity_measurement_comparison(df: pd.DataFrame, save_dir: str | None = None):
    """Create two clear velocity RMSE plots for the requested measurement scenarios."""
    if df.empty:
        return

    scenarios = [
        ("full measurements", "velocity_rmse_full_model_full_measurements.png"),
        ("no flow-sensor measurements", "velocity_rmse_full_model_no_flow_sensor.png"),
    ]

    for scenario_name, file_name in scenarios:
        sub = df[df["scenario"] == scenario_name].copy()
        if sub.empty:
            continue
        sub = sub.sort_values("csv")

        fig, ax = plt.subplots(figsize=(11, 5.5), constrained_layout=True)
        fig.suptitle(f"Velocity RMSE — Full Model, {scenario_name.title()}", fontsize=13)

        x = np.arange(len(sub))
        y = sub["vel_total"].to_numpy()
        ax.bar(x, y, width=0.65, color="#4C78A8", edgecolor="black", linewidth=0.6)

        mean_val = float(np.nanmean(y))
        ax.axhline(mean_val, color="#F58518", linestyle="--", linewidth=1.6,
                   label=f"mean = {mean_val:.3f} m/s")

        ax.set_xticks(x)
        ax.set_xticklabels(sub["csv"].tolist(), rotation=25, ha="right", fontsize=9)
        ax.set_ylabel("Velocity total RMSE (m/s)")
        ax.set_xlabel("Trajectory")
        ax.grid(axis="y", alpha=0.3)
        ax.legend(loc="upper right", framealpha=0.9)

        if save_dir:
            fig.savefig(f"{save_dir}/{file_name}", dpi=150, bbox_inches="tight")
            plt.close(fig)


# ---------------------------------------------------------------------------
# Plotting – feature importance (kept for quick summary)
# ---------------------------------------------------------------------------

def plot_feature_importance(ablation_df: pd.DataFrame, save_dir: str | None = None):
    """
    Compute and plot single-feature importance:
    ΔRMSE when removing just that one feature (vs all-on baseline).
    """
    agg = ablation_df.groupby(["drag", "cop", "fdp"], sort=False).mean(numeric_only=True).reset_index()

    base = agg[(agg["drag"] == True) & (agg["cop"] == True) & (agg["fdp"] == True)]
    if base.empty:
        return

    metrics = [m for m in ["vel_total", "pos_total", "att_roll", "att_pitch", "att_yaw"] if m in agg.columns]
    if not metrics:
        return

    removals = {
        "Remove drag":         dict(drag=False, cop=True, fdp=True),
        "Remove CoP offset":   dict(drag=True, cop=False, fdp=True),
        "Remove flowdeck pos": dict(drag=True, cop=True, fdp=False),
    }

    fig, ax = plt.subplots(figsize=(10, 6), constrained_layout=True)
    fig.suptitle("Feature Importance: RMSE increase when removing one feature", fontsize=13)

    labels = []
    deltas = {m: [] for m in metrics}

    for label, filt in removals.items():
        row = agg[(agg["drag"] == filt["drag"]) &
                  (agg["cop"] == filt["cop"]) &
                  (agg["fdp"] == filt["fdp"])]
        if row.empty:
            continue
        labels.append(label)
        for m in metrics:
            if m in base.columns and m in row.columns:
                delta = row[m].values[0] - base[m].values[0]
                deltas[m].append(delta)

    x = np.arange(len(labels))
    width = 0.35 if len(metrics) <= 2 else 0.25
    for j, metric in enumerate(metrics):
        if "pos" in metric:
            mlabel = "Position ΔRMSE (m)"
        elif "vel" in metric:
            mlabel = "Velocity ΔRMSE (m/s)"
        else:
            mlabel = "Attitude ΔRMSE (deg)"
        bars = ax.bar(x + j * width - (len(metrics) - 1) * width / 2, deltas[metric], width,
                      label=mlabel, edgecolor="white", linewidth=0.5)
        for bar, val in zip(bars, deltas[metric]):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                    f"{val:+.4f}", ha="center", va="bottom", fontsize=9)

    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=11)
    ax.set_ylabel("ΔRMSE (increase from baseline)")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)
    ax.axhline(0, color="k", linewidth=0.5)
    if save_dir:
        fig.savefig(f"{save_dir}/feature_importance.png", dpi=150, bbox_inches="tight")
        plt.close(fig)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="EKF Ablation & Sensitivity Study")
    size_group = parser.add_mutually_exclusive_group()
    size_group.add_argument("--small", action="store_true", default=True,
                            help="Use the curated 6 CSV subset (default)")
    size_group.add_argument("--large", action="store_true",
                            help="Use ALL CSVs in DataFlapperEKF/")
    parser.add_argument("--csv", type=str, nargs="+", default=None,
                        help="Explicit CSV file(s) to test (overrides --small/--large)")
    parser.add_argument("--no-ablation", action="store_true",
                        help="Skip the ablation (on/off combo) study")
    parser.add_argument("--no-sensitivity", action="store_true",
                        help="Skip the sensitivity (parameter sweep) study")
    parser.add_argument("--save", type=str, default=None,
                        help="Save results to CSV prefix (e.g. 'ekf_study' → "
                             "ekf_study_small_ablation.csv, ...)")
    parser.add_argument("--save-plots", type=str, default=None,
                        help="Base directory for plot PNGs (suffixed with _small/_large)")
    args = parser.parse_args()

    # Determine dataset and tag
    if args.csv is not None:
        csv_files = [f for f in args.csv if Path(f).exists()]
        size_tag = "custom"
    elif args.large:
        csv_files = [f for f in ALL_CSVS if Path(f).exists()]
        size_tag = "large"
    else:
        csv_files = [f for f in SMALL_CSVS if Path(f).exists()]
        size_tag = "small"

    if not csv_files:
        print("Error: no CSV files found.")
        return
    print(f"[{size_tag}] Using {len(csv_files)} CSV file(s): "
          f"{[Path(f).name for f in csv_files]}\n")

    ablation_df = None
    sensitivity_df = None
    vel_meas_df = None

    # --- Ablation ---
    if not args.no_ablation:
        print("=" * 60)
        print("  ABLATION STUDY (8 feature on/off combinations)")
        print("=" * 60)
        ablation_df = run_ablation(csv_files)
        print(f"\nAblation complete: {len(ablation_df)} rows\n")

        if args.save:
            path = f"{args.save}_{size_tag}_ablation.csv"
            ablation_df.to_csv(path, index=False)
            print(f"Saved ablation results to {path}")

    # --- Small-set velocity comparison for requested measurement cases ---
    if size_tag == "small":
        print("=" * 60)
        print("  VELOCITY COMPARISON (full model; full vs no flow-sensor data)")
        print("=" * 60)
        vel_meas_df = run_velocity_measurement_comparison(csv_files)
        print(f"\nVelocity measurement comparison complete: {len(vel_meas_df)} rows\n")

        if args.save:
            path = f"{args.save}_{size_tag}_velocity_measurement_compare.csv"
            vel_meas_df.to_csv(path, index=False)
            print(f"Saved velocity measurement comparison to {path}")

    # --- Sensitivity ---
    if not args.no_sensitivity:
        print("=" * 60)
        print("  SENSITIVITY STUDY (parameter scaling sweeps)")
        print("=" * 60)
        sensitivity_df = run_sensitivity(csv_files)
        print(f"\nSensitivity complete: {len(sensitivity_df)} rows\n")

        if args.save:
            path = f"{args.save}_{size_tag}_sensitivity.csv"
            sensitivity_df.to_csv(path, index=False)
            print(f"Saved sensitivity results to {path}")

    # --- Print summary tables ---
    if ablation_df is not None and not ablation_df.empty:
        print("\n" + "=" * 80)
        print("  ABLATION SUMMARY (averaged across datasets)")
        print("=" * 80)
        summary_cols = ["case"]
        for c in ["pos_x", "pos_y", "pos_z", "pos_total",
                   "vel_x", "vel_y", "vel_z", "vel_total",
                   "att_roll", "att_pitch", "att_yaw"]:
            if c in ablation_df.columns:
                summary_cols.append(c)
        summary = ablation_df.groupby("case", sort=False)[summary_cols[1:]].mean()
        # Sort by vel_total if available
        if "vel_total" in summary.columns:
            summary = summary.sort_values("vel_total")
        pd.set_option("display.float_format", "{:.4f}".format)
        print(summary.to_string())
        print("=" * 80)

    # --- Plot ---
    save_dir = f"{args.save_plots}_{size_tag}" if args.save_plots else None
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)

    if ablation_df is not None and not ablation_df.empty:
        plot_ablation(ablation_df, save_dir=save_dir)
        plot_ablation_by_trajectory(ablation_df, save_dir=save_dir)
        plot_feature_importance(ablation_df, save_dir=save_dir)

    if sensitivity_df is not None and not sensitivity_df.empty:
        plot_sensitivity(sensitivity_df, save_dir=save_dir)
        plot_sensitivity_by_trajectory(sensitivity_df, save_dir=save_dir)

    if vel_meas_df is not None and not vel_meas_df.empty:
        plot_velocity_measurement_comparison(vel_meas_df, save_dir=save_dir)

    if save_dir:
        print(f"\nPlots saved to {save_dir}/")

    plt.show()


if __name__ == "__main__":
    main()
