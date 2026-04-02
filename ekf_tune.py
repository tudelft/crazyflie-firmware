#!/usr/bin/env python3
"""
Hyperparameter tuning for the Crazyflie EKF replay.

Uses Optuna (Bayesian TPE sampler) to search over the tunable EKF parameters
and minimise body-frame velocity RMSE against locSrv ground truth.

Usage:
    python ekf_tune.py                        # default: 200 trials, both CSVs
    python ekf_tune.py --n-trials 500
    python ekf_tune.py --csv flight_sine.csv   # single dataset only
    python ekf_tune.py --params drag_x drag_y flow_std_fixed  # tune subset
    python ekf_tune.py --show-best             # print best params & re-run with plot

Does NOT modify ekf_replay.py.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import os
import warnings
from pathlib import Path

import numpy as np
import optuna
from optuna.samplers import TPESampler

from ekf_replay import EKFParams, run_ekf

warnings.filterwarnings("ignore", category=FutureWarning)

# ---------------------------------------------------------------------------
# Objective
# ---------------------------------------------------------------------------

# CSV_FILES = ["flight_sine.csv", "flight_square.csv", "flight_square_yaw.csv"]
CSV_FILES = ["flight_diag_lr_mtf.csv", 
            #  "flight_diag_rl_mtf.csv", 
             "flight_sine_mtf.csv",
            #  "flight_square_mtf.csv",
            #  "flight_x_mtf.csv", 
             "flight_y_mtf.csv",
             "flight_z_mtf.csv",
             "square_60cm_mtf.csv",
            #  "square_80cm_mtf.csv",
            #  "square_100cm_mtf.csv",
             "square_120cm_mtf.csv"]



def _rmse(a, b) -> float:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        d = a - b
        return float(np.sqrt(np.nanmean(d * d)))


def _body_vel_rmse(results) -> dict[str, float]:
    """Compute body-frame velocity RMSE from a run_ekf result DataFrame."""
    return {
        "vx_b": _rmse(results["vx_b"].to_numpy(), results["ls_vx_b"].to_numpy()),
        "vy_b": _rmse(results["vy_b"].to_numpy(), results["ls_vy_b"].to_numpy()),
        "vz_b": _rmse(results["vz_b"].to_numpy(), results["ls_vz_b"].to_numpy()),
    }


def _pos_rmse(results) -> dict[str, float] | None:
    """Compute world-frame position RMSE. Returns None if locSrv not available."""
    if "ls_x" not in results.columns:
        return None
    return {
        "x": _rmse(results["x"].to_numpy(), results["ls_x"].to_numpy()),
        "y": _rmse(results["y"].to_numpy(), results["ls_y"].to_numpy()),
        "z": _rmse(results["z"].to_numpy(), results["ls_z"].to_numpy()),
    }


def _att_rmse(results) -> dict[str, float] | None:
    """Compute attitude RMSE in degrees. Returns None if locSrv attitude not available."""
    if "ls_roll" not in results.columns:
        return None
    return {
        "roll":  _rmse(results["roll"].to_numpy(),  results["ls_roll"].to_numpy()),
        "pitch": _rmse(results["pitch"].to_numpy(), results["ls_pitch"].to_numpy()),
        "yaw":   _rmse(results["yaw"].to_numpy(),   results["ls_yaw"].to_numpy()),
    }


# All tunable parameter definitions: name -> (low, high, log, default)
PARAM_SPACE: dict[str, tuple[float, float, bool, float]] = {
    # --- Process noise (how much EKF trusts IMU integration) ---
    "proc_noise_acc_xy": (0.5,    2.5,   True,  2.4),
    "proc_noise_acc_z":  (0.2,    2.5,   True,  0.8),
    # --- Gyro measurement noise (attitude correction gain) ---
    "meas_noise_gyro_rp":  (0.01,  0.2,  True,  0.034),
    "meas_noise_gyro_yaw": (0.02,  0.3,  True,  0.078),
    # --- Drag coefficients (body velocity prediction model) ---
    "drag_x": (1.5, 7.0, False, 4.2),
    "drag_y": (1.5, 7.0, False, 3.2),
    "drag_z": (0.05, 1.0, False, 0.3),
    # --- Sensor noise (how much EKF trusts flow & ToF) ---
    "flow_std_fixed_x": (0.5, 8.0, True, 2.0),
    "flow_std_fixed_y": (0.5, 24.0, True, 4.0),
    "tof_exp_std_a":  (0.0005, 0.02, True, 0.0025),
    # --- Flow sensor model ---
    "flow_resolution": (0.05, 0.5, False, 0.2),
    # --- Offsets (Z-component only; X/Y kept at 0) ---
    "cop_z":          (-0.05, 0.1, False, 0.03),
    "flowdeck_pos_z": (-0.25, 0.25, False, -0.12),
}

# Default subset when --params is not given (original 9 parameters)
DEFAULT_PARAMS = [
    "proc_noise_acc_xy", "proc_noise_acc_z",
    "meas_noise_gyro_rp", "meas_noise_gyro_yaw",
    "drag_x", "drag_y", "drag_z",
    "flow_std_fixed_x", "flow_std_fixed_y", #"tof_exp_std_a",
    "flow_resolution",
    # "cop_z", "flowdeck_pos_z",
]


def build_params(trial: optuna.Trial, active_params: list[str] | None = None) -> EKFParams:
    """Sample EKF parameters from the search space.

    Only the parameters listed in *active_params* are suggested by Optuna;
    all others stay at their defaults.  This allows tuning an arbitrary
    subset of the full parameter space.
    """
    if active_params is None:
        active_params = DEFAULT_PARAMS

    values: dict[str, float] = {}
    for name in PARAM_SPACE:
        lo, hi, use_log, default = PARAM_SPACE[name]
        if name in active_params:
            values[name] = trial.suggest_float(name, lo, hi, log=use_log)
        else:
            values[name] = default

    return _ekf_params_from_values(values)


def _ekf_params_from_values(d: dict[str, float]) -> EKFParams:
    """Construct EKFParams from a flat dict, filling missing keys with defaults."""
    defaults = {name: spec[3] for name, spec in PARAM_SPACE.items()}
    merged = {**defaults, **d}
    return EKFParams(
        proc_noise_acc_xy=merged["proc_noise_acc_xy"],
        proc_noise_acc_z=merged["proc_noise_acc_z"],
        meas_noise_gyro_rp=merged["meas_noise_gyro_rp"],
        meas_noise_gyro_yaw=merged["meas_noise_gyro_yaw"],
        drag_x=merged["drag_x"],
        drag_y=merged["drag_y"],
        drag_z=merged["drag_z"],
        flow_std_fixed_x=merged["flow_std_fixed_x"],
        flow_std_fixed_y=merged["flow_std_fixed_y"],
        tof_exp_std_a=merged["tof_exp_std_a"],
        flow_resolution=merged["flow_resolution"],

        cop=np.array([0.0, 0.0, merged["cop_z"]]),
        flowdeck_pos=np.array([0.0, 0.0, merged["flowdeck_pos_z"]]),
    )


def objective(trial: optuna.Trial, csv_files: list[str],
              active_params: list[str] | None = None,
              vel_weight: float = 1.0,
              pos_weight: float = 0.0,
              att_weight: float = 0.0) -> float:
    """Run EKF on each CSV and return weighted mean RMSE across enabled metrics.

    Weights:
      vel_weight — body-frame velocity (m/s per axis)
      pos_weight — world-frame position (m per axis)
      att_weight — attitude roll/pitch/yaw (deg per axis; scale ~1/57 to match metres)
    """
    params = build_params(trial, active_params=active_params)
    total_cost = 0.0
    n = 0
    for csv_path in csv_files:
        if not Path(csv_path).exists():
            continue
        try:
            with open(os.devnull, "w") as devnull, contextlib.redirect_stdout(devnull):
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    results = run_ekf(csv_path, params=params)
        except Exception:
            return float("inf")

        if "ls_vx_b" not in results.columns:
            return float("inf")

        cost = 0.0
        if vel_weight > 0.0:
            m = _body_vel_rmse(results)
            cost += vel_weight * (m["vx_b"] + m["vy_b"] + m["vz_b"])

        if pos_weight > 0.0:
            m = _pos_rmse(results)
            if m is not None:
                cost += pos_weight * (m["x"] + m["y"] + m["z"])

        if att_weight > 0.0:
            m = _att_rmse(results)
            if m is not None:
                cost += att_weight * (m["roll"] + m["pitch"] + m["yaw"])

        total_cost += cost
        n += 1

    if n == 0:
        return float("inf")

    return total_cost / n


# ---------------------------------------------------------------------------
# Pretty printing
# ---------------------------------------------------------------------------

def print_best(study: optuna.Study, csv_files: list[str]):
    """Print the best parameters and per-dataset RMSE breakdown."""
    best = study.best_trial
    print("\n" + "=" * 60)
    print(f"  BEST TRIAL #{best.number}  —  objective = {best.value:.4f}")
    print("=" * 60)

    # Print params grouped
    groups = {
        "Process noise": ["proc_noise_acc_xy", "proc_noise_acc_z"],
        "Meas noise":    ["meas_noise_gyro_rp", "meas_noise_gyro_yaw"],
        "Drag":          ["drag_x", "drag_y", "drag_z"],
        "Flow noise":    ["flow_std_fixed_x", "flow_std_fixed_y"],
        "Flow model":    ["flow_resolution"],
        "ToF":           ["tof_exp_std_a"],
        "Offsets":       ["cop_z", "flowdeck_pos_z"],
    }
    for group_name, keys in groups.items():
        print(f"\n  {group_name}:")
        for k in keys:
            if k in best.params:
                print(f"    {k:25s} = {best.params[k]:.6g}")

    # Per-dataset breakdown
    params = _ekf_params_from_values(best.params)
    print("\n  Per-dataset RMSE:")
    for csv_path in csv_files:
        if not Path(csv_path).exists():
            continue
        results = run_ekf(csv_path, params=params)
        vel = _body_vel_rmse(results)
        pos = _pos_rmse(results)
        att = _att_rmse(results)
        vel_str = f"vx_b={vel['vx_b']:.4f}  vy_b={vel['vy_b']:.4f}  vz_b={vel['vz_b']:.4f}"
        pos_str = (f"  |  x={pos['x']:.4f}  y={pos['y']:.4f}  z={pos['z']:.4f} m"
                   if pos is not None else "")
        att_str = (f"  |  roll={att['roll']:.3f}  pitch={att['pitch']:.3f}  yaw={att['yaw']:.3f} deg"
                   if att is not None else "")
        print(f"    {csv_path:30s}  {vel_str}{pos_str}{att_str}")
    print("=" * 60 + "\n")

    # Print as copy-pasteable EKFParams constructor
    print("Copy-paste for ekf_replay.py:\n")
    print("    params = EKFParams(")
    for k, v in best.params.items():
        if k in ("cop_z", "flowdeck_pos_z"):
            continue
        print(f"        {k}={v:.6g},")
    cop_z = best.params.get("cop_z", 0.03)
    fdp_z = best.params.get("flowdeck_pos_z", -0.12)
    print(f"        cop=np.array([0.0, 0.0, {cop_z:.6g}]),")
    print(f"        flowdeck_pos=np.array([0.0, 0.0, {fdp_z:.6g}]),")
    print("    )\n")


def build_params_from_dict(d: dict) -> EKFParams:
    """Reconstruct EKFParams from a flat parameter dict (e.g. best_trial.params)."""
    return _ekf_params_from_values(d)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Tune EKF replay hyperparameters")
    parser.add_argument("--n-trials", type=int, default=200,
                        help="Number of Optuna trials (default: 200)")
    parser.add_argument("--csv", type=str, nargs="+", default=CSV_FILES,
                        help="CSV file(s) to use for tuning")
    parser.add_argument("--params", type=str, nargs="+", default=None,
                        help="Subset of parameters to tune (default: all original 9). "
                             f"Choices: {', '.join(sorted(PARAM_SPACE.keys()))}")
    parser.add_argument("--study-name", type=str, default="ekf_tune",
                        help="Optuna study name (for DB storage)")
    parser.add_argument("--db", type=str, default=None,
                        help="Optuna storage URL, e.g. sqlite:///ekf_tune.db")
    parser.add_argument("--show-best", action="store_true",
                        help="Load existing study, print best, and run with plot")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--vel-weight", type=float, default=1.0,
                        help="Weight for body-frame velocity RMSE (m/s, default: 1.0)")
    parser.add_argument("--pos-weight", type=float, default=1.0,
                        help="Weight for world-frame position RMSE (m, default: 1.0)")
    parser.add_argument("--att-weight", type=float, default=0.01,
                        help="Weight for attitude RMSE (deg; default 0.01 ≈ deg→rad scale)")
    args = parser.parse_args()

    storage = args.db

    # Validate --params names
    active_params = args.params if args.params else DEFAULT_PARAMS
    unknown = set(active_params) - set(PARAM_SPACE)
    if unknown:
        parser.error(f"Unknown parameter(s): {unknown}. "
                     f"Valid: {', '.join(sorted(PARAM_SPACE.keys()))}")

    if args.show_best:
        study = optuna.load_study(study_name=args.study_name, storage=storage)
        print_best(study, args.csv)

        # Re-run best on each CSV with plots
        from ekf_replay import plot_results
        best_params = build_params_from_dict(study.best_trial.params)
        for csv_path in args.csv:
            if Path(csv_path).exists():
                print(f"\nRunning best params on {csv_path} ...")
                results = run_ekf(csv_path, params=best_params)
                plot_results(results)
        return

    # Suppress per-trial EKF prints
    optuna.logging.set_verbosity(optuna.logging.WARNING)

    sampler = TPESampler(seed=args.seed, n_startup_trials=10)
    study = optuna.create_study(
        study_name=args.study_name,
        storage=storage,
        direction="minimize",
        sampler=sampler,
        load_if_exists=True,
    )

    # Add current defaults as the first trial so we have a baseline
    default_dict = {name: PARAM_SPACE[name][3] for name in active_params}
    # Only enqueue if study is fresh
    if len(study.trials) == 0:
        study.enqueue_trial(default_dict)

    csv_files = [f for f in args.csv if Path(f).exists()]
    if not csv_files:
        print("Error: no CSV files found.")
        return

    print(f"Tuning EKF on: {csv_files}")
    print(f"Tuning parameters: {active_params}")
    print(f"Running {args.n_trials} trials (TPE sampler, seed={args.seed})")
    objective_parts = []
    if args.vel_weight > 0:
        objective_parts.append(f"vel×{args.vel_weight}")
    if args.pos_weight > 0:
        objective_parts.append(f"pos×{args.pos_weight}")
    if args.att_weight > 0:
        objective_parts.append(f"att×{args.att_weight}")
    print(f"Objective: {' + '.join(objective_parts)}\n")

    # Progress callback
    best_so_far = float("inf")
    def _callback(study, trial):
        nonlocal best_so_far
        val_str = f"{trial.value:.4f}" if trial.value is not None and np.isfinite(trial.value) else "FAILED"
        if trial.value is not None and trial.value < best_so_far:
            best_so_far = trial.value
            print(f"  Trial {trial.number:4d}  obj={val_str}  ★ new best")
        else:
            print(f"  Trial {trial.number:4d}  obj={val_str}")

    study.optimize(
        lambda trial: objective(trial, csv_files, active_params=active_params,
                                vel_weight=args.vel_weight,
                                pos_weight=args.pos_weight,
                                att_weight=args.att_weight),
        n_trials=args.n_trials,
        callbacks=[_callback],
        show_progress_bar=True,
    )

    print_best(study, csv_files)


if __name__ == "__main__":
    main()
