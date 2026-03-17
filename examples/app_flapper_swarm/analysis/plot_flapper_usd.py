#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
USDLOG_DIR = HERE.parents[2] / "tools" / "usdlog"
sys.path.insert(0, str(USDLOG_DIR))

import argparse
import cfusdlog
import matplotlib.pyplot as plt
import pandas as pd

"""
Decode Crazyflie uSD log, save fixedFrequency data to CSV,
and plot flapper swarm / EKF related variables.

Usage:
    python3 plot_flapper_usd.py flaplog00
    python3 plot_flapper_usd.py flaplog00 --csv mylog.csv

Notes:
- Assumes cfusdlog.decode(...) returns a dict with an event named 'fixedFrequency'
- Saves all decoded fixedFrequency variables to CSV
- Plots only variables that actually exist in the file
"""

import argparse
from pathlib import Path

import cfusdlog
import matplotlib.pyplot as plt
import pandas as pd


def build_dataframe(log_data: dict) -> pd.DataFrame:
    """
    Convert decoded fixedFrequency log dict to pandas DataFrame.
    """
    df = pd.DataFrame(log_data)

    # Make a nicer time column if timestamp exists
    if "timestamp" in df.columns:
        # Crazyflie timestamps are typically in ms
        df["t_s"] = (df["timestamp"] - df["timestamp"].iloc[0]) / 1000.0

    return df


def save_csv(df: pd.DataFrame, csv_path: Path) -> None:
    df.to_csv(csv_path, index=False)
    print(f"Saved CSV: {csv_path}")


def existing(df: pd.DataFrame, cols: list[str]) -> list[str]:
    return [c for c in cols if c in df.columns]


def plot_group(df: pd.DataFrame, cols: list[str], title: str, ylabel: str,
               convert=None, labels=None) -> None:
    cols_present = existing(df, cols)
    if not cols_present:
        print(f"Skipping plot '{title}' because no matching variables were found.")
        return

    x = df["t_s"] if "t_s" in df.columns else df.index

    plt.figure(figsize=(12, 6))
    for c in cols_present:
        y = df[c]
        if convert is not None:
            y = convert(c, y)
        label = labels[c] if labels and c in labels else c
        plt.plot(x, y, label=label)

    plt.xlabel("Time [s]" if "t_s" in df.columns else "Sample")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", help="Decoded uSD binary log file, e.g. flaplog00")
    parser.add_argument(
        "--csv",
        help="Optional CSV output filename. Default: same stem with .csv",
        default=None,
    )
    args = parser.parse_args()

    log_all = cfusdlog.decode(args.filename)

    if "fixedFrequency" not in log_all:
        raise KeyError(
            f"'fixedFrequency' not found in decoded log. Available keys: {list(log_all.keys())}"
        )

    log_data = log_all["fixedFrequency"]
    df = build_dataframe(log_data)

    print("\nDecoded columns:")
    for c in df.columns:
        print(f"  {c}")

    # Save CSV
    input_path = Path(args.filename)
    csv_path = Path(args.csv) if args.csv else input_path.with_suffix(".csv")
    save_csv(df, csv_path)

    # --------------------------------------------------
    # Plot 1: self state estimates
    # stateEstimateZ.vx/vy/vz are mm/s
    # stateEstimateZ.rateYaw is mrad/s
    # --------------------------------------------------
    plot_group(
        df,
        cols=[
            "stateEstimateZ.vx",
            "stateEstimateZ.vy",
            "stateEstimateZ.vz",
        ],
        title="Self velocity estimate",
        ylabel="Velocity [m/s]",
        convert=lambda c, y: y / 1000.0,
        labels={
            "stateEstimateZ.vx": "self vx",
            "stateEstimateZ.vy": "self vy",
            "stateEstimateZ.vz": "self vz",
        },
    )

    plot_group(
        df,
        cols=["stateEstimateZ.rateYaw"],
        title="Self yaw rate estimate",
        ylabel="Yaw rate [rad/s]",
        convert=lambda c, y: y / 1000.0,
        labels={"stateEstimateZ.rateYaw": "self yaw rate"},
    )

    # --------------------------------------------------
    # Plot 2: direct ranges
    # --------------------------------------------------
    plot_group(
        df,
        cols=[
            "ranging.distance0",
            "ranging.distance1",
            "ranging.distance2",
        ],
        title="Direct ranges",
        ylabel="Range [mm]",
    )

    # --------------------------------------------------
    # Plot 3: indirect ranges
    # --------------------------------------------------
    plot_group(
        df,
        cols=[
            "ranging.inD01",
            "ranging.inD02",
            "ranging.inD10",
            "ranging.inD12",
            "ranging.inD20",
            "ranging.inD21",
        ],
        title="Indirect ranges",
        ylabel="Range [mm]",
    )

    # --------------------------------------------------
    # Plot 4: shared peer velocities
    # --------------------------------------------------
    plot_group(
        df,
        cols=["ranging.vx0", "ranging.vx1", "ranging.vx2"],
        title="Shared vx values",
        ylabel="vx [m/s or firmware units]",
    )

    plot_group(
        df,
        cols=["ranging.vy0", "ranging.vy1", "ranging.vy2"],
        title="Shared vy values",
        ylabel="vy [m/s or firmware units]",
    )

    plot_group(
        df,
        cols=["ranging.vz0", "ranging.vz1", "ranging.vz2"],
        title="Shared vz values",
        ylabel="vz [m/s or firmware units]",
    )

    plot_group(
        df,
        cols=["ranging.yawR0", "ranging.yawR1", "ranging.yawR2"],
        title="Shared yaw rate values",
        ylabel="Yaw rate [rad/s or firmware units]",
    )

    # --------------------------------------------------
    # Plot 5: heights
    # --------------------------------------------------
    plot_group(
        df,
        cols=["ranging.height0", "ranging.height1", "ranging.height2"],
        title="Shared heights",
        ylabel="Height [m or firmware units]",
    )

    # --------------------------------------------------
    # Plot 6: EKF relative states
    # --------------------------------------------------
    plot_group(
        df,
        cols=["relLoc.x0", "relLoc.x1", "relLoc.x2"],
        title="Relative EKF x states",
        ylabel="x [m]",
    )

    plot_group(
        df,
        cols=["relLoc.y0", "relLoc.y1", "relLoc.y2"],
        title="Relative EKF y states",
        ylabel="y [m]",
    )

    plot_group(
        df,
        cols=["relLoc.z0", "relLoc.z1", "relLoc.z2"],
        title="Relative EKF z states",
        ylabel="z [m]",
    )

    plot_group(
        df,
        cols=["relLoc.psi0", "relLoc.psi1", "relLoc.psi2"],
        title="Relative EKF yaw states",
        ylabel="Yaw [rad]",
    )

    plt.show()


if __name__ == "__main__":
    main()