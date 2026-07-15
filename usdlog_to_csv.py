#!/usr/bin/env python3
"""
Convert a Crazyflie uSD-card binary log file to a flat CSV.

For logs with a single event type (e.g. fixedFrequency), the output is a
direct translation.  For event-based logs (e.g. config_kalman.txt style),
all event streams are merged on timestamp using forward-fill so that every
row has a value for every column — matching the format expected by
ekf_replay.py.

Usage:
    python usdlog_to_csv.py log01
    python usdlog_to_csv.py log01 --output flight.csv
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

# cfusdlog lives next to the log files in tools/usdlog/
sys.path.insert(0, str(Path(__file__).parent / "tools" / "usdlog"))
import cfusdlog


def log_to_dataframe(log_data: dict) -> pd.DataFrame:
    """Convert the dict returned by cfusdlog.decode() to a single DataFrame."""
    event_names = list(log_data.keys())

    if len(event_names) == 1:
        # Single event type — straightforward conversion.
        name = event_names[0]
        df = pd.DataFrame(log_data[name])
        df = df.rename(columns={"timestamp": "timestamp"})
        return df.sort_values("timestamp").reset_index(drop=True)

    # Multiple event types — merge by timestamp with forward-fill so that
    # slower sensors (flow, ToF) carry their last value into every IMU row.
    frames = []
    for name in event_names:
        df = pd.DataFrame(log_data[name])
        df = df.set_index("timestamp")
        frames.append(df)

    # Outer join on timestamp index, then forward-fill gaps.
    merged = frames[0]
    for df in frames[1:]:
        merged = merged.join(df, how="outer", rsuffix="_dup")

    merged = merged.sort_index()
    merged = merged.ffill()

    # Drop rows that still have NaNs (i.e. the very first rows before all
    # streams have produced at least one sample).
    merged = merged.dropna()

    merged.index.name = "timestamp"
    return merged.reset_index()


def main():
    parser = argparse.ArgumentParser(
        description="Convert a Crazyflie uSD binary log to CSV."
    )
    parser.add_argument("filename", help="Binary log file (e.g. log01)")
    parser.add_argument(
        "--output", "-o",
        default=None,
        help="Output CSV file (default: <filename>.csv)",
    )
    args = parser.parse_args()

    input_path = Path(args.filename)
    output_path = Path(args.output) if args.output else input_path.with_suffix(".csv")

    print(f"Decoding {input_path} ...")
    log_data = cfusdlog.decode(str(input_path))

    if log_data is None:
        sys.exit("Failed to decode log file.")

    print(f"Found event types: {list(log_data.keys())}")

    df = log_to_dataframe(log_data)

    print(f"Writing {len(df)} rows × {len(df.columns)} columns to {output_path}")
    print(f"Columns: {list(df.columns)}")
    df.to_csv(output_path, index=False)
    print("Done.")


if __name__ == "__main__":
    main()
