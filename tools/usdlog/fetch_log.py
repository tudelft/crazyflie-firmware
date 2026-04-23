#!/usr/bin/env python3
"""
Decode the latest SD-card log into a CSV under tools/usdlog/ and optionally plot it.

Default: find the highest-numbered logXX on the crazyflie card, decode it, and
write logXX.csv next to this script. Pass --plot to also run plot_rl on it.
"""
import argparse
import getpass
import os
import re
import sys

import cfusdlog
import pandas as pd

LOG_RE = re.compile(r"^log(\d+)$")
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def find_sd_mount(label="crazyflie"):
    user = getpass.getuser()
    for base in (f"/media/{user}", f"/run/media/{user}", "/media", "/mnt"):
        candidate = os.path.join(base, label)
        if os.path.isdir(candidate):
            return candidate
    return None


def list_logs(sd_path):
    logs = []
    for name in os.listdir(sd_path):
        m = LOG_RE.match(name)
        if m and os.path.isfile(os.path.join(sd_path, name)):
            logs.append((int(m.group(1)), name))
    logs.sort()
    return logs


def decode_to_csv(binary_path, csv_path):
    decoded = cfusdlog.decode(binary_path)
    if "fixedFrequency" not in decoded:
        raise RuntimeError(f"No 'fixedFrequency' group in {binary_path}")
    fixed = decoded["fixedFrequency"]
    df = pd.DataFrame({k: fixed[k] for k in fixed})
    df.to_csv(csv_path, index=False)
    return len(df)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sd-label", default="crazyflie",
                    help="Volume label of the SD card (default: crazyflie)")
    ap.add_argument("--sd-mount", default=None,
                    help="Override mount path (skips label lookup)")
    ap.add_argument("--log", default=None,
                    help="Specific log name to fetch (e.g. log15). Default: highest-numbered.")
    ap.add_argument("--output-dir", default=SCRIPT_DIR,
                    help=f"Where to write the CSV (default: {SCRIPT_DIR})")
    ap.add_argument("--plot", action="store_true",
                    help="After writing the CSV, run plot_rl on it")
    ap.add_argument("--overwrite", action="store_true",
                    help="Overwrite an existing CSV without prompting")
    args = ap.parse_args()

    sd = args.sd_mount or find_sd_mount(args.sd_label)
    if not sd or not os.path.isdir(sd):
        print(f"ERROR: SD card not found (label={args.sd_label}). "
              f"Mount it or pass --sd-mount /path/to/card", file=sys.stderr)
        sys.exit(1)

    logs = list_logs(sd)
    if not logs:
        print(f"ERROR: no logXX files found on {sd}", file=sys.stderr)
        sys.exit(1)

    if args.log:
        m = LOG_RE.match(args.log)
        if not m:
            print(f"ERROR: --log must look like 'logXX', got '{args.log}'",
                  file=sys.stderr)
            sys.exit(1)
        target_name = args.log
        binary_path = os.path.join(sd, target_name)
        if not os.path.isfile(binary_path):
            print(f"ERROR: {binary_path} not found on SD card", file=sys.stderr)
            sys.exit(1)
    else:
        _, target_name = logs[-1]
        binary_path = os.path.join(sd, target_name)

    csv_path = os.path.join(args.output_dir, f"{target_name}.csv")
    if os.path.exists(csv_path) and not args.overwrite:
        reply = input(f"{csv_path} exists. Overwrite? [y/N] ").strip().lower()
        if reply != "y":
            print("Aborted.")
            sys.exit(0)

    print(f"Decoding {binary_path} -> {csv_path}")
    n_rows = decode_to_csv(binary_path, csv_path)
    print(f"Wrote {n_rows} rows.")

    if args.plot:
        from plot_rl import plot_rl_log
        plot_rl_log(csv_path)


if __name__ == "__main__":
    main()
