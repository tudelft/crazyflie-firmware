# -*- coding: utf-8 -*-
"""
code to write usd logged crazyflie data to csv in format used for the snn pid
"""
import cfusdlog
import argparse
import pandas as pd
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--filename", type=str, default="data/bin/rlearning01")
parser.add_argument("--all", action="store_true", help="process all binary files in data/bin/")
parser.add_argument("--outdir", type=str, default="data/new/", help="output directory for csv files (default: same as input)")
args = parser.parse_args()

if args.all:
    bin_dir = Path("data/bin")
    filenames = [str(p) for p in sorted(bin_dir.iterdir()) if p.is_file() and p.suffix == ""]
else:
    filenames = [args.filename]


outdir = Path(args.outdir) if args.outdir else None
if outdir:
    outdir.mkdir(parents=True, exist_ok=True)


def process_file(filename):
    p = Path(filename)
    out = (outdir / p.name).with_suffix('.csv') if outdir else p.with_suffix('.csv')
    if out.exists():
        print(f"skipping {filename} (csv already exists)")
        return
    logData = cfusdlog.decode(filename)
    logData = logData['fixedFrequency']
    data = pd.DataFrame({key: logData[key] for key in logData})
    data.to_csv(out, index=False)
    print(f"wrote {out}")


for filename in filenames:
    process_file(filename)