#!/usr/bin/env bash
#
# Batch-convert Crazyflie uSD binary logs to CSV.
#
# Converts every "logN" file in LOG_DIR to "flightN.csv" in OUT_DIR by calling
# usdlog_to_csv.py, and also copies the raw binary alongside it as "flightN.bin"
# so each flight is self-contained (CSV + raw) in OUT_DIR. Already-processed
# logs are skipped, so re-running after collecting more flights only processes
# the new ones.
#
# Usage:
#   ./convert_logs.sh [LOG_DIR] [OUT_DIR] [--force]
#
# Examples:
#   ./convert_logs.sh /media/austin/6232-3761            # -> ./flightN.csv
#   ./convert_logs.sh /media/austin/6232-3761 ./csv      # -> ./csv/flightN.csv
#   ./convert_logs.sh /media/austin/6232-3761 . --force  # reconvert everything
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONVERTER="$SCRIPT_DIR/usdlog_to_csv.py"

# --- parse args (positional LOG_DIR, OUT_DIR; --force anywhere) ---
FORCE=0
positional=()
for arg in "$@"; do
  if [[ "$arg" == "--force" ]]; then
    FORCE=1
  else
    positional+=("$arg")
  fi
done
LOG_DIR="${positional[0]:-/media/austin/6232-3761}"
OUT_DIR="${positional[1]:-.}"

if [[ ! -d "$LOG_DIR" ]]; then
  echo "error: log directory not found: $LOG_DIR" >&2
  echo "       (is the SD card mounted?)" >&2
  exit 1
fi
if [[ ! -f "$CONVERTER" ]]; then
  echo "error: converter not found: $CONVERTER" >&2
  exit 1
fi

mkdir -p "$OUT_DIR"

converted=0
copied=0
skipped=0
failed=0

shopt -s nullglob
# Numeric-version sort so log2 comes before log10.
mapfile -t logs < <(printf '%s\n' "$LOG_DIR"/log[0-9]* | sort -V)

if [[ ${#logs[@]} -eq 0 ]]; then
  echo "No 'logN' files found in $LOG_DIR"
  exit 0
fi

for log in "${logs[@]}"; do
  [[ -f "$log" ]] || continue
  base="$(basename "$log")"        # e.g. log15
  num="${base#log}"                # e.g. 15
  out="$OUT_DIR/flight${num}.csv"
  raw="$OUT_DIR/flight${num}.bin"  # raw binary copy, paired with the CSV

  # --- CSV conversion ---
  if [[ -f "$out" && "$FORCE" -ne 1 ]]; then
    echo "skip  $base -> flight${num}.csv (exists)"
    skipped=$((skipped + 1))
  else
    echo "conv  $base -> flight${num}.csv"
    if python3 "$CONVERTER" "$log" -o "$out"; then
      converted=$((converted + 1))
    else
      echo "FAIL  $base (see message above)" >&2
      rm -f "$out"                 # don't leave a half-written CSV behind
      failed=$((failed + 1))
    fi
  fi

  # --- raw binary backup, next to the CSV ---
  if [[ -f "$raw" && "$FORCE" -ne 1 ]]; then
    echo "skip  $base -> flight${num}.bin (exists)"
  else
    echo "copy  $base -> flight${num}.bin"
    cp -p "$log" "$raw"
    copied=$((copied + 1))
  fi
done

echo "-----"
echo "done: $converted converted, $copied copied, $skipped skipped, $failed failed"
[[ "$failed" -eq 0 ]]
