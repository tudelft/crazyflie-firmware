# -*- coding: utf-8 -*-
#
# Read-only dump of the estimator-relevant parameter values from a live
# Crazyflie/Flapper, printed to the terminal. Use it to verify the onboard
# config matches a known-good experiment (flow std devs, kalman process/measure
# noise, drag/lever-arm terms, initial state, estimator/controller selection).
#
# This ONLY reads parameters -- it never writes, arms, or flies anything.
#
# Usage:
#   python dump_params.py                       # default URI (see below)
#   python dump_params.py --uri radio://0/80/2M/E7E7E7E703
#   python dump_params.py --all                 # dump every param group
#   python dump_params.py --groups kalman mtf02 locSrv
#
# To flag drift from a reference, fill in REFERENCE below with the values from
# your known-good experiment; matching params print "ok", differing ones print
# the expected value and a "<-- MISMATCH" marker.

import argparse

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Same default as nocap.py so it just works on your usual drone.
DEFAULT_URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703')

# Parameter groups considered "estimator relevant" by default. --all overrides.
DEFAULT_GROUPS = [
    'kalman',       # process/measurement noise, drag, lever arm, initial state
    'mtf02',        # optical-flow calibration: flowStdX/Y, flowScale, range std
    'locSrv',       # external-pose fusion (enExtPoseFuse, extQuatStdDev)
    'stabilizer',   # estimator / controller selection
    'flapper',      # trims + max thrust (affect hover attitude bias)
]

# Optional: expected values from a prior experiment, keyed by full param name.
# Leave empty to just print. Example:
#   REFERENCE = {
#       'mtf02.flowStdX': 1.07615,
#       'mtf02.flowStdY': 5.41112,
#       'mtf02.flowScale': 2.3,
#       'kalman.pNVel': 0.5,
#   }
REFERENCE: dict[str, float] = {}

# How close a float must be to its reference to count as a match.
REFERENCE_TOL = 1e-4


def _fmt_value(v: str) -> str:
    """Pretty-print a value string: trim float noise, keep ints/enums as-is."""
    try:
        f = float(v)
    except (TypeError, ValueError):
        return str(v)
    if f == int(f):
        return str(int(f))
    return f'{f:.6g}'


def _matches_reference(full_name: str, value: str):
    """Return (has_ref, is_match, expected_str) for the reference comparison."""
    if full_name not in REFERENCE:
        return False, None, None
    expected = REFERENCE[full_name]
    try:
        is_match = abs(float(value) - float(expected)) <= REFERENCE_TOL
    except (TypeError, ValueError):
        is_match = str(value) == str(expected)
    return True, is_match, _fmt_value(str(expected))


def dump(cf, groups, dump_all):
    toc = cf.param.toc.toc  # {group: {name: TocElement}}
    group_names = sorted(toc.keys()) if dump_all else groups

    total = 0
    mismatches = 0
    for group in group_names:
        if group not in toc:
            print(f'\n[{group}]  (not present in this firmware)')
            continue

        names = sorted(toc[group].keys())
        print(f'\n[{group}]  ({len(names)} params)')
        for name in names:
            full = f'{group}.{name}'
            element = toc[group][name]
            try:
                access = element.get_readable_access()
            except Exception:
                access = '??'
            try:
                raw = cf.param.get_value(full)
                value = _fmt_value(raw)
            except Exception as e:
                value = f'<read failed: {e}>'
                raw = None

            has_ref, is_match, expected = _matches_reference(full, raw)
            suffix = ''
            if has_ref:
                if is_match:
                    suffix = '   ok'
                else:
                    suffix = f'   <-- MISMATCH (expected {expected})'
                    mismatches += 1

            print(f'  {name:<22} {access:<3} {value:>14}{suffix}')
            total += 1

    print(f'\nRead {total} params from {len(group_names)} group(s).')
    if REFERENCE:
        print(f'Reference check: {len(REFERENCE)} expected, {mismatches} mismatch(es).')


def main():
    ap = argparse.ArgumentParser(description='Read-only dump of Crazyflie params.')
    ap.add_argument('--uri', default=DEFAULT_URI, help='Crazyflie URI')
    ap.add_argument('--all', action='store_true', help='dump every param group')
    ap.add_argument('--groups', nargs='+', default=DEFAULT_GROUPS,
                    help='param groups to dump (default: estimator-relevant)')
    args = ap.parse_args()

    cflib.crtp.init_drivers()
    print(f'Connecting to {args.uri} ...')
    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        print('Connected. Reading parameters (read-only)...')
        dump(scf.cf, args.groups, args.all)


if __name__ == '__main__':
    main()
