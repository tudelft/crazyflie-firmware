# -*- coding: utf-8 -*-
#
# Set the mocap-tuned EKF parameters on a live Crazyflie/Flapper and persist
# them to EEPROM, so the drone actually flies with the tuned dynamics model.
#
# WHY THIS IS NEEDED: these params are PARAM_PERSISTENT. A stored EEPROM value
# overrides the firmware's compiled default at boot, and reflashing does NOT
# clear EEPROM. So even after flashing firmware that has the right defaults, the
# drone can keep flying stale persisted values. This script forces them.
#
# The values below match ekf_replay.py's EKFParams class defaults and the
# ekf_playground firmware compile defaults (mocap-tuned on the ekf_data branch).
#
# Usage:
#   python set_ekf_params.py                 # set + persist the tuned values
#   python set_ekf_params.py --dry-run       # only read current values, change nothing
#   python set_ekf_params.py --no-store      # set for this session only (not persisted)
#   python set_ekf_params.py --clear         # clear persistence -> firmware defaults take over
#   python set_ekf_params.py --uri radio://0/80/2M/E7E7E7E703

import argparse
import threading

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

DEFAULT_URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703')

# Mocap-tuned EKF values (ekf_data Optuna run). Keep in sync with
# ekf_replay.py EKFParams and the firmware compile defaults.
#   Note: dragBz (0.0611769) and drag_z are compile-time only (not params),
#   and the flow terms (flowStdX/Y=1.076/5.411, flowScale=2.3) already match,
#   so they are not set here.
TUNED = {
    'kalman.pNAcc_xy':          1.05006,
    'kalman.pNAcc_z':           0.604273,
    'kalman.mNGyro_rollpitch':  0.0521776,
    'kalman.mNGyro_yaw':        0.116742,
    'kalman.dragBx':            4.39468,
    'kalman.dragBy':            2.88896,
    'kalman.drag_rz':           0.03,
}

STORE_TOL = 1e-4


def _sync_persistent(fn, name, timeout=3.0):
    """Run an async persistent_store/persistent_clear synchronously."""
    ev = threading.Event()
    result = {'ok': False}

    def cb(complete_name, success):
        result['ok'] = bool(success)
        ev.set()

    fn(name, callback=cb)
    ev.wait(timeout)
    return result['ok']


def main():
    ap = argparse.ArgumentParser(description='Set + persist mocap-tuned EKF params.')
    ap.add_argument('--uri', default=DEFAULT_URI, help='Crazyflie URI')
    ap.add_argument('--dry-run', action='store_true',
                    help='only read current values, change nothing')
    ap.add_argument('--no-store', action='store_true',
                    help='set for this session only, do not persist to EEPROM')
    ap.add_argument('--clear', action='store_true',
                    help='clear persistence for these params so firmware defaults take over')
    args = ap.parse_args()

    cflib.crtp.init_drivers()
    print(f'Connecting to {args.uri} ...')
    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        print('Connected.\n')
        n_ok = 0
        for name, target in TUNED.items():
            try:
                before = float(cf.param.get_value(name))
            except Exception as e:
                print(f'  {name:26s} <read failed: {e}>')
                continue

            if args.dry_run:
                flag = 'ok' if abs(before - target) <= STORE_TOL else '<-- differs'
                print(f'  {name:26s} now={before:<12g} target={target:<12g} {flag}')
                continue

            if args.clear:
                cleared = _sync_persistent(cf.param.persistent_clear, name)
                after = float(cf.param.get_value(name))
                print(f'  {name:26s} cleared={cleared}  now={after:g} (firmware default)')
                n_ok += 1 if cleared else 0
                continue

            cf.param.set_value(name, target)
            stored = None
            if not args.no_store:
                stored = _sync_persistent(cf.param.persistent_store, name)
            after = float(cf.param.get_value(name))

            match = abs(after - target) <= STORE_TOL
            if args.no_store:
                tag = 'set (session only)'
            else:
                tag = 'SET + STORED' if stored else 'SET, STORE FAILED'
            mark = '' if match else '   <-- value mismatch!'
            print(f'  {name:26s} {before:<10g} -> {after:<10g} [{tag}]{mark}')
            n_ok += 1 if (match and (args.no_store or stored)) else 0

        print()
        if args.dry_run:
            print('Dry run only; nothing changed.')
        elif args.clear:
            print(f'Cleared persistence on {n_ok}/{len(TUNED)} params. '
                  'Reboot the drone so firmware defaults load.')
        else:
            print(f'{n_ok}/{len(TUNED)} params set'
                  + ('' if args.no_store else ' + persisted') + '.')
            if not args.no_store:
                print('Persisted values survive reboot. Re-run with --dry-run to verify.')


if __name__ == '__main__':
    main()
