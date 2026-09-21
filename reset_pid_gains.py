# -*- coding: utf-8 -*-
#
# Clear the PERSISTED PID controller gains so they revert to the firmware
# compile-time defaults on the next reboot. This is the scripted version of
# clicking "Clear" for each gain in cfclient's persistent-parameter view.
#
# It ONLY touches the PID controller groups below -- it does NOT change the
# kalman/EKF params, the flapper trims, or anything else.
#
# Clearing removes the STORED (EEPROM) value; the live RAM value is unchanged
# until you REBOOT (power-cycle) the drone, at which point the firmware default
# loads because nothing is persisted to override it.
#
# Usage:
#   python reset_pid_gains.py            # clear persisted PID gains
#   python reset_pid_gains.py --dry-run  # list what would be cleared, change nothing
#   python reset_pid_gains.py --groups pid_attitude pid_rate
#   python reset_pid_gains.py --uri radio://0/80/2M/E7E7E7E703

import argparse
import threading

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

DEFAULT_URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703')

# PID controller gain groups only. The active controller is PID (stabilizer.
# controller=1), which uses these four. NOT kalman, NOT flapper trims, NOT the
# ctrlINDI/ctrlLee/ctrlMel groups (those belong to other, inactive controllers).
PID_GROUPS = ['pid_attitude', 'pid_rate', 'posCtlPid', 'velCtlPid']


def _sync_clear(cf, name, timeout=3.0):
    ev = threading.Event()
    result = {'ok': False}

    def cb(complete_name, success):
        result['ok'] = bool(success)
        ev.set()

    cf.param.persistent_clear(name, callback=cb)
    ev.wait(timeout)
    return result['ok']


def main():
    ap = argparse.ArgumentParser(
        description='Clear persisted PID gains -> firmware defaults on reboot.')
    ap.add_argument('--uri', default=DEFAULT_URI, help='Crazyflie URI')
    ap.add_argument('--groups', nargs='+', default=PID_GROUPS,
                    help='param groups to clear (default: the PID controller groups)')
    ap.add_argument('--dry-run', action='store_true',
                    help='list what would be cleared, change nothing')
    args = ap.parse_args()

    cflib.crtp.init_drivers()
    print(f'Connecting to {args.uri} ...')
    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        toc = cf.param.toc.toc
        print('Connected.\n')

        n_total = 0
        n_cleared = 0
        for group in args.groups:
            if group not in toc:
                print(f'[{group}] not present in this firmware -- skipping')
                continue
            print(f'[{group}]')
            for name in sorted(toc[group].keys()):
                full = f'{group}.{name}'
                n_total += 1
                if args.dry_run:
                    print(f'  would clear {full}')
                    continue
                ok = _sync_clear(cf, full)
                n_cleared += 1 if ok else 0
                print(f'  cleared {full}' if ok
                      else f'  {full}: nothing stored / no-op')

        print()
        if args.dry_run:
            print(f'Dry run: {n_total} params across {len(args.groups)} group(s).')
        else:
            print(f'Cleared persistence on {n_cleared}/{n_total} params.')
            print('>>> POWER-CYCLE the drone so the firmware default gains load. <<<')
            print('    (Clearing removes the stored value; the live value only '
                  'reverts on reboot.)')


if __name__ == '__main__':
    main()
