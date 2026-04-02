#!/usr/bin/env python3
"""
Set USD logging ON or OFF on all swarm drones.

Usage:
    python3 toggle_usd_logging.py on
    python3 toggle_usd_logging.py off
"""
import sys
import time
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

URIS = [
    'radio://0/80/2M/E7E7E7E700',
    'radio://0/80/2M/E7E7E7E701',
    # 'radio://0/80/2M/E7E7E7E702',
]

MAX_RETRIES = 5
RETRY_DELAY = 0.5


def set_logging(uri, value):
    try:
        with SyncCrazyflie(uri) as scf:
            cf = scf.cf

            for attempt in range(1, MAX_RETRIES + 1):
                cf.param.set_value('usd.logging', str(value))
                time.sleep(RETRY_DELAY)

                # Read back to verify
                actual = int(cf.param.get_value('usd.logging'))
                if actual == value:
                    state_str = 'ON' if value else 'OFF'
                    print(f'  {uri}: usd.logging = {value} ({state_str}) [verified, attempt {attempt}]')
                    return True
                else:
                    print(f'  {uri}: attempt {attempt} - set {value} but read back {actual}, retrying...')

            print(f'  {uri}: FAILED after {MAX_RETRIES} attempts! Value stuck at {actual}')
            return False
    except Exception as e:
        print(f'  {uri}: FAILED - {e}')
        return False


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ('on', 'off'):
        print('Usage: python3 toggle_usd_logging.py [on|off]')
        sys.exit(1)

    value = 1 if sys.argv[1] == 'on' else 0
    state_str = 'ON' if value else 'OFF'

    cflib.crtp.init_drivers(enable_debug_driver=False)
    print(f'Setting USD logging {state_str} on all drones...\n')

    results = {}
    for uri in URIS:
        results[uri] = set_logging(uri, value)

    print()
    failed = [uri for uri, ok in results.items() if not ok]
    if failed:
        print(f'WARNING: {len(failed)} drone(s) failed:')
        for uri in failed:
            print(f'  - {uri}')
        sys.exit(1)
    else:
        print('Done. All drones verified.')


if __name__ == '__main__':
    main()