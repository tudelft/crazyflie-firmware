#!/usr/bin/env python3
"""
Launch/stop the flapper swarm demo via cflib (no RC remote needed).

Requires swarm.launchMode=1 (persistent, set once via cfclient or this script).

Usage:
    python3 launch_swarm.py start          # Enable logging + trigger flight
    python3 launch_swarm.py stop           # Disable logging (flight ends via demoTime)
    python3 launch_swarm.py mode-script    # Set launchMode=1 (script) on all drones
    python3 launch_swarm.py mode-rc        # Set launchMode=0 (RC) on all drones
"""
import sys
import time
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# Drone URIs — cycle order: drone 2, drone 0 (beacon), drone 1
URIS = {
    2: 'radio://0/80/2M/E7E7E7E702',
    0: 'radio://0/80/2M/E7E7E7E700',
    1: 'radio://0/80/2M/E7E7E7E701',
}
CYCLE_ORDER = [2, 0, 1]

MAX_RETRIES = 5
RETRY_DELAY = 0.5


def set_param_verified(cf, full_name, value, retries=MAX_RETRIES):
    """Set a param and read back to verify."""
    for attempt in range(1, retries + 1):
        cf.param.set_value(full_name, str(value))
        time.sleep(RETRY_DELAY)
        actual = cf.param.get_value(full_name)
        if str(actual).strip() == str(value).strip():
            return True
        print(f'    retry {attempt}: {full_name}={value} but read {actual}')
    return False


def configure_drone(drone_id, usd_logging=None, script_trig=None, launch_mode=None):
    """Connect to a drone and set the requested params."""
    uri = URIS[drone_id]
    label = f'Drone {drone_id} ({uri})'
    try:
        with SyncCrazyflie(uri) as scf:
            cf = scf.cf

            if launch_mode is not None:
                ok = set_param_verified(cf, 'swarm.launchMode', launch_mode)
                print(f'  {label}: swarm.launchMode = {launch_mode} [{"OK" if ok else "FAILED"}]')

            if usd_logging is not None:
                ok = set_param_verified(cf, 'usd.logging', usd_logging)
                state = 'ON' if usd_logging else 'OFF'
                print(f'  {label}: usd.logging = {usd_logging} ({state}) [{"OK" if ok else "FAILED"}]')

            if script_trig is not None:
                if script_trig == 1:
                    # Brief 0 first to guarantee a rising edge in the app
                    cf.param.set_value('swarm.scriptTrig', '0')
                    time.sleep(0.3)
                ok = set_param_verified(cf, 'swarm.scriptTrig', script_trig)
                print(f'  {label}: swarm.scriptTrig = {script_trig} [{"OK" if ok else "FAILED"}]')

            return True
    except Exception as e:
        print(f'  {label}: FAILED — {e}')
        return False


def main():
    usage = (
        'Usage: python3 launch_swarm.py [start|stop|mode-script|mode-rc]\n'
        '  start        — enable USD logging + trigger flight on all drones\n'
        '  stop         — disable USD logging (flight ends via demoTime)\n'
        '  mode-script  — set launchMode=1 (script trigger) on all drones\n'
        '  mode-rc      — set launchMode=0 (RC remote) on all drones'
    )

    if len(sys.argv) != 2 or sys.argv[1] not in ('start', 'stop', 'mode-script', 'mode-rc'):
        print(usage)
        sys.exit(1)

    cmd = sys.argv[1]
    cflib.crtp.init_drivers(enable_debug_driver=False)

    if cmd == 'start':
        print('=== Enabling USD logging + triggering flight ===\n')
        for drone_id in CYCLE_ORDER:
            if drone_id == 0:
                # Beacon: logging only, no flight trigger
                configure_drone(drone_id, usd_logging=1)
            else:
                # Flying drones: logging + trigger
                configure_drone(drone_id, usd_logging=1, script_trig=1)

    elif cmd == 'stop':
        print('=== Disabling USD logging ===\n')
        for drone_id in CYCLE_ORDER:
            if drone_id == 0:
                configure_drone(drone_id, usd_logging=0)
            else:
                configure_drone(drone_id, usd_logging=0, script_trig=0)

    elif cmd == 'mode-script':
        print('=== Setting launchMode = 1 (script) on all drones ===\n')
        for drone_id in CYCLE_ORDER:
            configure_drone(drone_id, launch_mode=1)

    elif cmd == 'mode-rc':
        print('=== Setting launchMode = 0 (RC) on all drones ===\n')
        for drone_id in CYCLE_ORDER:
            configure_drone(drone_id, launch_mode=0)

    print('\nDone.')


if __name__ == '__main__':
    main()
