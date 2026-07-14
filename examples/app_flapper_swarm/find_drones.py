#!/usr/bin/env python3
"""Scan the Crazyradio for live drones and print each one's droneId.

Used after `make cload-id` to identify a freshly-flashed drone whose
radio address you didn't know going in. droneId is the last nibble of
the radio address (matches the selfID convention in flapper_swarm.c
and lpsTwrTag.c).

Override defaults via env if needed:
  CFLIB_SCAN_BOOT_DELAY   seconds to wait for the drone to boot (default 3)
"""
import os
import sys
import time

try:
    import cflib.crtp
except ImportError:
    print("[find_drones] cflib not installed - skipping ID lookup.")
    print("[find_drones] Install with: pip install cflib")
    sys.exit(0)

cflib.crtp.init_drivers(enable_debug_driver=False)

# Give the drone a few seconds to finish boot + sensor calibration before
# probing — otherwise it may not respond to scan packets yet.
boot_delay = float(os.environ.get('CFLIB_SCAN_BOOT_DELAY', '3'))
time.sleep(boot_delay)

# Addresses to probe:
#   - 0xE7E7E7E7E7: Bitcraze factory default (brand-new drones)
#   - 0xE7E7E7E700..0F: standard swarm range we use in this project
addresses = [0xE7E7E7E7E7] + [0xE7E7E7E700 + i for i in range(16)]

print("\n>>> Scanning radio for drones...")
found = set()
for addr in addresses:
    try:
        for uri, _comment in cflib.crtp.scan_interfaces(address=addr):
            found.add(uri)
    except Exception:
        # Individual address failures are fine — keep scanning the rest.
        pass

if not found:
    print(">>> No drones found.")
    print(">>> If you just flashed one, give it ~5s to boot and re-run `make scan-drones`.")
    sys.exit(0)

print(">>> Found:")
for uri in sorted(found):
    # URI format: radio://0/<channel>/<rate>/<addr_hex>
    addr_hex = uri.split('/')[-1]
    nibble = addr_hex[-1]
    try:
        drone_id = int(nibble, 16)
        # Special-case: factory default 0xE7E7E7E7E7 has nibble 'E' = 14 which
        # isn't a real droneId — call it out distinctly.
        if addr_hex.upper() == 'E7E7E7E7E7':
            print(f"    [factory default, droneId not configured yet]  URI={uri}")
        else:
            print(f"    droneId={drone_id}  URI={uri}")
    except ValueError:
        print(f"    URI={uri}")

print()
print("    Plug the URI into cfclient (Connect > Select interface) to connect.")
