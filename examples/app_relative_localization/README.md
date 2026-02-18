# Flapper Swarm App for Crazyflie/Flapper

This folder contains a unified swarm application that supports multiple drones with a single codebase.

## Overview

The drone ID is automatically derived from the **last nibble of the radio address** (same as `lpsTwrTag` uses for ranging). This ensures consistency between the app behavior and the UWB ranging ID.

For example:
- Radio address `0xE7E7E7E7E1` → droneId = 1
- Radio address `0xE7E7E7E7E2` → droneId = 2

| droneId | Role | Trigger | Avoidance Distance | Avoidance Yaw |
|---------|------|---------|-------------------|---------------|
| 1 | Primary drone | RC remote (`cppm.aux0`, active low) | `distance2` (to drone 2) | CW (+70°/s) |
| 2 | Secondary drone | UWB (`ranging.aux1`, active high) | `distance1` (to drone 1) | CCW (-70°/s) |

## Common Behavior (All Drones)

- Monitor `distance0` (beacon) for outer boundary emergency land
- Flight pattern: STRAIGHT → TURN → RECOVER
- Kill switch via `ranging.aux2` (drone 2+ only)

## Configuration

The drone ID is set automatically from the radio address. To configure a Crazyflie's address, use:

```bash
# Set radio address (last digit determines droneId)
cfclient --address 0xE7E7E7E7E1  # droneId = 1
cfclient --address 0xE7E7E7E7E2  # droneId = 2
```

Or use the Crazyflie client to configure the address in EEPROM.

Flight parameters can be tuned at runtime:

```python
cf.param.set_value('swarm.targetHeight', 1.2)
cf.param.set_value('swarm.fwdSpeed', 0.6)
cf.param.set_value('swarm.demoTime', 90000)  # 90 seconds
```

## Building

```bash
cd examples/app_flapper_swarm
make clean && make
```

## Flashing

```bash
cfloader flash build/flapper.bin stm32-fw -w radio://0/80/2M
```

See App layer API guide and build instructions [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/)
