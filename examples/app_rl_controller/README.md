# RL Controller App for Flapper Platform

A simple app for the Crazyflie Flapper platform that provides a framework for testing reinforcement learning (RL) controllers.

## Features

- **Takeoff and Hover**: Press aux0 switch to take off to a configurable altitude (default 1.3 m) and hover stably
- **RL Controller Toggle**: Press aux1 switch to switch between hover control and the RL controller
- **Safe Landing**: Press aux0 again at any time to land
- **Configurable Target Altitude**: The target altitude can be adjusted via parameters

## How to Use

### Building the App

```bash
cd examples/app_rl_controller
make
```

### Flashing to Device

```bash
make cload
```

## Operation

### Flight States

1. **IDLE**: Initial state. Waiting for takeoff command.
   - Press **aux0** to transition to HOVERING state

2. **HOVERING**: Actively hovering at the target altitude.
   - Press **aux1** to switch to RL controller
   - Press **aux0** again to land and return to IDLE

3. **RL_CONTROL**: Using the RL controller (currently a stand-in implementation).
   - Press **aux1** to return to hover control at current altitude
   - Press **aux0** to land and return to IDLE

### RC Controller Mapping (Aux Switches)

- **aux0**: Takeoff/Land trigger (active low)
- **aux1**: RL controller toggle (active low)

Active low means the signal is considered active when the RC value is below ~1400.

## Configuration

The following parameters can be adjusted in real-time via the parameter interface:

- **rlapp.targetAltitude**: Target takeoff altitude in meters (default: 1.3 m)

## Implementation Details

### Current State

- The app implements a simple state machine with three states: IDLE, HOVERING, and RL_CONTROL
- Hover and RL control states use velocity setpoints for stable altitude control
- The RL controller is currently a stand-in that maintains hover at the current altitude

### Extending the App

To implement the actual RL controller:

1. Replace the `rlControllerCompute()` function with the actual RL algorithm
2. You can access the current state estimate via log variables:
   - `stateEstimate.z`: Current Z position
   - `stateEstimate.x`, `stateEstimate.y`: Current position
   - `stateEstimate.vx`, `stateEstimate.vy`, `stateEstimate.vz`: Current velocities
   - `stateEstimate.yaw`: Current yaw angle
   - Additional sensor data via other log groups
3. Send control commands using `commanderSetSetpoint()` with appropriate setpoint structures

### Log Variables

The app logs the following information:

- **rlapp.state**: Current state (0=IDLE, 1=HOVERING, 2=RL_CONTROL)
- **rlapp.hoverAltitude**: Altitude being maintained in hover/RL control states

## Notes

- The app uses the same aux channel convention as the main flapper swarm app (active low)
- All altitudes are in absolute world coordinates (meters above ground)
- The RL controller receives sensor data via the logging system but can be extended to use other data sources
