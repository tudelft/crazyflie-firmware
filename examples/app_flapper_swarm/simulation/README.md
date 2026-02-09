# Flapper Swarm Simulation

A 2D Python simulation for testing and tuning the drone swarm algorithm used in `flapper_swarm.c`.

## Installation

```bash
pip install pygame
```

## Usage

```bash
cd examples/app_flapper_swarm/simulation
python run_sim.py
```

## Controls

| Key   | Action          |
|-------|-----------------|
| SPACE | Pause/Resume    |
| R     | Reset simulation|
| ESC   | Quit            |

## Configuration

**All tunable parameters are in `config.py`** - edit `default_config` at the bottom of that file.

The configuration is organized into sections:
- **Flight parameters** - Algorithm parameters (speeds, distances, thresholds)
- **Simulation parameters** - Time step, derivative buffer settings
- **Visualization parameters** - Window size, colors, trails
- **Per-drone configurations** - Initial position and noise settings for each drone

### Per-Drone Noise

Each drone has its own noise configuration, allowing you to model different real-world characteristics:

```python
drone1=DroneConfig(
    initial=DroneInitialState(x=0.8, y=0.0, yaw=270.0),
    noise=DroneNoiseParams(
        enable_process_noise=True,
        enable_sensor_noise=True,
        process_vx_std=0.02,
        process_vy_std=0.05,
        process_vy_bias=0.0,  # Set to non-zero for drift
        uwb_distance_std=0.05,
    ),
),
```

## Noise Models

### Process Noise (Velocity Tracking Errors)

Models imperfect velocity control due to state estimation errors:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_process_noise` | `True` | Enable/disable process noise |
| `process_vx_std` | `0.02` m/s | Forward velocity noise std dev |
| `process_vy_std` | `0.05` m/s | Lateral velocity noise std dev (typically larger) |
| `process_yaw_rate_std` | `2.0` deg/s | Yaw rate noise std dev |
| `process_vy_bias` | `0.0` m/s | Constant lateral drift |

### Sensor Noise (UWB Distance Measurements)

Models UWB ranging measurement noise:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_sensor_noise` | `True` | Enable/disable sensor noise |
| `uwb_distance_std` | `0.05` m | Distance measurement noise std dev |
| `uwb_distance_bias` | `0.0` m | Systematic measurement bias |

## Architecture

The simulation is modular for easy extension:

- **`config.py`** - **Single source of truth** for all tunable parameters
- **`drone.py`** - Drone physics (`KinematicPhysics`, `NoisyKinematicPhysics`, `UWBSensor`)
- **`controller.py`** - Flight control algorithm (swappable)
- **`simulator.py`** - Main loop and Pygame visualization
- **`run_sim.py`** - Entry point (just runs with default config)

### Swapping the Physics Model

The drone uses a `PhysicsModel` protocol. To add dynamics:

```python
from simulation.drone import Drone, DronePhysicsState

class MyDynamicPhysics:
    def update(self, state, cmd_vx, cmd_vy, cmd_yaw_rate, dt):
        # Add your dynamics here
        ...
        return new_state

drone = Drone(drone_id=1, physics_model=MyDynamicPhysics())
```

### Swapping the Controller

Create a new controller class with the same interface as `SwarmController`.

## Visualization

- **Yellow dot**: Center beacon
- **Green circle**: Inner boundary (turn trigger)
- **Red circle**: Outer boundary (emergency land)
- **Blue drone**: Drone 1 (yaws CW during avoidance)
- **Orange drone**: Drone 2 (yaws CCW during avoidance)
- **Magenta**: Avoidance mode active
- **Trails**: Recent trajectory history
- **HUD**: Shows per-drone noise status (P=Process, S=Sensor)
