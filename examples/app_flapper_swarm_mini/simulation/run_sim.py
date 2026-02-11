#!/usr/bin/env python3
"""
Run the Flapper Swarm Simulation.

This is the main entry point for the 2D simulation.
Run with: python run_sim.py

Controls:
  SPACE - Pause/Resume
  R     - Reset simulation
  ESC   - Quit

Configuration:
  All tunable parameters are in config.py - edit default_config there.
"""
import sys
import os

# Add parent directory to path for imports when running as module
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
sys.path.insert(0, parent_dir)

from simulation import run_simulation, default_config


def main():
    """Run the simulation with the default configuration from config.py."""
    config = default_config
    
    print("=" * 60)
    print("Flapper Swarm Simulation")
    print("=" * 60)
    print()
    print("Controls:")
    print("  SPACE - Pause/Resume")
    print("  R     - Reset simulation")
    print("  ESC   - Quit")
    print()
    print("Flight parameters:")
    print(f"  Inner boundary:     {config.flight.inner_bound_m}m")
    print(f"  Outer boundary:     {config.flight.dist0_abort_m}m")
    print(f"  Avoidance distance: {config.flight.peer_close_m}m")
    print(f"  Forward speed:      {config.flight.fwd_speed_mps}m/s")
    print(f"  Demo duration:      {config.flight.demo_time_s}s")
    print()
    print("Drone configurations:")
    for i, drone_cfg in enumerate([config.drone1, config.drone2], start=1):
        noise = drone_cfg.noise
        init = drone_cfg.initial
        print(f"  Drone {i}: pos=({init.x:.1f}, {init.y:.1f})m, yaw={init.yaw:.0f}°")
        print(f"    Process noise: {'ON' if noise.enable_process_noise else 'OFF'}", end="")
        if noise.enable_process_noise:
            print(f" (vx={noise.process_vx_std}, vy={noise.process_vy_std}, bias={noise.process_vy_bias})")
        else:
            print()
        print(f"    Sensor noise:  {'ON' if noise.enable_sensor_noise else 'OFF'}", end="")
        if noise.enable_sensor_noise:
            print(f" (std={noise.uwb_distance_std}m)")
        else:
            print()
    print()
    print("Edit config.py to change parameters.")
    print()
    
    run_simulation(config)


if __name__ == "__main__":
    main()
