#!/usr/bin/env python3
"""
Run Monte Carlo simulations to evaluate the swarm policy.

This script runs many headless simulations and collects statistics
on success rate, failure modes (out of bounds vs collision), and other metrics.

Usage:
    python run_monte_carlo.py                    # Run 100 simulations
    python run_monte_carlo.py --runs 500         # Run 500 simulations
    python run_monte_carlo.py --output results.csv  # Save results to CSV
    python run_monte_carlo.py --seed 42          # Use fixed seed for reproducibility

Examples:
    # Quick test
    python run_monte_carlo.py --runs 10
    
    # Full evaluation with saved results
    python run_monte_carlo.py --runs 1000 --output results.csv --seed 42
"""
import argparse
import sys
import os

# Add parent directory to path for imports when running as module
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
sys.path.insert(0, parent_dir)

from simulation import default_config
from simulation.monte_carlo import run_monte_carlo, MonteCarloRunner


def main():
    parser = argparse.ArgumentParser(
        description="Run Monte Carlo simulations to evaluate the swarm policy.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        "--runs", "-n",
        type=int,
        default=100,
        help="Number of simulations to run (default: 100)"
    )
    parser.add_argument(
        "--seed", "-s",
        type=int,
        default=None,
        help="Random seed for reproducibility (default: random)"
    )
    parser.add_argument(
        "--output", "-o",
        type=str,
        default=None,
        help="Output file for results (CSV or JSON based on extension)"
    )
    parser.add_argument(
        "--quiet", "-q",
        action="store_true",
        help="Suppress progress output"
    )
    
    args = parser.parse_args()
    
    config = default_config
    
    print("=" * 60)
    print("Monte Carlo Simulation")
    print("=" * 60)
    print()
    print(f"Number of runs: {args.runs}")
    print(f"Random seed:    {args.seed if args.seed is not None else 'random'}")
    print(f"Output file:    {args.output if args.output else 'none'}")
    print()
    print("Configuration:")
    print(f"  Demo duration:      {config.flight.demo_time_s}s")
    print(f"  Outer boundary:     {config.flight.dist0_abort_m}m")
    print(f"  Avoidance distance: {config.flight.peer_close_m}m")
    print(f"  Collision distance: {config.flight.avoid_min_land_m}m")
    print()
    print("Drone 1 noise:")
    print(f"  Process: {'ON' if config.drone1.noise.enable_process_noise else 'OFF'}")
    print(f"  Sensor:  {'ON' if config.drone1.noise.enable_sensor_noise else 'OFF'}")
    print("Drone 2 noise:")
    print(f"  Process: {'ON' if config.drone2.noise.enable_process_noise else 'OFF'}")
    print(f"  Sensor:  {'ON' if config.drone2.noise.enable_sensor_noise else 'OFF'}")
    print()
    print("Running simulations...")
    print()
    
    stats = run_monte_carlo(
        config=config,
        num_runs=args.runs,
        seed=args.seed,
        output_file=args.output,
        verbose=not args.quiet,
    )
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
