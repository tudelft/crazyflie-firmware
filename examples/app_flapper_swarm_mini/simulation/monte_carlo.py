"""
Monte Carlo simulation runner for evaluating swarm policies.

This module provides tools to run many simulations and collect statistics
to evaluate the robustness of different controller configurations.
"""
import copy
import math
import random
import time
from dataclasses import dataclass, field
from typing import List, Optional, Callable
import json
import csv
from pathlib import Path

from .config import Config, default_config
from .simulator import Simulator, SimulationResult, TerminationReason


# =============================================================================
# CONFIG MODIFIERS FOR RANDOMIZING INITIAL CONDITIONS
# =============================================================================

def create_random_initial_conditions_modifier(
    x_range: tuple = (-1.5, 1.5),
    y_range: tuple = (-1.5, 1.5),
    yaw_range: tuple = (-180, 180.0),
    min_drone_separation: float = 1.0,
) -> Callable[[Config, int], Config]:
    """
    Create a config modifier that randomizes initial x, y, and yaw for both drones.
    
    Args:
        x_range: (min, max) range for x position in meters
        y_range: (min, max) range for y position in meters  
        yaw_range: (min, max) range for yaw in degrees
        min_drone_separation: Minimum distance between drones at start (meters)
    
    Returns:
        A config modifier function suitable for MonteCarloRunner
    """
    def modifier(config: Config, run_index: int) -> Config:
        # Randomize drone 1 position
        config.drone1.initial.x = random.uniform(*x_range)
        config.drone1.initial.y = random.uniform(*y_range)
        config.drone1.initial.yaw = random.uniform(*yaw_range)
        
        # Randomize drone 2 position, ensuring minimum separation
        max_attempts = 100
        for _ in range(max_attempts):
            x2 = random.uniform(*x_range)
            y2 = random.uniform(*y_range)
            
            # Check separation from drone 1
            dx = x2 - config.drone1.initial.x
            dy = y2 - config.drone1.initial.y
            separation = math.sqrt(dx * dx + dy * dy)
            
            if separation >= min_drone_separation:
                config.drone2.initial.x = x2
                config.drone2.initial.y = y2
                break
        else:
            # If we couldn't find a valid position, just use the random one
            config.drone2.initial.x = x2
            config.drone2.initial.y = y2

        config.drone2.initial.yaw = random.uniform(*yaw_range)
        
        return config
    
    return modifier


@dataclass
class MonteCarloStats:
    """Aggregated statistics from Monte Carlo runs."""
    
    total_runs: int = 0
    successful_runs: int = 0
    partial_success_runs: int = 0  # One drone had issues but other completed
    out_of_bounds_failures: int = 0
    collision_failures: int = 0
    unknown_failures: int = 0
    
    # Timing
    total_sim_time: float = 0.0  # Sum of all simulation times
    avg_sim_time: float = 0.0   # Average time before termination
    
    # Distance statistics
    min_peer_distance_overall: float = float('inf')
    avg_min_peer_distance: float = 0.0
    
    # Avoidance statistics
    total_avoidance_events: int = 0
    avg_avoidance_events: float = 0.0
    
    # Wall clock time
    wall_clock_time: float = 0.0
    
    @property
    def success_rate(self) -> float:
        """Percentage of fully successful runs (both drones completed)."""
        return (self.successful_runs / self.total_runs * 100) if self.total_runs > 0 else 0.0
    
    @property
    def partial_success_rate(self) -> float:
        """Percentage of runs where one drone had issues but simulation completed."""
        return (self.partial_success_runs / self.total_runs * 100) if self.total_runs > 0 else 0.0
    
    @property
    def out_of_bounds_rate(self) -> float:
        """Percentage of runs that failed due to out of bounds."""
        return (self.out_of_bounds_failures / self.total_runs * 100) if self.total_runs > 0 else 0.0
    
    @property
    def collision_rate(self) -> float:
        """Percentage of runs that failed due to collision."""
        return (self.collision_failures / self.total_runs * 100) if self.total_runs > 0 else 0.0
    
    @property
    def failure_rate(self) -> float:
        """Percentage of failed runs (both drones down early)."""
        return (self.out_of_bounds_failures + self.collision_failures) / self.total_runs * 100 if self.total_runs > 0 else 0.0
    
    @property
    def any_issue_rate(self) -> float:
        """Percentage of runs with any issue (partial success + failures)."""
        return 100.0 - self.success_rate
    
    def __str__(self) -> str:
        """Human-readable summary."""
        lines = [
            "=" * 60,
            "Monte Carlo Simulation Results",
            "=" * 60,
            f"Total runs:              {self.total_runs}",
            f"Wall clock time:         {self.wall_clock_time:.1f}s",
            "",
            "Outcomes:",
            f"  Full success:          {self.success_rate:.1f}% ({self.successful_runs}/{self.total_runs})",
            f"  Partial success:       {self.partial_success_rate:.1f}% ({self.partial_success_runs}/{self.total_runs})",
            f"  Out of bounds:         {self.out_of_bounds_rate:.1f}% ({self.out_of_bounds_failures}/{self.total_runs})",
            f"  Collision:             {self.collision_rate:.1f}% ({self.collision_failures}/{self.total_runs})",
            "",
            "Timing:",
            f"  Avg simulation time:   {self.avg_sim_time:.2f}s",
            "",
            "Distance:",
            f"  Min peer distance:     {self.min_peer_distance_overall:.3f}m (overall)",
            f"  Avg min peer distance: {self.avg_min_peer_distance:.3f}m",
            "",
            "Avoidance:",
            f"  Total events:          {self.total_avoidance_events}",
            f"  Avg events per run:    {self.avg_avoidance_events:.2f}",
            "=" * 60,
        ]
        return "\n".join(lines)


class MonteCarloRunner:
    """
    Runs multiple simulations and collects statistics.
    
    Usage:
        runner = MonteCarloRunner(config, num_runs=100)
        stats = runner.run()
        print(stats)
    """
    
    def __init__(
        self,
        config: Config = None,
        num_runs: int = 100,
        seed: Optional[int] = None,
        verbose: bool = True,
        config_modifier: Optional[Callable[[Config, int], Config]] = None,
    ):
        """
        Initialize the Monte Carlo runner.
        
        Args:
            config: Base configuration (uses default if not provided)
            num_runs: Number of simulations to run
            seed: Random seed for reproducibility (None for random)
            verbose: Print progress during runs
            config_modifier: Optional function to modify config for each run.
                             Signature: (config, run_index) -> modified_config
                             Use this for parameter sweeps or randomization.
        """
        self.base_config = config or default_config
        self.num_runs = num_runs
        self.seed = seed
        self.verbose = verbose
        self.config_modifier = config_modifier
        
        # Store individual results
        self.results: List[SimulationResult] = []
    
    def run(self) -> MonteCarloStats:
        """
        Run all simulations and return aggregated statistics.
        
        Returns:
            MonteCarloStats with aggregated results
        """
        if self.seed is not None:
            random.seed(self.seed)
        
        self.results = []
        stats = MonteCarloStats()
        
        start_wall_time = time.time()
        
        for i in range(self.num_runs):
            # Get config for this run (potentially modified)
            if self.config_modifier:
                run_config = self.config_modifier(copy.deepcopy(self.base_config), i)
            else:
                run_config = self.base_config
            
            # Run simulation
            sim = Simulator(run_config, headless=True)
            result = sim.run_headless()
            self.results.append(result)
            
            # Update stats
            stats.total_runs += 1
            stats.total_sim_time += result.time_elapsed
            stats.total_avoidance_events += result.avoidance_events
            stats.min_peer_distance_overall = min(
                stats.min_peer_distance_overall, 
                result.min_peer_distance
            )
            
            if result.termination_reason == TerminationReason.SUCCESS:
                stats.successful_runs += 1
            elif result.termination_reason == TerminationReason.PARTIAL_SUCCESS:
                stats.partial_success_runs += 1
            elif result.termination_reason == TerminationReason.OUT_OF_BOUNDS:
                stats.out_of_bounds_failures += 1
            elif result.termination_reason == TerminationReason.COLLISION:
                stats.collision_failures += 1
            else:
                stats.unknown_failures += 1
            
            # Progress output
            if self.verbose and (i + 1) % max(1, self.num_runs // 10) == 0:
                progress = (i + 1) / self.num_runs * 100
                print(f"Progress: {i + 1}/{self.num_runs} ({progress:.0f}%) - "
                      f"Success: {stats.success_rate:.1f}%, "
                      f"Partial: {stats.partial_success_rate:.1f}%, "
                      f"OOB: {stats.out_of_bounds_rate:.1f}%, "
                      f"Collision: {stats.collision_rate:.1f}%")
        
        # Compute averages
        stats.wall_clock_time = time.time() - start_wall_time
        stats.avg_sim_time = stats.total_sim_time / stats.total_runs
        stats.avg_avoidance_events = stats.total_avoidance_events / stats.total_runs
        stats.avg_min_peer_distance = sum(r.min_peer_distance for r in self.results) / stats.total_runs
        
        return stats
    
    def save_results(self, filepath: str, format: str = "csv") -> None:
        """
        Save individual run results to a file.
        
        Args:
            filepath: Output file path
            format: Output format ("csv" or "json")
        """
        if not self.results:
            raise ValueError("No results to save. Run simulations first.")
        
        path = Path(filepath)
        
        if format.lower() == "csv":
            with open(path, "w", newline="") as f:
                writer = csv.writer(f)
                # Header
                writer.writerow([
                    "run",
                    "termination_reason",
                    "time_elapsed",
                    "min_peer_distance",
                    "avoidance_events",
                    "drone1_landed_early",
                    "drone2_landed_early",
                    "drone1_init_x",
                    "drone1_init_y",
                    "drone1_init_yaw",
                    "drone2_init_x",
                    "drone2_init_y",
                    "drone2_init_yaw",
                ])
                # Data
                for i, result in enumerate(self.results):
                    d1_init = result.drone1_initial
                    d2_init = result.drone2_initial
                    writer.writerow([
                        i + 1,
                        result.termination_reason.name,
                        f"{result.time_elapsed:.3f}",
                        f"{result.min_peer_distance:.4f}",
                        result.avoidance_events,
                        result.drone1_landed_early,
                        result.drone2_landed_early,
                        f"{d1_init.x:.4f}" if d1_init else "",
                        f"{d1_init.y:.4f}" if d1_init else "",
                        f"{d1_init.yaw:.4f}" if d1_init else "",
                        f"{d2_init.x:.4f}" if d2_init else "",
                        f"{d2_init.y:.4f}" if d2_init else "",
                        f"{d2_init.yaw:.4f}" if d2_init else "",
                    ])
        
        elif format.lower() == "json":
            data = {
                "runs": [
                    {
                        "run": i + 1,
                        "termination_reason": result.termination_reason.name,
                        "time_elapsed": result.time_elapsed,
                        "min_peer_distance": result.min_peer_distance,
                        "avoidance_events": result.avoidance_events,
                        "drone1_landed_early": result.drone1_landed_early,
                        "drone2_landed_early": result.drone2_landed_early,
                        "drone1_initial": {
                            "x": result.drone1_initial.x,
                            "y": result.drone1_initial.y,
                            "yaw": result.drone1_initial.yaw,
                        } if result.drone1_initial else None,
                        "drone2_initial": {
                            "x": result.drone2_initial.x,
                            "y": result.drone2_initial.y,
                            "yaw": result.drone2_initial.yaw,
                        } if result.drone2_initial else None,
                    }
                    for i, result in enumerate(self.results)
                ]
            }
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
        
        else:
            raise ValueError(f"Unknown format: {format}. Use 'csv' or 'json'.")
        
        if self.verbose:
            print(f"Results saved to: {path}")


def run_monte_carlo(
    config: Config = None,
    num_runs: int = 100,
    seed: Optional[int] = None,
    output_file: Optional[str] = None,
    verbose: bool = True,
    randomize_initial_conditions: bool = True,
    x_range: tuple = (-1.5, 1.5),
    y_range: tuple = (-1.5, 1.5),
    yaw_range: tuple = (-180, 180.0),
    min_drone_separation: float = 1.5,
) -> MonteCarloStats:
    """
    Convenience function to run Monte Carlo simulations.
    
    Args:
        config: Configuration to use (default if not provided)
        num_runs: Number of simulations to run
        seed: Random seed for reproducibility
        output_file: Optional file to save results (CSV format)
        verbose: Print progress and results
        randomize_initial_conditions: If True, randomize x, y, yaw for both drones
        x_range: (min, max) range for x position in meters
        y_range: (min, max) range for y position in meters
        yaw_range: (min, max) range for yaw in degrees
        min_drone_separation: Minimum distance between drones at start (meters)
    
    Returns:
        MonteCarloStats with aggregated results
    """
    # Create modifier if randomizing initial conditions
    config_modifier = None
    if randomize_initial_conditions:
        config_modifier = create_random_initial_conditions_modifier(
            x_range=x_range,
            y_range=y_range,
            yaw_range=yaw_range,
            min_drone_separation=min_drone_separation,
        )
    
    runner = MonteCarloRunner(
        config=config,
        num_runs=num_runs,
        seed=seed,
        verbose=verbose,
        config_modifier=config_modifier,
    )
    
    stats = runner.run()
    
    if verbose:
        print()
        print(stats)
    
    if output_file:
        # Determine format from extension
        ext = Path(output_file).suffix.lower()
        fmt = "json" if ext == ".json" else "csv"
        runner.save_results(output_file, format=fmt)
    
    return stats
