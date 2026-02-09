"""
Main simulator with Pygame visualization.

This module ties together the drone physics and controller,
and provides a visual simulation using Pygame.
"""
import math
import random
import sys
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Tuple, Optional

from .config import Config, default_config, DroneConfig, DroneInitialState
from .drone import Drone, DroneState, NoisyKinematicPhysics, KinematicPhysics, UWBSensor
from .controller import SwarmController, FlightMode, LandingReason


# =============================================================================
# SIMULATION RESULT TYPES (for headless/Monte Carlo mode)
# =============================================================================

class TerminationReason(Enum):
    """Reason why a simulation ended."""
    SUCCESS = auto()              # Both drones completed full demo time
    PARTIAL_SUCCESS = auto()
    OUT_OF_BOUNDS = auto()        # At least one drone exceeded outer boundary
    COLLISION = auto()            # Drones got too close to each other
    UNKNOWN = auto()              # Unknown/unexpected termination


@dataclass
class InitialState:
    """Initial state of a drone."""
    x: float
    y: float
    z: float
    yaw: float


@dataclass
class SimulationResult:
    """Result of a single simulation run."""
    termination_reason: TerminationReason
    time_elapsed: float           # How long the simulation ran (seconds)
    min_peer_distance: float      # Minimum distance between drones (meters)
    avoidance_events: int         # Number of times avoidance was triggered
    drone1_landed_early: bool     # Whether drone 1 landed before demo time
    drone2_landed_early: bool     # Whether drone 2 landed before demo time
    drone1_initial: InitialState = None  # Initial state of drone 1
    drone2_initial: InitialState = None  # Initial state of drone 2
    
    @property
    def success(self) -> bool:
        """True if simulation completed fully successfully."""
        return self.termination_reason == TerminationReason.SUCCESS
    
    @property
    def partial_succes(self) -> bool:
        return self.termination_reason == TerminationReason.PARTIAL_SUCCESS
    
    @property
    def failed_out_of_bounds(self) -> bool:
        """True if simulation failed due to out of bounds."""
        return self.termination_reason == TerminationReason.OUT_OF_BOUNDS
    
    @property
    def failed_collision(self) -> bool:
        """True if simulation failed due to collision."""
        return self.termination_reason == TerminationReason.COLLISION
    
    @property
    def any_failure(self) -> bool:
        """True if there was any problem (including partial success)."""
        return self.termination_reason != TerminationReason.SUCCESS


class Simulator:
    """
    Main simulation class.
    
    Handles the simulation loop, visualization, and coordination
    between drones and their controllers.
    
    Can run in two modes:
    - Visual mode (default): Uses Pygame for visualization
    - Headless mode: No visualization, faster, returns results
    """
    
    def __init__(self, config: Config = None, headless: bool = False):
        """
        Initialize the simulator.
        
        Args:
            config: Configuration object (uses default if not provided)
            headless: If True, run without visualization (for Monte Carlo)
        """
        self.config = config or default_config
        self.running = False
        self.paused = False
        self.time = 0.0
        self.headless = headless
        
        # Termination tracking (for headless mode)
        self._termination_reason = TerminationReason.UNKNOWN
        
        # Initialize pygame only if not headless
        if not headless:
            try:
                import pygame
                self._pygame = pygame
            except ImportError:
                print("Pygame is required for visual mode. Install with: pip install pygame")
                sys.exit(1)
            
            self._pygame.init()
            self._pygame.display.set_caption("Flapper Swarm Simulation")
            self.screen = self._pygame.display.set_mode(self.config.viz.window_size)
            self.clock = self._pygame.time.Clock()
            self.font = self._pygame.font.Font(None, 24)
        else:
            self._pygame = None
            self.screen = None
            self.clock = None
            self.font = None
        
        # Create per-drone UWB sensor models
        self.uwb_sensors: List[UWBSensor] = []
        
        # Create drones
        self.drones: List[Drone] = []
        self.controllers: List[SwarmController] = []
        self._init_drones()
        
        # Statistics
        self.min_peer_distance = float('inf')
        self.avoidance_events = 0
        
        # Random initial state overrides (None = use config values)
        self._random_initial_states: List[Optional[DroneInitialState]] = [None, None]
    
    def _generate_random_initial_state(self, drone_id: int) -> DroneInitialState:
        """
        Generate a random initial state for a drone.
        
        Places drones randomly within the inner boundary with random yaw,
        ensuring they are not too close to each other or the beacon.
        
        Args:
            drone_id: The drone ID (1 or 2)
            
        Returns:
            DroneInitialState with random x, y, yaw (z is kept from config)
        """
        cfg = self.config
        flight = cfg.flight
        beacon_x, beacon_y, beacon_z = cfg.beacon_pos
        
        # Get z from config (flying altitude)
        z = cfg.drone1.initial.z if drone_id == 1 else cfg.drone2.initial.z
        
        # Calculate safe radius: between min_safe and inner_bound
        min_safe_dist = flight.peer_close_m * 0.6  # Stay away from center a bit
        max_dist = flight.inner_bound_m * 0.95  # Stay inside inner bound
        
        # For drone 2, we need to ensure it's not too close to drone 1
        min_peer_dist = flight.peer_close_m * 1.2  # Start with safe margin from peer
        
        max_attempts = 100
        for _ in range(max_attempts):
            # Random distance from beacon
            r = random.uniform(min_safe_dist, max_dist)
            
            # Random angle around beacon
            theta = random.uniform(0, 2 * math.pi)
            
            x = beacon_x + r * math.cos(theta)
            y = beacon_y + r * math.sin(theta)
            
            # Random yaw (heading)
            yaw = random.uniform(-180, 180)
            
            # For drone 2, check distance to drone 1
            if drone_id == 2 and self._random_initial_states[0] is not None:
                d1 = self._random_initial_states[0]
                dist_to_drone1 = math.sqrt((x - d1.x)**2 + (y - d1.y)**2)
                if dist_to_drone1 < min_peer_dist:
                    continue  # Too close, try again
            
            return DroneInitialState(x=x, y=y, z=z, yaw=yaw)
        
        # Fallback: if we couldn't find a good position, use config defaults
        if drone_id == 1:
            return cfg.drone1.initial
        else:
            return cfg.drone2.initial
    
    def randomize_initial_states(self) -> None:
        """Generate random initial states for both drones."""
        self._random_initial_states[0] = self._generate_random_initial_state(1)
        self._random_initial_states[1] = self._generate_random_initial_state(2)

    def _init_drones(self) -> None:
        """Initialize drones and controllers with per-drone configurations."""
        cfg = self.config
        drone_configs = [cfg.drone1, cfg.drone2]
        
        self.drones = []
        self.controllers = []
        self.uwb_sensors = []
        
        for drone_id, drone_cfg in enumerate(drone_configs, start=1):
            noise = drone_cfg.noise
            
            # Use random initial state if set, otherwise use config
            idx = drone_id - 1
            if hasattr(self, '_random_initial_states') and self._random_initial_states[idx] is not None:
                init = self._random_initial_states[idx]
            else:
                init = drone_cfg.initial
            
            # Select physics model based on noise configuration
            if noise.enable_process_noise:
                physics = NoisyKinematicPhysics(
                    vx_std=noise.process_vx_std,
                    vy_std=noise.process_vy_std,
                    yaw_rate_std=noise.process_yaw_rate_std,
                    vy_bias=noise.process_vy_bias,
                    vy_bias_max=noise.process_vy_bias_max,
                    vy_bias_walk_std=noise.process_vy_bias_walk_std
                )
            else:
                physics = KinematicPhysics()
            
            # Create drone
            drone = Drone(
                drone_id=drone_id,
                initial_x=init.x,
                initial_y=init.y,
                initial_z=init.z,
                initial_yaw=init.yaw,
                physics_model=physics,
                trail_length=cfg.viz.trail_length
            )
            
            # Create controller
            controller = SwarmController(
                drone_id=drone_id,
                params=cfg.flight,
                sim_params=cfg.sim
            )
            
            # Create UWB sensor for this drone
            uwb_sensor = UWBSensor(
                distance_std=noise.uwb_distance_std,
                distance_bias=noise.uwb_distance_bias,
                enabled=noise.enable_sensor_noise
            )
            
            self.drones.append(drone)
            self.controllers.append(controller)
            self.uwb_sensors.append(uwb_sensor)
    
    def reset(self) -> None:
        """Reset simulation to initial state."""
        self.time = 0.0
        self.min_peer_distance = float('inf')
        self.avoidance_events = 0
        
        # Track early landings and reasons for headless mode
        self._early_landing_times = [None, None]  # When each drone landed early (None if didn't)
        self._early_landing_reasons = [None, None]  # 'collision' or 'out_of_bounds'
        
        # Reinitialize drones with fresh physics models
        self._init_drones()
        
        # Reset controllers
        for ctrl in self.controllers:
            ctrl.reset()
    
    def run(self) -> None:
        """Run the simulation loop with visualization."""
        if self.headless:
            raise RuntimeError("Cannot run visual mode in headless simulator. Use run_headless() instead.")
        
        self.running = True
        self.reset()
        
        # Start drones
        for drone in self.drones:
            drone.takeoff()
        
        while self.running:
            self._handle_events()
            
            if not self.paused:
                self._update()
            
            self._render()
            self.clock.tick(self.config.viz.fps)
        
        self._pygame.quit()
    
    def run_headless(self) -> SimulationResult:
        """
        Run the simulation without visualization.
        
        Returns:
            SimulationResult with termination reason and statistics
        """
        self.running = True
        self.reset()
        self._termination_reason = TerminationReason.UNKNOWN
        
        # Capture initial states before simulation starts
        drone1_init = InitialState(
            x=self.config.drone1.initial.x,
            y=self.config.drone1.initial.y,
            z=self.config.drone1.initial.z,
            yaw=self.config.drone1.initial.yaw,
        )
        drone2_init = InitialState(
            x=self.config.drone2.initial.x,
            y=self.config.drone2.initial.y,
            z=self.config.drone2.initial.z,
            yaw=self.config.drone2.initial.yaw,
        )
        
        # Start drones
        for drone in self.drones:
            drone.takeoff()
        
        # Run until termination
        while self.running:
            self._update_headless()
        
        # Determine if drones landed early
        demo_time = self.config.flight.demo_time_s
        drone1_early = not self.drones[0].is_flying
        drone2_early = not self.drones[1].is_flying
        
        return SimulationResult(
            termination_reason=self._termination_reason,
            time_elapsed=self.time,
            min_peer_distance=self.min_peer_distance,
            avoidance_events=self.avoidance_events,
            drone1_landed_early=drone1_early,
            drone2_landed_early=drone2_early,
            drone1_initial=drone1_init,
            drone2_initial=drone2_init,
        )
    
    def _handle_events(self) -> None:
        """Handle pygame events."""
        pygame = self._pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_r:
                    # Randomize initial positions and reset
                    self.randomize_initial_states()
                    self.reset()
                    for drone in self.drones:
                        drone.takeoff()
    
    def _update(self) -> None:
        """Update simulation state."""
        dt = self.config.sim.dt
        beacon = self.config.beacon_pos
        
        # Check if demo time exceeded
        if self.time >= self.config.flight.demo_time_s:
            for drone in self.drones:
                if drone.is_flying:
                    drone.land()
            return
        
        # Track avoidance state changes
        prev_avoiding = [ctrl.is_avoiding for ctrl in self.controllers]
        
        # Update each drone
        for i, (drone, ctrl, uwb) in enumerate(zip(self.drones, self.controllers, self.uwb_sensors)):
            if not drone.is_flying:
                continue
            
            # Get other drone for peer distance
            other_drone = self.drones[1 - i]
            
            # Calculate TRUE distances (for statistics)
            true_d0 = drone.distance_to(*beacon)
            true_peer_dist = drone.distance_to_drone(other_drone)
            # Track minimum TRUE peer distance (for collision detection stats)
            self.min_peer_distance = min(self.min_peer_distance, true_peer_dist)
            # Get MEASURED distances (with per-drone sensor noise for controller)
            measured_d0 = uwb.measure(true_d0)
            measured_peer_dist = uwb.measure(true_peer_dist) #if true_peer_dist != float('inf') else float('inf')
            
            # Get peer drone's z-height
            peer_z = other_drone.state.z
            
            # Get control command using MEASURED (noisy) distances
            cmd, emergency_reason = ctrl.update(
                d0=measured_d0,
                peer_dist=measured_peer_dist,
                current_yaw=drone.yaw,
                time=self.time,
                peer_z=peer_z
            )
            
            if emergency_reason != LandingReason.NONE:
                drone.land()
                continue
            
            # Apply command to drone
            drone.update(cmd.vx_body, cmd.vy_body, cmd.yaw_rate, dt)
        
        # Count avoidance events
        for i, ctrl in enumerate(self.controllers):
            if ctrl.is_avoiding and not prev_avoiding[i]:
                self.avoidance_events += 1
        
        self.time += dt
    
    def _update_headless(self) -> None:
        """Update simulation state in headless mode (with termination detection)."""
        dt = self.config.sim.dt
        beacon = self.config.beacon_pos
        flight = self.config.flight
        
        # Check if demo time exceeded -> SUCCESS
        if self.time >= flight.demo_time_s:
            
            # Check if any drone landed early (partial success)
            if any(t is not None for t in self._early_landing_times):
                # Determine termination reason from the recorded landing reasons
                if any(r == LandingReason.COLLISION for r in self._early_landing_reasons if r):
                    self._termination_reason = TerminationReason.COLLISION
                elif any(r == LandingReason.OUT_OF_BOUNDS for r in self._early_landing_reasons if r):
                    self._termination_reason = TerminationReason.OUT_OF_BOUNDS
                else:
                    self._termination_reason = TerminationReason.PARTIAL_SUCCESS
            else:
                self._termination_reason = TerminationReason.SUCCESS
            self.running = False
            return
        
        # Check if all drones have landed (premature termination)
        if not any(drone.is_flying for drone in self.drones):
            # Determine why based on the recorded early landing reasons
            if any(r == LandingReason.OUT_OF_BOUNDS for r in self._early_landing_reasons if r):
                self._termination_reason = TerminationReason.OUT_OF_BOUNDS
            else:
                self._termination_reason = TerminationReason.COLLISION
            self.running = False
            return
        
        # Track avoidance state changes
        prev_avoiding = [ctrl.is_avoiding for ctrl in self.controllers]
        
        # Update each drone
        for i, (drone, ctrl, uwb) in enumerate(zip(self.drones, self.controllers, self.uwb_sensors)):
            if not drone.is_flying:
                continue
            
            # Get other drone for peer distance
            other_drone = self.drones[1 - i]
            
            # Calculate TRUE distances (for statistics)
            true_d0 = drone.distance_to(*beacon)
            true_peer_dist = drone.distance_to_drone(other_drone)
            # Track minimum TRUE peer distance (for collision detection stats)
            self.min_peer_distance = min(self.min_peer_distance, true_peer_dist)

            # Get MEASURED distances (with per-drone sensor noise for controller)
            measured_d0 = uwb.measure(true_d0)
            measured_peer_dist = uwb.measure(true_peer_dist) if true_peer_dist != float('inf') else float('inf')
            
            # Get peer drone's z-height
            peer_z = other_drone.state.z
            
            # Get control command using MEASURED (noisy) distances
            cmd, emergency_reason = ctrl.update(
                d0=measured_d0,
                peer_dist=measured_peer_dist,
                current_yaw=drone.yaw,
                time=self.time,
                peer_z=peer_z
            )
            
            if emergency_reason != LandingReason.NONE:
                # print(f"drone: {i}, dist: {true_peer_dist:.2f},  min_dist: {self.min_peer_distance:.2f}, reason: {emergency_reason}")
                # Record early landing info
                self._early_landing_times[i] = self.time
                self._early_landing_reasons[i] = emergency_reason
                drone.land()
                continue
            
            # Apply command to drone
            drone.update(cmd.vx_body, cmd.vy_body, cmd.yaw_rate, dt)
        
        # Count avoidance events
        for i, ctrl in enumerate(self.controllers):
            if ctrl.is_avoiding and not prev_avoiding[i]:
                self.avoidance_events += 1
        
        self.time += dt
    
    def _world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to screen coordinates."""
        viz = self.config.viz
        cx, cy = viz.window_size[0] // 2, viz.window_size[1] // 2
        
        # Note: Y is inverted (screen Y increases downward)
        sx = int(cx + x * viz.pixels_per_meter)
        sy = int(cy - y * viz.pixels_per_meter)
        
        return (sx, sy)
    
    def _render(self) -> None:
        """Render the simulation."""
        pygame = self._pygame
        viz = self.config.viz
        flight = self.config.flight
        
        # Clear screen
        self.screen.fill(viz.background_color)
        
        # Draw boundaries (use only x, y for 2D visualization)
        beacon_x, beacon_y, beacon_z = self.config.beacon_pos
        center = self._world_to_screen(beacon_x, beacon_y)
        
        # Get flying altitude (use first flying drone's z, or default to drone config)
        flying_z = self.config.drone1.initial.z
        for drone in self.drones:
            if drone.is_flying:
                flying_z = drone.state.z
            break
        
        # Calculate horizontal distance scaling factor due to altitude difference
        dz = abs(flying_z - beacon_z)
        
        # Outer abort boundary (adjusted for altitude)
        outer_radius_3d = math.sqrt(flight.dist0_abort_m**2 - dz**2) if flight.dist0_abort_m > dz else 0
        outer_radius = int(outer_radius_3d * viz.pixels_per_meter)
        pygame.draw.circle(self.screen, viz.outer_bound_color, center, outer_radius, 2)
        
        # Inner turn boundary (adjusted for altitude)
        inner_radius_3d = math.sqrt(flight.inner_bound_m**2 - dz**2) if flight.inner_bound_m > dz else 0
        inner_radius = int(inner_radius_3d * viz.pixels_per_meter)
        pygame.draw.circle(self.screen, viz.inner_bound_color, center, inner_radius, 2)
        
        # Peer avoidance radius (for reference)
        avoid_radius = int(flight.peer_close_m * viz.pixels_per_meter)
        # Draw around each drone
        for drone in self.drones:
            if drone.is_flying:
                pos = self._world_to_screen(*drone.position)
                pygame.draw.circle(self.screen, (50, 50, 50), pos, avoid_radius, 1)
        
        # Draw beacon
        pygame.draw.circle(self.screen, viz.beacon_color, center, 8)
        
        # Draw drones
        drone_colors = [viz.drone1_color, viz.drone2_color]
        
        for i, (drone, ctrl) in enumerate(zip(self.drones, self.controllers)):
            color = drone_colors[i]
            
            # Use special color for AVOID or DANCE states
            if ctrl.is_avoiding:
                color = viz.avoidance_color
            elif ctrl.is_dancing:
                color = viz.dance_color
            
            # Draw trail
            if viz.show_trail and len(drone.trail) > 1:
                trail_points = [self._world_to_screen(x, y) for x, y in drone.trail]
                if len(trail_points) >= 2:
                    pygame.draw.lines(self.screen, color, False, trail_points, 1)
            
            # Draw drone
            pos = self._world_to_screen(*drone.position)
            pygame.draw.circle(self.screen, color, pos, viz.drone_radius)
            
            # Draw heading indicator
            yaw_rad = math.radians(drone.yaw)
            head_len = viz.drone_radius * 2
            head_x = pos[0] + int(head_len * math.cos(yaw_rad))
            head_y = pos[1] - int(head_len * math.sin(yaw_rad))
            pygame.draw.line(self.screen, color, pos, (head_x, head_y), 2)
        
        # Draw HUD
        self._render_hud()
        
        pygame.display.flip()
    
    def _render_hud(self) -> None:
        """Render heads-up display with status info."""
        viz = self.config.viz
        flight = self.config.flight
        drone_configs = [self.config.drone1, self.config.drone2]
        
        lines = [
            f"Time: {self.time:.1f}s / {flight.demo_time_s:.1f}s",
            f"Min peer dist: {self.min_peer_distance:.2f}m",
            f"Avoidance events: {self.avoidance_events}",
            "",
        ]
        
        # Add drone status with per-drone noise info
        for i, (drone, ctrl) in enumerate(zip(self.drones, self.controllers)):
            d0 = drone.distance_to(*self.config.beacon_pos)
            mode_str = ctrl.mode.name
            if ctrl.is_avoiding:
                mode_str = "AVOID"
            state_str = drone.flight_state.name
            
            # Per-drone noise status
            noise = drone_configs[i].noise
            noise_flags = []
            if noise.enable_process_noise:
                noise_flags.append("P")
            if noise.enable_sensor_noise:
                noise_flags.append("S")
            noise_str = "+".join(noise_flags) if noise_flags else "-"
            
            lines.append(f"Drone {i+1}: {state_str} | {mode_str} | d0={d0:.2f}m | noise={noise_str}")
        
        lines.append("")
        lines.append("[SPACE] Pause  [R] Random Reset  [ESC] Quit")
        
        if self.paused:
            lines.insert(0, "== PAUSED ==")
        
        y = 10
        for line in lines:
            text = self.font.render(line, True, viz.text_color)
            self.screen.blit(text, (10, y))
            y += 20


def run_simulation(config: Config = None) -> None:
    """
    Run the simulation with the given configuration.
    
    Args:
        config: Configuration object (uses default if not provided)
    """
    sim = Simulator(config)
    sim.run()
