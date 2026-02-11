"""
Configuration parameters for the drone swarm simulation.

All distances are in meters, angles in degrees, time in seconds.
These mirror the parameters from flapper_swarm.c but converted to SI units.

=============================================================================
THIS IS THE SINGLE SOURCE OF TRUTH FOR ALL TUNABLE PARAMETERS
Edit the values in `default_config` at the bottom of this file to tune.
=============================================================================
"""
from dataclasses import dataclass, field
from typing import Tuple


# =============================================================================
# FLIGHT PARAMETERS (matching C firmware)
# =============================================================================
@dataclass
class FlightParams:
    """Flight parameters matching the C firmware."""
    
    # Forward speed in m/s
    fwd_speed_mps: float = 0.5
    
    # Outer emergency boundary to beacon (meters) - land if exceeded
    dist0_abort_m: float = 3.0
    
    # Inner boundary (meters) - start turning when exceeded
    inner_bound_m: float = 1.30
    
    # Yaw rate while turning (deg/s)
    turn_yaw_rate_dps: float = 40.0
    
    # Peer distance threshold for avoidance (meters)
    peer_close_m: float = 2.0
    
    # Minimum peer distance before emergency land (meters)
    avoid_min_land_m: float = 0.6
    
    # Speed factor during avoidance (multiplier)
    avoid_speed_factor: float = 1.0
    
    # Yaw rate magnitude for avoidance (deg/s)
    avoid_yaw_rate_dps: float = 70.0
    
    # Confirmation counts (number of consecutive samples to trigger)
    abort_confirm_count: int = 2
    avoid_enter_confirm_count: int = 2
    avoid_exit_confirm_count: int = 4
    
    # Derivative-based recovery parameters
    des_deriv_mps: float = -1.4  # Desired derivative (m/s, negative = toward beacon)
    recover_yaw_rate_dps: float = 50.0  # Yaw rate at 0 derivative
    recover_deadzone_dps: float = 30.0  # Stop rotating within this yaw rate
    
    # Demo duration in seconds
    demo_time_s: float = 60.0


# =============================================================================
# SIMULATION PARAMETERS
# =============================================================================
@dataclass
class SimulationParams:
    """Simulation-specific parameters."""
    
    # Time step in seconds
    dt: float = 0.02  # 50 Hz, matching the 20ms loop in firmware
    
    # Derivative buffer size (for linear regression)
    deriv_buffer_size: int = 10
    
    # Derivative sample interval in seconds
    deriv_sample_interval_s: float = 0.02


# =============================================================================
# NOISE PARAMETERS (per-drone)
# =============================================================================
@dataclass
class DroneNoiseParams:
    """
    Noise parameters for a single drone.
    
    Process noise models imperfect velocity tracking due to estimation errors.
    Sensor noise models UWB distance measurement noise.
    """
    
    # === Enable/Disable Flags ===
    enable_process_noise: bool = True
    enable_sensor_noise: bool = True
    
    # === Process Noise (velocity tracking errors) ===
    # Forward velocity noise std dev (m/s)
    process_vx_std: float = 0.02
    
    # Lateral velocity noise std dev (m/s) - typically larger due to worse estimation
    process_vy_std: float = 0.05
    
    # Yaw rate noise std dev (deg/s)
    process_yaw_rate_std: float = 2.0
    
    # Initial lateral drift bias (m/s) - starting value for random walk
    process_vy_bias: float = 0.0
    
    # Maximum lateral drift (m/s) - bounds the random walk
    process_vy_bias_max: float = 0.15
    
    # Random walk step std dev (m/s per update) - controls how fast bias changes
    process_vy_bias_walk_std: float = 0.01
    
    # === Sensor Noise (UWB distance measurements) ===
    # Distance measurement noise std dev (meters)
    uwb_distance_std: float = 0.05
    
    # Systematic measurement bias (meters)
    uwb_distance_bias: float = 0.0


# =============================================================================
# DRONE INITIAL STATE
# =============================================================================
@dataclass
class DroneInitialState:
    """Initial state for a drone."""
    x: float = 0.0  # meters
    y: float = 0.0  # meters
    z: float = 0.0  # meters (altitude)
    yaw: float = 0.0  # degrees


# =============================================================================
# COMPLETE DRONE CONFIGURATION
# =============================================================================
@dataclass
class DroneConfig:
    """Complete configuration for a single drone (initial state + noise)."""
    initial: DroneInitialState = field(default_factory=DroneInitialState)
    noise: DroneNoiseParams = field(default_factory=DroneNoiseParams)


# =============================================================================
# VISUALIZATION PARAMETERS
# =============================================================================
@dataclass
class VisualizationParams:
    """Pygame visualization parameters."""
    
    # Window size in pixels
    window_size: Tuple[int, int] = (900, 900)
    
    # Scale: pixels per meter
    pixels_per_meter: float = 90.0
    
    # Target FPS
    fps: int = 50
    
    # Colors (RGB)
    background_color: Tuple[int, int, int] = (30, 30, 30)
    beacon_color: Tuple[int, int, int] = (255, 255, 0)
    inner_bound_color: Tuple[int, int, int] = (0, 100, 0)
    outer_bound_color: Tuple[int, int, int] = (100, 0, 0)
    drone1_color: Tuple[int, int, int] = (0, 150, 255)
    drone2_color: Tuple[int, int, int] = (255, 100, 0)
    avoidance_color: Tuple[int, int, int] = (255, 0, 255)
    dance_color: Tuple[int, int, int] = (255, 255, 0)  # Yellow for DANCE state
    text_color: Tuple[int, int, int] = (255, 255, 255)
    
    # Drone visual size in pixels
    drone_radius: int = 10
    
    # Show trajectory trail
    show_trail: bool = True
    trail_length: int = 300


# =============================================================================
# MAIN CONFIGURATION CONTAINER
# =============================================================================
@dataclass
class Config:
    """Main configuration container."""
    
    # Beacon position (center of the arena)
    beacon_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # (x, y, z) in meters
    
    # Sub-configurations
    flight: FlightParams = field(default_factory=FlightParams)
    sim: SimulationParams = field(default_factory=SimulationParams)
    viz: VisualizationParams = field(default_factory=VisualizationParams)
    
    # Per-drone configurations
    drone1: DroneConfig = field(default_factory=DroneConfig)
    drone2: DroneConfig = field(default_factory=DroneConfig)


# =============================================================================
# DEFAULT CONFIGURATION - EDIT VALUES HERE TO TUNE
# =============================================================================
default_config = Config(
    # -------------------------------------------------------------------------
    # FLIGHT PARAMETERS
    # -------------------------------------------------------------------------
    flight=FlightParams(
        fwd_speed_mps=0.4,
        dist0_abort_m=3.0,
        inner_bound_m=1.6,
        turn_yaw_rate_dps=60.0,
        peer_close_m=2.0,
        avoid_min_land_m=0.5,
        avoid_speed_factor=1.0,
        avoid_yaw_rate_dps=60.0,
        abort_confirm_count=1,
        avoid_enter_confirm_count=1,
        avoid_exit_confirm_count=1,
        des_deriv_mps=-1.4,
        recover_yaw_rate_dps=50.0,
        recover_deadzone_dps=30.0,
        demo_time_s=60.0,
    ),
    
    # -------------------------------------------------------------------------
    # SIMULATION PARAMETERS
    # -------------------------------------------------------------------------
    sim=SimulationParams(
        dt=0.02,
        deriv_buffer_size=10,
        deriv_sample_interval_s=0.02,
    ),
    
    # -------------------------------------------------------------------------
    # VISUALIZATION PARAMETERS
    # -------------------------------------------------------------------------
    viz=VisualizationParams(
        window_size=(900, 900),
        pixels_per_meter=90.0,
        fps=50,
        show_trail=True,
        trail_length=300,
    ),
    
    # -------------------------------------------------------------------------
    # DRONE 1 CONFIGURATION
    # -------------------------------------------------------------------------
    drone1=DroneConfig(
        initial=DroneInitialState(
            x=1.6,
            y=-0.5,
            z=1.0,  # Flying altitude (meters)
            yaw=-170,
        ),
        noise=DroneNoiseParams(
            enable_process_noise=True,
            enable_sensor_noise=True,
            # Process noise
            process_vx_std=0.01,
            process_vy_std=0.05,
            process_yaw_rate_std=2.0,
            process_vy_bias=0.05,
            # Sensor noise
            uwb_distance_std=0.05,
            uwb_distance_bias=0.0,
        ),
    ),
    
    # -------------------------------------------------------------------------
    # DRONE 2 CONFIGURATION
    # -------------------------------------------------------------------------
    drone2=DroneConfig(
        initial=DroneInitialState(
            x=-0.5,
            y=0.3,
            z=1.0,  # Flying altitude (meters)
            yaw=-10.2283,
        ),
        noise=DroneNoiseParams(
            enable_process_noise=True,
            enable_sensor_noise=True,
            # Process noise
            process_vx_std=0.01,
            process_vy_std=0.05,
            process_yaw_rate_std=2.0,
            process_vy_bias=0.0,
            # Sensor noise
            uwb_distance_std=0.05,
            uwb_distance_bias=0.0,
        ),
    ),
)
