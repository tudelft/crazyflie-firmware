"""
Drone physics model for simulation.

This module contains the Drone class which handles the physical state
and kinematics of a drone. The physics model is kept simple and modular
so it can be extended with dynamics later. The simulation uses 3D positions
but with fixed altitude for proper distance calculations.
"""
import math
import random
from dataclasses import dataclass, field
from typing import Tuple, List, Protocol, Optional
from enum import Enum, auto


class DroneState(Enum):
    """Flight state of the drone."""
    IDLE = auto()
    FLYING = auto()
    LANDED = auto()


@dataclass
class DronePhysicsState:
    """Physical state of a drone in 3D (with fixed altitude)."""
    x: float = 0.0  # Position X (meters)
    y: float = 0.0  # Position Y (meters)
    z: float = 0.0  # Position Z / altitude (meters)
    yaw: float = 0.0  # Heading (degrees, 0 = +X axis, CCW positive)
    vx: float = 0.0  # Velocity X (m/s) - world frame
    vy: float = 0.0  # Velocity Y (m/s) - world frame
    yaw_rate: float = 0.0  # Yaw rate (deg/s)


class PhysicsModel(Protocol):
    """Protocol for physics models (allows swapping different dynamics)."""
    
    def update(
        self,
        state: DronePhysicsState,
        cmd_vx_body: float,
        cmd_vy_body: float,
        cmd_yaw_rate: float,
        dt: float
    ) -> DronePhysicsState:
        """Update physics state based on commands."""
        ...


class KinematicPhysics:
    """
    Simple kinematic physics model.
    
    Commands translate directly to state without any dynamics/lag.
    This can be replaced with a more complex model later.
    """
    
    def update(
        self,
        state: DronePhysicsState,
        cmd_vx_body: float,
        cmd_vy_body: float,
        cmd_yaw_rate: float,
        dt: float
    ) -> DronePhysicsState:
        """
        Update physics state based on velocity commands.
        
        Args:
            state: Current physics state
            cmd_vx_body: Commanded forward velocity (m/s) in body frame
            cmd_vy_body: Commanded lateral velocity (m/s) in body frame
            cmd_yaw_rate: Commanded yaw rate (deg/s)
            dt: Time step (seconds)
            
        Returns:
            New physics state
        """
        # Convert yaw to radians for trig
        yaw_rad = math.radians(state.yaw)
        
        # Transform body velocities to world frame
        vx_world = cmd_vx_body * math.cos(yaw_rad) - cmd_vy_body * math.sin(yaw_rad)
        vy_world = cmd_vx_body * math.sin(yaw_rad) + cmd_vy_body * math.cos(yaw_rad)
        
        # Update state (z remains constant - altitude hold)
        new_state = DronePhysicsState(
            x=state.x + vx_world * dt,
            y=state.y + vy_world * dt,
            z=state.z,
            yaw=self._normalize_angle(state.yaw + cmd_yaw_rate * dt),
            vx=vx_world,
            vy=vy_world,
            yaw_rate=cmd_yaw_rate
        )
        
        return new_state
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-180, 180] degrees."""
        while angle > 180:
            angle -= 360
        while angle <= -180:
            angle += 360
        return angle


class NoisyKinematicPhysics:
    """
    Kinematic physics model with process noise.
    
    Models imperfect velocity tracking due to estimation errors.
    Noise is added to the commanded velocities before integration.
    The lateral velocity bias follows a random walk with bounds.
    """
    
    def __init__(
        self,
        vx_std: float = 0.02,
        vy_std: float = 0.05,
        yaw_rate_std: float = 2.0,
        vy_bias: float = 0.0,
        vy_bias_max: float = 0.15,
        vy_bias_walk_std: float = 0.005
    ):
        """
        Initialize noisy physics model.
        
        Args:
            vx_std: Forward velocity noise std dev (m/s)
            vy_std: Lateral velocity noise std dev (m/s)
            yaw_rate_std: Yaw rate noise std dev (deg/s)
            vy_bias: Initial lateral velocity bias (m/s)
            vy_bias_max: Maximum absolute value for vy_bias random walk (m/s)
            vy_bias_walk_std: Std dev for random walk step per update (m/s)
        """
        self.vx_std = vx_std
        self.vy_std = vy_std
        self.yaw_rate_std = yaw_rate_std
        self.vy_bias = vy_bias
        self.vy_bias_max = vy_bias_max
        self.vy_bias_walk_std = vy_bias_walk_std
    
    def update(
        self,
        state: DronePhysicsState,
        cmd_vx_body: float,
        cmd_vy_body: float,
        cmd_yaw_rate: float,
        dt: float
    ) -> DronePhysicsState:
        """
        Update physics state with noisy velocity commands.
        
        Args:
            state: Current physics state
            cmd_vx_body: Commanded forward velocity (m/s) in body frame
            cmd_vy_body: Commanded lateral velocity (m/s) in body frame
            cmd_yaw_rate: Commanded yaw rate (deg/s)
            dt: Time step (seconds)
            
        Returns:
            New physics state
        """
        # Update vy_bias with random walk (bounded)
        self.vy_bias += random.gauss(0, self.vy_bias_walk_std)
        self.vy_bias = max(-self.vy_bias_max, min(self.vy_bias_max, self.vy_bias))
        
        # Add process noise to commands (in body frame)
        noisy_vx = cmd_vx_body + random.gauss(0, self.vx_std)
        noisy_vy = cmd_vy_body + random.gauss(0, self.vy_std) + self.vy_bias
        noisy_yaw_rate = cmd_yaw_rate + random.gauss(0, self.yaw_rate_std)
        
        # Convert yaw to radians for trig
        yaw_rad = math.radians(state.yaw)
        
        # Transform body velocities to world frame
        vx_world = noisy_vx * math.cos(yaw_rad) - noisy_vy * math.sin(yaw_rad)
        vy_world = noisy_vx * math.sin(yaw_rad) + noisy_vy * math.cos(yaw_rad)
        
        # Update state (z remains constant - altitude hold)
        new_state = DronePhysicsState(
            x=state.x + vx_world * dt,
            y=state.y + vy_world * dt,
            z=state.z,
            yaw=self._normalize_angle(state.yaw + noisy_yaw_rate * dt),
            vx=vx_world,
            vy=vy_world,
            yaw_rate=noisy_yaw_rate
        )
        
        return new_state
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-180, 180] degrees."""
        while angle > 180:
            angle -= 360
        while angle <= -180:
            angle += 360
        return angle


class Drone:
    """
    Represents a drone in the simulation.
    
    Handles physical state, position history for trails, and distance calculations.
    The physics model is pluggable for easy extension.
    """
    
    def __init__(
        self,
        drone_id: int,
        initial_x: float = 0.0,
        initial_y: float = 0.0,
        initial_z: float = 0.0,
        initial_yaw: float = 0.0,
        physics_model: PhysicsModel = None,
        trail_length: int = 200
    ):
        """
        Initialize a drone.
        
        Args:
            drone_id: Unique identifier (1 or 2 for the two-drone scenario)
            initial_x: Initial X position (meters)
            initial_y: Initial Y position (meters)
            initial_z: Initial Z position / altitude (meters)
            initial_yaw: Initial heading (degrees)
            physics_model: Physics model to use (default: KinematicPhysics)
            trail_length: Number of positions to store for trail visualization
        """
        self.drone_id = drone_id
        self.physics = physics_model or KinematicPhysics()
        self.trail_length = trail_length
        
        # Initialize state
        self.state = DronePhysicsState(
            x=initial_x,
            y=initial_y,
            z=initial_z,
            yaw=initial_yaw
        )
        self.flight_state = DroneState.IDLE
        
        # Position history for trail
        self.trail: List[Tuple[float, float]] = []
        
    def update(
        self,
        cmd_vx_body: float,
        cmd_vy_body: float,
        cmd_yaw_rate: float,
        dt: float
    ) -> None:
        """
        Update drone state based on velocity commands.
        
        Args:
            cmd_vx_body: Commanded forward velocity (m/s) in body frame
            cmd_vy_body: Commanded lateral velocity (m/s) in body frame
            cmd_yaw_rate: Commanded yaw rate (deg/s)
            dt: Time step (seconds)
        """
        if self.flight_state != DroneState.FLYING:
            return
            
        # Store position for trail
        self.trail.append((self.state.x, self.state.y))
        if len(self.trail) > self.trail_length:
            self.trail.pop(0)
        
        # Update physics
        self.state = self.physics.update(
            self.state,
            cmd_vx_body,
            cmd_vy_body,
            cmd_yaw_rate,
            dt
        )
    
    def takeoff(self) -> None:
        """Start flying."""
        self.flight_state = DroneState.FLYING
        self.trail.clear()
    
    def land(self) -> None:
        """Land the drone."""
        self.flight_state = DroneState.LANDED
        self.state.vx = 0.0
        self.state.vy = 0.0
        self.state.yaw_rate = 0.0
        self.state.z = 0.0
    
    def distance_to(self, x: float, y: float, z: float = None) -> float:
        """Calculate 3D distance to a point.
        
        Args:
            x: X coordinate of the point
            y: Y coordinate of the point
            z: Z coordinate of the point (if None, uses drone's z for 2D distance)
        """
        dx = self.state.x - x
        dy = self.state.y - y
        if z is None:
            dz = 0.0
        else:
            dz = self.state.z - z
        return math.sqrt(dx * dx + dy * dy + dz * dz)
    
    def distance_to_drone(self, other: 'Drone') -> float:
        """Calculate 3D distance to another drone."""
        return self.distance_to(other.state.x, other.state.y, other.state.z)
    
    @property
    def position(self) -> Tuple[float, float]:
        """Get current position."""
        return (self.state.x, self.state.y)
    
    @property
    def yaw(self) -> float:
        """Get current heading in degrees."""
        return self.state.yaw
    
    @property
    def is_flying(self) -> bool:
        """Check if drone is currently flying."""
        return self.flight_state == DroneState.FLYING
    
    def reset(self, x: float, y: float, yaw: float, z: float = None) -> None:
        """Reset drone to initial state.
        
        Args:
            x: X position (meters)
            y: Y position (meters)
            yaw: Heading (degrees)
            z: Z position / altitude (meters), if None keeps current z
        """
        current_z = self.state.z if z is None else z
        self.state = DronePhysicsState(x=x, y=y, z=current_z, yaw=yaw)
        self.flight_state = DroneState.IDLE
        self.trail.clear()


class UWBSensor:
    """
    UWB distance sensor model with noise.
    
    Models the noise characteristics of UWB ranging measurements
    including Gaussian noise, bias.
    """
    
    def __init__(
        self,
        distance_std: float = 0.05,
        distance_bias: float = 0.0,
        enabled: bool = True
    ):
        """
        Initialize UWB sensor model.
        
        Args:
            distance_std: Distance measurement noise std dev (meters)
            distance_bias: Systematic measurement bias (meters)
            enabled: Whether noise is enabled
        """
        self.distance_std = distance_std
        self.distance_bias = distance_bias
        self.enabled = enabled
    
    def measure(self, true_distance: float) -> float:
        """
        Get a noisy distance measurement.
        
        Args:
            true_distance: Actual distance (meters)
            
        Returns:
            Noisy distance measurement (meters), always >= 0
        """
        if not self.enabled:
            return true_distance
        
        # Start with true distance plus bias
        measurement = true_distance + self.distance_bias
        
        # Add Gaussian noise
        measurement += random.gauss(0, self.distance_std)
        
        # Distance can't be negative
        return max(0.0, measurement)
    
    def measure_distance_between(
        self,
        x1: float, y1: float, z1: float,
        x2: float, y2: float, z2: float
    ) -> float:
        """
        Measure noisy 3D distance between two points.
        
        Args:
            x1, y1, z1: First point coordinates
            x2, y2, z2: Second point coordinates
            
        Returns:
            Noisy distance measurement (meters)
        """
        true_distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
        return self.measure(true_distance)
