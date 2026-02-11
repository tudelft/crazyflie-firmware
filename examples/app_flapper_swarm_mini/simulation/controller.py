"""
Flight controller for the drone swarm simulation.

This module implements the control logic from flapper_swarm.c as a proper state machine.
The controller mirrors the C firmware's state machine structure with:
  - STATE_STRAIGHT: Flying straight forward
  - STATE_TURN: Turning at boundary (90° arc)
  - STATE_AVOID: Avoiding peer drone
  - STATE_RECOVER: Recovery mode to return to inner circle

Each state has: on_enter, on_exit, check_transition, execute
"""
import math
from dataclasses import dataclass, field
from typing import Tuple, Optional
from enum import Enum, auto
from collections import deque

from .config import FlightParams, SimulationParams


class FlightState(Enum):
    """Flight state of the drone (matching C enum)."""
    STRAIGHT = 0
    TURN = 1
    AVOID = 2
    RECOVER = 3
    DANCE = 4

class LandingReason(Enum):
        """Reason for emergency landing."""
        NONE = auto()
        OUT_OF_BOUNDS = auto()
        COLLISION = auto()

@dataclass
class ControlCommand:
    """Velocity command output from the controller."""
    vx_body: float = 0.0  # Forward velocity (m/s)
    vy_body: float = 0.0  # Lateral velocity (m/s)
    yaw_rate: float = 0.0  # Yaw rate (deg/s)


@dataclass
class StateContext:
    """State machine context (shared between state handlers)."""
    # Arc tracking (TURN state)
    arc_active: bool = False
    arc_cooldown: bool = False
    arc_yaw_start: float = 0.0
    target_yaw: float = 0.0
    rotation_direction: int = 1  # Matches C: rotationDirection
    
    # Routine counter
    routines: int = 0
    
    # Current sensor readings (updated each loop)
    d0: float = 0.0  # Distance to beacon (meters)
    d0_deriv: float = 0.0  # Derivative of d0 (m/s)
    peer_dist: float = 0.0  # Distance to peer drone (meters)
    peer_dist_deriv: float = 0.0  # Derivative of peer distance (m/s, negative = closing)
    peer_z: float = 1.0  # Peer drone's z-height (meters), default to flying height
    
    # Dance state tracking (matching C firmware)
    dance_has_landed: bool = False
    dance_has_taken_off: bool = False


class DerivativeEstimator:
    """
    Estimates derivative of distance using linear regression.
    
    Uses a circular buffer of samples to compute the slope via least squares.
    Matches the C firmware's d0BufferGetDerivative() function.
    """
    
    def __init__(self, buffer_size: int = 10, sample_interval: float = 0.02):
        """
        Initialize the derivative estimator.
        
        Args:
            buffer_size: Number of samples to keep
            sample_interval: Minimum time between samples (seconds)
        """
        self.buffer_size = buffer_size
        self.sample_interval = sample_interval
        self.samples: deque = deque(maxlen=buffer_size)
        self.last_sample_time: float = -float('inf')
    
    def add_sample(self, value: float, time: float) -> None:
        """
        Add a sample if enough time has passed.
        
        Args:
            value: Distance value (meters)
            time: Current time (seconds)
        """
        if time - self.last_sample_time >= self.sample_interval:
            self.samples.append(value)
            self.last_sample_time = time
    
    def get_derivative(self) -> float:
        """
        Compute derivative using linear regression.
        
        Returns:
            Derivative in m/s (positive = moving away, negative = moving toward)
        """
        n = len(self.samples)
        if n < 2:
            return 0.0
        
        # Compute sums for least squares
        sum_t = 0.0
        sum_y = 0.0
        sum_ty = 0.0
        sum_t2 = 0.0
        
        for i, y in enumerate(self.samples):
            t = float(i)
            sum_t += t
            sum_y += y
            sum_ty += t * y
            sum_t2 += t * t
        
        denom = n * sum_t2 - sum_t * sum_t
        if abs(denom) < 1e-6:
            return 0.0
        
        slope_per_interval = (n * sum_ty - sum_t * sum_y) / denom
        
        # Convert to per-second rate
        return slope_per_interval / self.sample_interval
    
    def reset(self) -> None:
        """Clear the buffer."""
        self.samples.clear()
        self.last_sample_time = -float('inf')


class SwarmController:
    """
    Controller implementing the flapper swarm algorithm as a state machine.
    
    Each drone has its own controller instance. The controller makes decisions
    based on distance to beacon and distance to peer drone.
    
    State machine structure matches flapper_swarm.c exactly.
    """
    
    def __init__(
        self,
        drone_id: int,
        params: FlightParams,
        sim_params: SimulationParams
    ):
        """
        Initialize the controller.
        
        Args:
            drone_id: Drone identifier (1 or 2)
            params: Flight parameters
            sim_params: Simulation parameters
        """
        self.drone_id = drone_id
        self.params = params
        self.sim_params = sim_params
        
        # Current state
        self.current_state = FlightState.STRAIGHT
        
        # State context (shared data)
        self.ctx = StateContext()
        
        # Confirmation counters
        self.abort_over_count = 0
        self.inner_over_count = 0
        self.inner_under_count = 0
        self.approach_count = 0
        self.depart_count = 0
        
        # Dance state counters (matching C firmware)
        self.peer_landed_count = 0
        self.rejoin_count = 0
        
        # Constants matching C firmware
        self.PEER_LANDED_HEIGHT_M = 0.1
        self.PEER_LANDED_CONFIRM_COUNT = 3
        self.REJOIN_EXTRA_MM = 0.05  # 50mm in meters
        self.REJOIN_CONFIRM_COUNT = 3

        
        # Derivative estimator for beacon distance
        self.d0_deriv = DerivativeEstimator(
            buffer_size=sim_params.deriv_buffer_size,
            sample_interval=sim_params.deriv_sample_interval_s
        )
        
        # Derivative estimator for peer distance (for cooperative avoidance)
        self.peer_dist_deriv = DerivativeEstimator(
            buffer_size=sim_params.deriv_buffer_size,
            sample_interval=sim_params.deriv_sample_interval_s
        )
        
        # Compute recover factor (matching C code: recoverFactor = -(RECOVER_YAWRATE / DES_DERIV))
        self.recover_factor = -(params.recover_yaw_rate_dps / params.des_deriv_mps)
    def reset(self) -> None:
        """Reset controller state."""
        self.current_state = FlightState.STRAIGHT
        self.ctx = StateContext()
        self.ctx.rotation_direction = 1  # Default rotation direction
        self.abort_over_count = 0
        self.inner_over_count = 0
        self.inner_under_count = 0
        self.approach_count = 0
        self.depart_count = 0
        self.peer_landed_count = 0
        self.rejoin_count = 0
        self.d0_deriv.reset()
        self.peer_dist_deriv.reset()
        
        # Call onEnter for initial state
        self._on_enter_state(self.current_state)
    
    def update(
        self,
        d0: float,  # Distance to beacon (meters)
        peer_dist: float,  # Distance to peer drone (meters)
        current_yaw: float,  # Current heading (degrees)
        time: float,  # Current simulation time (seconds)
        peer_z: float = 1.0  # Peer drone's z-height (meters)
    ) -> Tuple[ControlCommand, bool]:
        """
        Update controller and get command.
        
        Args:
            d0: Distance to beacon (meters)
            peer_dist: Distance to peer drone (meters)
            current_yaw: Current heading (degrees)
            time: Current simulation time (seconds)
            
        Returns:
            Tuple of (ControlCommand, should_land)
        """
        p = self.params
        
        # Update derivative buffers
        self.d0_deriv.add_sample(d0, time)
        self.peer_dist_deriv.add_sample(peer_dist, time)
        
        # Update context with current sensor readings
        self.ctx.d0 = d0
        self.ctx.d0_deriv = self.d0_deriv.get_derivative()
        self.ctx.peer_dist = peer_dist
        self.ctx.peer_z = peer_z  # Store peer z-height
        self.ctx.peer_dist_deriv = self.peer_dist_deriv.get_derivative()
        
        # Check emergency conditions
        emergency_reason = self._check_emergency(d0, peer_dist)
        if emergency_reason != LandingReason.NONE:
            return ControlCommand(), emergency_reason
        
        # Clear arc cooldown once we re-enter the inner circle
        if self.ctx.arc_cooldown and d0 > 0 and d0 <= p.inner_bound_m:
            self.ctx.arc_cooldown = False
        
        # State machine: check transitions
        next_state = self._check_transition(self.current_state, current_yaw)
        
        if next_state != self.current_state:
            self._on_exit_state(self.current_state)
            self.current_state = next_state
            self._on_enter_state(self.current_state, current_yaw)
        
        # Execute current state
        cmd = self._execute_state(self.current_state)
        
        return cmd, emergency_reason
    
    # =========================================================================
    # Emergency checks
    # =========================================================================
    def _check_emergency(self, d0: float, peer_dist: float) -> LandingReason:
        """
        Check emergency conditions (matching C firmware).
        
        Returns:
            LandingReason if should emergency land, otherwise NONE
        """
        p = self.params
        
        # Check outer boundary (distance0 abort)
        # NOTE: In C firmware, this is NOT checked during DANCE state for drone 1
        if self.current_state != FlightState.DANCE or self.drone_id != 1:
            if d0 > p.dist0_abort_m:
                self.abort_over_count += 1
                if self.abort_over_count >= p.abort_confirm_count:
                    return LandingReason.OUT_OF_BOUNDS
            else:
                self.abort_over_count = 0
        
        # NOTE: In C firmware, peer collision during AVOID triggers DANCE state,
        # not emergency landing. This is handled in _check_transition_avoid().
        
        return LandingReason.NONE
    
    # =========================================================================
    # Avoidance confirmation logic (matching C firmware)
    # =========================================================================
    
    def _is_peer_landed(self) -> bool:
        """Check if peer drone has landed (z < threshold)."""
        return self.ctx.peer_z >= 0.0 and self.ctx.peer_z < self.PEER_LANDED_HEIGHT_M
    
    def _should_enter_avoid(self) -> bool:
        """Check if peer is too close and we should enter AVOID state (with confirmation)."""
        p = self.params
        peer_dist = self.ctx.peer_dist
        
        # Don't enter avoid if peer has landed (z < 0.1m)
        if self._is_peer_landed():
            self.approach_count = 0  # Reset counter since we're not tracking
            return False
        
        if peer_dist > 0 and peer_dist <= p.peer_close_m:
            self.approach_count += 1
            if self.approach_count >= p.avoid_enter_confirm_count:
                self.approach_count = 0
                self.depart_count = 0
                return True
        else:
            self.approach_count = 0
        return False
    
    def _should_exit_avoid(self) -> bool:
        """Check if peer is far enough and we should exit AVOID state (with confirmation)."""
        p = self.params
        peer_dist = self.ctx.peer_dist
        
        # Exit avoid immediately if peer has landed (no collision risk)
        if self._is_peer_landed():
            self.depart_count = 0
            self.approach_count = 0
            return True
        
        if peer_dist >= p.peer_close_m:
            self.depart_count += 1
            if self.depart_count >= p.avoid_exit_confirm_count:
                self.depart_count = 0
                self.approach_count = 0
                return True
        else:
            self.depart_count = 0
        return False
    
    def _should_enter_dance(self) -> bool:
        """Check if we should enter DANCE state (peer dangerously close during AVOID)."""
        p = self.params
        peer_dist = self.ctx.peer_dist
        return (self.current_state == FlightState.AVOID and 
                peer_dist > 0 and peer_dist <= p.avoid_min_land_m)
    
    # =========================================================================
    # State machine: on_enter handlers
    # =========================================================================
    
    def _on_enter_state(self, state: FlightState, current_yaw: float = 0.0) -> None:
        """Called when entering a state."""
        if state == FlightState.STRAIGHT:
            self._on_enter_straight()
        elif state == FlightState.TURN:
            self._on_enter_turn(current_yaw)
        elif state == FlightState.AVOID:
            self._on_enter_avoid()
        elif state == FlightState.RECOVER:
            self._on_enter_recover()
        elif state == FlightState.DANCE:
            self._on_enter_dance()
    
    def _on_enter_straight(self) -> None:
        """Enter STRAIGHT state."""
        self.inner_over_count = 0
    
    def _on_enter_turn(self, current_yaw: float) -> None:
        """Enter TURN state."""
        self.inner_under_count = 0
        self.ctx.arc_yaw_start = current_yaw
        # C firmware: ctx.targetYaw = normalizeAngle(ctx.arcYawStart - 110.0f);
        self.ctx.target_yaw = self._normalize_angle(current_yaw - 110.0)
        self.ctx.arc_active = math.isfinite(current_yaw)
    
    def _on_enter_avoid(self) -> None:
        """Enter AVOID state."""
        self.peer_landed_count = 0
    
    def _on_enter_recover(self) -> None:
        """Enter RECOVER state."""
        # C firmware: if (droneId != 1) { if (ctx.d0 > dist0AbortMm - 800) rotationDirection = -1; }
        if self.drone_id != 1:
            if self.ctx.d0 > self.params.dist0_abort_m - 0.8:  # 800mm = 0.8m
                self.ctx.rotation_direction = -1
    
    def _on_enter_dance(self) -> None:
        """Enter DANCE state."""
        self.peer_landed_count = 0
        self.rejoin_count = 0
        self.ctx.dance_has_landed = False
        self.ctx.dance_has_taken_off = False
    
    # =========================================================================
    # State machine: on_exit handlers
    # =========================================================================
    
    def _on_exit_state(self, state: FlightState) -> None:
        """Called when exiting a state."""
        if state == FlightState.STRAIGHT:
            self._on_exit_straight()
        elif state == FlightState.TURN:
            self._on_exit_turn()
        elif state == FlightState.AVOID:
            self._on_exit_avoid()
        elif state == FlightState.RECOVER:
            self._on_exit_recover()
        elif state == FlightState.DANCE:
            self._on_exit_dance()
    
    def _on_exit_straight(self) -> None:
        """Exit STRAIGHT state."""
        pass  # Nothing special in C
    
    def _on_exit_turn(self) -> None:
        """Exit TURN state."""
        self.ctx.arc_active = False
    
    def _on_exit_avoid(self) -> None:
        """Exit AVOID state."""
        self.approach_count = 0
        self.depart_count = 0
        self.peer_landed_count = 0
    
    def _on_exit_recover(self) -> None:
        """Exit RECOVER state."""
        # C firmware: ctx.rotationDirection = 1;
        self.ctx.rotation_direction = 1
    
    def _on_exit_dance(self) -> None:
        """Exit DANCE state."""
        self.peer_landed_count = 0
        self.rejoin_count = 0
        self.ctx.dance_has_landed = False
        self.ctx.dance_has_taken_off = False
    
    # =========================================================================
    # State machine: check_transition handlers
    # =========================================================================
    
    def _check_transition(self, state: FlightState, current_yaw: float) -> FlightState:
        """Check for state transitions."""
        if state == FlightState.STRAIGHT:
            return self._check_transition_straight()
        elif state == FlightState.TURN:
            return self._check_transition_turn(current_yaw)
        elif state == FlightState.AVOID:
            return self._check_transition_avoid()
        elif state == FlightState.RECOVER:
            return self._check_transition_recover()
        elif state == FlightState.DANCE:
            return self._check_transition_dance()
        return state
    
    def _check_transition_straight(self) -> FlightState:
        """Check transitions from STRAIGHT state."""
        p = self.params
        ctx = self.ctx
        
        # STRAIGHT -> AVOID: peer too close
        if self._should_enter_avoid():
            return FlightState.AVOID
        
        # STRAIGHT -> TURN: reached outer bound (only if not in arc cooldown)
        if ctx.d0 >= p.inner_bound_m and not ctx.arc_cooldown:
            self.inner_over_count += 1
            if self.inner_over_count >= p.abort_confirm_count:
                return FlightState.TURN
        elif ctx.d0 < p.inner_bound_m:
            self.inner_over_count = 0
        
        # STRAIGHT -> RECOVER: outside inner bound and moving away from beacon (during cooldown)
        if ctx.d0 >= p.inner_bound_m and ctx.d0_deriv > 0 and ctx.arc_cooldown:
            return FlightState.RECOVER
        
        return FlightState.STRAIGHT
    
    def _check_transition_turn(self, current_yaw: float) -> FlightState:
        """Check transitions from TURN state."""
        p = self.params
        ctx = self.ctx
        
        # TURN -> AVOID: peer too close
        if self._should_enter_avoid():
            return FlightState.AVOID
        
        # TURN -> STRAIGHT: back inside inner bound
        if ctx.d0 <= p.inner_bound_m:
            self.inner_under_count += 1
            if self.inner_under_count >= p.abort_confirm_count:
                ctx.routines += 1
                return FlightState.STRAIGHT
        else:
            self.inner_under_count = 0
        
        # Arc tracking: when 90° reached, decide STRAIGHT or RECOVER
        if ctx.arc_active and math.isfinite(current_yaw):
            yaw_diff = abs(current_yaw - ctx.target_yaw)
            # Handle wrap-around
            if yaw_diff > 180:
                yaw_diff = 360 - yaw_diff
            
            reached = yaw_diff <= 3.0
            if reached:
                ctx.arc_active = False
                ctx.arc_cooldown = True
                if ctx.d0_deriv <= 0:
                    return FlightState.STRAIGHT
                else:
                    return FlightState.RECOVER
        elif ctx.arc_active:
            ctx.arc_active = False  # Yaw unavailable
        
        return FlightState.TURN
    
    def _check_transition_avoid(self) -> FlightState:
        """Check transitions from AVOID state."""
        p = self.params
        ctx = self.ctx
        
        # AVOID -> DANCE: peer dangerously close (emergency zone)
        if self._should_enter_dance():
            return FlightState.DANCE
        
        # AVOID -> STRAIGHT or RECOVER: peer far enough
        if self._should_exit_avoid():
            if ctx.d0 < p.inner_bound_m:
                return FlightState.STRAIGHT
            else:
                return FlightState.RECOVER
        
        return FlightState.AVOID
    
    def _check_transition_dance(self) -> FlightState:
        """Check transitions from DANCE state."""
        p = self.params
        ctx = self.ctx
        
        if self.drone_id == 1:
            # Drone 1: Exit after we've landed AND taken off again
            if ctx.dance_has_taken_off:
                if ctx.d0 < p.inner_bound_m:
                    return FlightState.STRAIGHT
                else:
                    return FlightState.RECOVER
        else:
            # Drone 2+: Exit once peer (drone 1) has landed
            if self._is_peer_landed():
                self.peer_landed_count += 1
            else:
                self.peer_landed_count = 0
            
            if self.peer_landed_count >= self.PEER_LANDED_CONFIRM_COUNT:
                if ctx.d0 < p.inner_bound_m:
                    return FlightState.STRAIGHT
                else:
                    return FlightState.RECOVER
        
        return FlightState.DANCE
    
    def _check_transition_recover(self) -> FlightState:
        """Check transitions from RECOVER state."""
        p = self.params
        ctx = self.ctx
        
        # RECOVER -> AVOID: peer too close
        if self._should_enter_avoid():
            return FlightState.AVOID
        
        # RECOVER -> STRAIGHT: back inside inner bound
        if ctx.d0 > 0 and ctx.d0 <= p.inner_bound_m:
            return FlightState.STRAIGHT
        
        return FlightState.RECOVER
    
    # =========================================================================
    # State machine: execute handlers
    # =========================================================================
    
    def _execute_state(self, state: FlightState) -> ControlCommand:
        """Execute current state and return command."""
        if state == FlightState.STRAIGHT:
            return self._execute_straight()
        elif state == FlightState.TURN:
            return self._execute_turn()
        elif state == FlightState.AVOID:
            return self._execute_avoid()
        elif state == FlightState.RECOVER:
            return self._execute_recover()
        elif state == FlightState.DANCE:
            return self._execute_dance()
        return ControlCommand()
    
    def _execute_straight(self) -> ControlCommand:
        """Execute STRAIGHT state."""
        return ControlCommand(
            vx_body=self.params.fwd_speed_mps,
            vy_body=0.0,
            yaw_rate=0.0
        )
    
    def _execute_turn(self) -> ControlCommand:
        """Execute TURN state."""
        # C firmware: modulate yaw rate based on derivative
        # if (ctx.d0Deriv < -50.0f) { yawRateCmd *= 0.5f; }
        # Note: In C, d0Deriv is in mm/s, so -50.0f mm/s = -0.05 m/s
        yaw_rate_cmd = self.params.turn_yaw_rate_dps
        if self.ctx.d0_deriv < -0.05:  # -50 mm/s in m/s
            yaw_rate_cmd *= 0.5
        
        return ControlCommand(
            vx_body=self.params.fwd_speed_mps,
            vy_body=0.0,
            yaw_rate=yaw_rate_cmd
        )
    
    def _execute_avoid(self) -> ControlCommand:
        """Execute AVOID state."""
        return ControlCommand(
            vx_body=self.params.fwd_speed_mps * self.params.avoid_speed_factor,
            vy_body=0.0,
            yaw_rate=self._get_avoid_yaw_rate()
        )
    
    def _execute_recover(self) -> ControlCommand:
        """Execute RECOVER state."""
        p = self.params
        ctx = self.ctx
        
        # C firmware:
        # float yawCommand = recoverFactor * fabsf(ctx.d0Deriv - DES_DERIV);
        # yawCommand = yawCommand > RECOVER_DEADZONE ? yawCommand : 0.0f;
        # yawCommand = fminf(yawCommand, 70.0f);
        # float yawCommandFinal = yawCommand * ctx.rotationDirection;
        yaw_cmd = self.recover_factor * abs(ctx.d0_deriv - p.des_deriv_mps)
        
        # Apply deadzone (before direction)
        if yaw_cmd <= p.recover_deadzone_dps:
            yaw_cmd = 0.0
        
        # Apply max cap
        yaw_cmd = min(yaw_cmd, 70.0)
        
        # Apply rotation direction
        yaw_cmd_final = yaw_cmd * ctx.rotation_direction
        
        return ControlCommand(
            vx_body=p.fwd_speed_mps,
            vy_body=0.0,
            yaw_rate=yaw_cmd_final
        )
    
    def _execute_dance(self) -> ControlCommand:
        """Execute DANCE state."""
        p = self.params
        ctx = self.ctx
        
        if self.drone_id == 1:
            # Drone 1: Land, wait for peer to move away, then take off
            # In simulation, we simplify this - drone just hovers/freezes
            # The actual landing/takeoff is handled by the simulator
            
            if not ctx.dance_has_landed:
                # Phase 1: Signal to land (simulator will handle actual landing)
                # For now, just freeze
                ctx.dance_has_landed = True
                return ControlCommand(vx_body=0.0, vy_body=0.0, yaw_rate=0.0)
            
            elif not ctx.dance_has_taken_off:
                # Phase 2: Wait for peer to move away
                peer_dist = ctx.peer_dist
                rejoin_thresh = p.peer_close_m + self.REJOIN_EXTRA_MM
                
                if peer_dist >= rejoin_thresh:
                    self.rejoin_count += 1
                else:
                    self.rejoin_count = 0
                
                if self.rejoin_count >= self.REJOIN_CONFIRM_COUNT:
                    ctx.dance_has_taken_off = True
                
                return ControlCommand(vx_body=0.0, vy_body=0.0, yaw_rate=0.0)
            
            # Phase 3: Takeoff complete, checkTransitionDance will handle exit
            return ControlCommand(vx_body=0.0, vy_body=0.0, yaw_rate=0.0)
        
        else:
            # Drone 2+: Just freeze and wait for peer to land
            return ControlCommand(vx_body=0.0, vy_body=0.0, yaw_rate=0.0)
    
    # =========================================================================
    # Helper methods
    # =========================================================================

    def _get_avoid_yaw_rate(self) -> float:
        """Get avoidance yaw rate based on drone ID.""" 
        # C firmware: drone 1 = positive (CW), drone 2+ = negative (CCW)
        if self.drone_id == 1:
            return self.params.avoid_yaw_rate_dps  # CW (positive)
        else:
            return -self.params.avoid_yaw_rate_dps  # CCW (negative)
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-180, 180] degrees."""
        while angle > 180:
            angle -= 360
        while angle <= -180:
            angle += 360
        return angle
    
    # =========================================================================
    # Properties for external access
    # =========================================================================
    
    @property
    def mode(self) -> FlightState:
        """Get current flight state."""
        return self.current_state
    
    @property
    def is_avoiding(self) -> bool:
        """Check if currently in AVOID state."""
        return self.current_state == FlightState.AVOID
    
    @property
    def is_dancing(self) -> bool:
        """Check if currently in DANCE state."""
        return self.current_state == FlightState.DANCE


# Keep FlightMode as alias for backward compatibility with simulator
FlightMode = FlightState
