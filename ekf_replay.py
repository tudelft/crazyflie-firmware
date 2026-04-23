"""
Crazyflie EKF replay from logged CSV data.

Reimplements the estimator from:
  src/modules/src/kalman_core/kalman_core.c
  src/modules/src/kalman_core/mm_flow.c
  src/modules/src/kalman_core/mm_tof.c

Expected CSV columns (logged at imu_rate, typically 500 Hz):
  acc.x, acc.y, acc.z       [g units]
  gyro.x, gyro.y, gyro.z   [deg/s]
  range.zrange              [mm, uint16 — convert to m internally]
  motion.deltaX             [raw flow pixels, 10x scale]
  motion.deltaY             [raw flow pixels, 10x scale]

Flow and ToF arrive at a lower rate; new measurements are detected
by value changes between consecutive rows.

Usage:
  python ekf_replay.py flight.csv
  python ekf_replay.py flight.csv --imu-rate 500 --no-plot
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

GRAVITY: float = 9.81          # m/s^2
DEG_TO_RAD: float = np.pi / 180.0
FLOW_RESOLUTION: float = 0.1  # sensor reports 10x pixel motion; scale back
FLOW_OUTLIER_LIMIT: int = 100  # raw pixels; matching OULIER_LIMIT in flowdeck_v1v2.c

# State vector indices
IX, IY, IZ = 0, 1, 2           # position in world frame [m]
IPX, IPY, IPZ = 3, 4, 5        # velocity in body frame [m/s]
ID0, ID1, ID2 = 6, 7, 8        # attitude error (Rodrigues parameters)
N_STATES: int = 9

_DIAG = np.diag_indices(N_STATES)


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------

@dataclass
class EKFParams:
    """All tunable parameters matching kalmanCoreParams_t and sensor defaults."""

    # --- Initial state ---
    initial_x: float = 0.0
    initial_y: float = 0.0
    initial_z: float = 0.0
    initial_yaw: float = 0.0    # radians; 0 = facing +X

    # --- Initial covariance (std devs) ---
    std_xy0: float = 0.05      # m
    std_z0: float = .05         # m
    std_vel0: float = 0.01      # m/s
    std_att_rp0: float = 0.01   # rad
    std_att_yaw0: float = 0.01  # rad

    # --- Process noise ---
    proc_noise_acc_xy: float = 0.5   # m/s^2·√Hz
    proc_noise_acc_z: float = 1.0    # m/s^2·√Hz
    proc_noise_vel: float = 0.0
    proc_noise_pos: float = 0.0
    proc_noise_att: float = 0.0
    meas_noise_gyro_rp: float = 0.1  # rad/s
    meas_noise_gyro_yaw: float = 0.1 # rad/s

    # --- Drag (body frame, default 0 for standard Crazyflie) ---
    drag_x: float = 0.0
    drag_y: float = 0.0
    drag_z: float = 0.0

    # --- Center-of-pressure offset from CoM (body frame, m) ---
    cop: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # --- Flowdeck position offset from CoM (body frame, m) ---
    flowdeck_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # --- Attitude reversion toward initial quaternion (when not flying) ---
    attitude_reversion: float = 0.001

    # --- ToF noise model: std(d) = a*(1 + exp(k*(d - d_a))) ---
    # Default values from zranger2 driver
    tof_exp_std_a: float = 0.0025   # m  (std at tof_exp_point_a)
    tof_exp_point_a: float = 2.5    # m
    tof_exp_std_b: float = 0.2      # m  (std at tof_exp_point_b)
    tof_exp_point_b: float = 4.0    # m

    # --- Flow noise (in raw sensor pixel units, fixed) ---
    flow_std_fixed_x: float = 2.0
    flow_std_fixed_y: float = 2.0    # was 3× flow_std_fixed

    # --- Flow sensor model (PMW3901 on flowdeck v2) ---
    flow_resolution: float = 0.1       # sensor reports 10x pixel motion; scale back
    flow_npix: float = 35.0            # pixels, aperture in x and y
    flow_theta_pix: float = 0.71674    # rad; 2*sin(42/2°)

    def tof_std_dev(self, distance_m: float) -> float:
        k = np.log(self.tof_exp_std_b / self.tof_exp_std_a) / (
            self.tof_exp_point_b - self.tof_exp_point_a
        )
        return self.tof_exp_std_a * (1.0 + np.exp(k * (distance_m - self.tof_exp_point_a)))


# ---------------------------------------------------------------------------
# EKF implementation
# ---------------------------------------------------------------------------

class CrazyflieEKF:
    """
    Extended Kalman Filter matching kalman_core.c from the Crazyflie firmware.

    State vector s (9-dim):
      [x, y, z]          — position in world frame (m)
      [vx_b, vy_b, vz_b] — velocity in body frame (m/s)
      [d0, d1, d2]       — attitude error (small-angle Rodrigues parameters)

    Attitude stored separately as quaternion q = [w, x, y, z].
    Rotation matrix R is derived from q and updated in finalize().
    """

    MAX_COV: float = 100.0
    MIN_COV: float = 1e-6
    EPS: float = 1e-6

    def __init__(self, params: EKFParams):
        self.p = params

        self.s = np.zeros(N_STATES)
        self.s[IX] = params.initial_x
        self.s[IY] = params.initial_y
        self.s[IZ] = params.initial_z

        yaw = params.initial_yaw
        self.q = np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])
        self._q0 = self.q.copy()   # initial quaternion for attitude reversion

        self.R = _quat_to_rot(self.q)

        self.P = np.zeros((N_STATES, N_STATES))
        self.P[IX, IX] = params.std_xy0 ** 2
        self.P[IY, IY] = params.std_xy0 ** 2
        self.P[IZ, IZ] = params.std_z0 ** 2
        self.P[IPX, IPX] = params.std_vel0 ** 2
        self.P[IPY, IPY] = params.std_vel0 ** 2
        self.P[IPZ, IPZ] = params.std_vel0 ** 2
        self.P[ID0, ID0] = params.std_att_rp0 ** 2
        self.P[ID1, ID1] = params.std_att_rp0 ** 2
        self.P[ID2, ID2] = params.std_att_yaw0 ** 2

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def predict(self, acc_g: np.ndarray, gyro_rad: np.ndarra0y, dt: float, quad_is_flying: bool):
        """
        IMU-driven prediction step.

        acc_g     : [ax, ay, az] accelerometer in g-units (as logged)
        gyro_rad  : [wx, wy, wz] gyroscope in rad/s
        dt        : time step in seconds
        quad_is_flying : enables drag and thrust-only Z-acceleration model
        """
        acc = acc_g * GRAVITY   # g → m/s²
        wx, wy, wz = gyro_rad
        ax, ay, az = acc
        dt2 = dt * dt
        R = self.R
        s = self.s
        p = self.p

        # --- Linearised Jacobian A (state transition matrix) ---
        A = np.eye(N_STATES)

        # ∂pos / ∂vel_body
        A[IX:IZ+1, IPX] = R[:, 0] * dt
        A[IX:IZ+1, IPY] = R[:, 1] * dt
        A[IX:IZ+1, IPZ] = R[:, 2] * dt

        # ∂pos / ∂att_error
        A[IX, ID0] = (s[IPY]*R[0,2] - s[IPZ]*R[0,1]) * dt
        A[IY, ID0] = (s[IPY]*R[1,2] - s[IPZ]*R[1,1]) * dt
        A[IZ, ID0] = (s[IPY]*R[2,2] - s[IPZ]*R[2,1]) * dt

        A[IX, ID1] = (-s[IPX]*R[0,2] + s[IPZ]*R[0,0]) * dt
        A[IY, ID1] = (-s[IPX]*R[1,2] + s[IPZ]*R[1,0]) * dt
        A[IZ, ID1] = (-s[IPX]*R[2,2] + s[IPZ]*R[2,0]) * dt

        A[IX, ID2] = (s[IPX]*R[0,1] - s[IPY]*R[0,0]) * dt
        A[IY, ID2] = (s[IPX]*R[1,1] - s[IPY]*R[1,0]) * dt
        A[IZ, ID2] = (s[IPX]*R[2,1] - s[IPY]*R[2,0]) * dt

        # ∂vel / ∂vel  (coriolis + optional drag)
        drag_x = p.drag_x if quad_is_flying else 0.0
        drag_y = p.drag_y if quad_is_flying else 0.0
        drag_z = p.drag_z if quad_is_flying else 0.0

        A[IPX, IPX] = 1 - drag_x * dt;  A[IPX, IPY] = wz * dt;     A[IPX, IPZ] = -wy * dt
        A[IPY, IPX] = -wz * dt;         A[IPY, IPY] = 1 - drag_y * dt;  A[IPY, IPZ] = wx * dt
        A[IPZ, IPX] = wy * dt;          A[IPZ, IPY] = -wx * dt;    A[IPZ, IPZ] = 1 - drag_z * dt

        # ∂vel / ∂att_error  (linearised gravity coupling)
        A[IPX, ID1] =  GRAVITY * R[2,2] * dt;  A[IPX, ID2] = -GRAVITY * R[2,1] * dt
        A[IPY, ID0] = -GRAVITY * R[2,2] * dt;  A[IPY, ID2] =  GRAVITY * R[2,0] * dt
        A[IPZ, ID0] =  GRAVITY * R[2,1] * dt;  A[IPZ, ID1] = -GRAVITY * R[2,0] * dt

        # ∂att_error / ∂att_error  (second-order Rodrigues covariance rotation)
        d0, d1, d2 = wx * dt / 2, wy * dt / 2, wz * dt / 2
        A[ID0, ID0] = 1 - d1*d1/2 - d2*d2/2;  A[ID0, ID1] = d2 + d0*d1/2;  A[ID0, ID2] = -d1 + d0*d2/2
        A[ID1, ID0] = -d2 + d0*d1/2;  A[ID1, ID1] = 1 - d0*d0/2 - d2*d2/2;  A[ID1, ID2] = d0 + d1*d2/2
        A[ID2, ID0] = d1 + d0*d2/2;   A[ID2, ID1] = -d0 + d1*d2/2;  A[ID2, ID2] = 1 - d0*d0/2 - d1*d1/2

        # --- Covariance prediction: P = A P A' ---
        self.P = A @ self.P @ A.T

        # --- State prediction ---
        spx, spy, spz = s[IPX], s[IPY], s[IPZ]

        if quad_is_flying:
            # Only body-Z acceleration (thrust); X/Y from velocity only
            dx = spx * dt
            dy = spy * dt
            dz = spz * dt + az * dt2 / 2.0

            s[IX] += R[0,0]*dx + R[0,1]*dy + R[0,2]*dz
            s[IY] += R[1,0]*dx + R[1,1]*dy + R[1,2]*dz
            s[IZ] += R[2,0]*dx + R[2,1]*dy + R[2,2]*dz - GRAVITY * dt2 / 2.0

            # velocity at center of pressure for drag computation
            cop = p.cop
            vcop_x = spx + wy * cop[2] - wz * cop[1]
            vcop_y = spy + wz * cop[0] - wx * cop[2]
            vcop_z = spz + wx * cop[1] - wy * cop[0]

            s[IPX] += dt * (wz*spy  - wy*spz  - GRAVITY*R[2,0] - p.drag_x*vcop_x)
            s[IPY] += dt * (-wz*spx + wx*spz  - GRAVITY*R[2,1] - p.drag_y*vcop_y)
            s[IPZ] += dt * (az + wy*spx - wx*spy - GRAVITY*R[2,2] - p.drag_z*vcop_z)
        else:
            # Full acceleration in all directions (free-fall / carried)
            dx = spx * dt + ax * dt2 / 2.0
            dy = spy * dt + ay * dt2 / 2.0
            dz = spz * dt + az * dt2 / 2.0

            s[IX] += R[0,0]*dx + R[0,1]*dy + R[0,2]*dz
            s[IY] += R[1,0]*dx + R[1,1]*dy + R[1,2]*dz
            s[IZ] += R[2,0]*dx + R[2,1]*dy + R[2,2]*dz - GRAVITY * dt2 / 2.0

            s[IPX] += dt * (ax + wz*spy  - wy*spz  - GRAVITY*R[2,0])
            s[IPY] += dt * (ay - wz*spx  + wx*spz  - GRAVITY*R[2,1])
            s[IPZ] += dt * (az + wy*spx  - wx*spy  - GRAVITY*R[2,2])

        # --- Attitude update via quaternion integration ---
        dtwx, dtwy, dtwz = wx * dt, wy * dt, wz * dt
        angle = np.sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + self.EPS
        ca, sa = np.cos(angle / 2.0), np.sin(angle / 2.0)
        dq = np.array([ca, sa*dtwx/angle, sa*dtwy/angle, sa*dtwz/angle])

        q = self.q
        tmp = np.array([
            dq[0]*q[0] - dq[1]*q[1] - dq[2]*q[2] - dq[3]*q[3],
            dq[1]*q[0] + dq[0]*q[1] + dq[3]*q[2] - dq[2]*q[3],
            dq[2]*q[0] - dq[3]*q[1] + dq[0]*q[2] + dq[1]*q[3],
            dq[3]*q[0] + dq[2]*q[1] - dq[1]*q[2] + dq[0]*q[3],
        ])

        if not quad_is_flying:
            keep = 1.0 - p.attitude_reversion
            tmp = keep * tmp + p.attitude_reversion * self._q0

        self.q = tmp / (np.linalg.norm(tmp) + self.EPS)

    def add_process_noise(self, dt: float):
        """Add diagonal process noise to the covariance (addProcessNoiseDt)."""
        p = self.p
        self.P[IX, IX]   += (p.proc_noise_acc_xy * dt**2 + p.proc_noise_vel * dt + p.proc_noise_pos) ** 2
        self.P[IY, IY]   += (p.proc_noise_acc_xy * dt**2 + p.proc_noise_vel * dt + p.proc_noise_pos) ** 2
        self.P[IZ, IZ]   += (p.proc_noise_acc_z  * dt**2 + p.proc_noise_vel * dt + p.proc_noise_pos) ** 2
        self.P[IPX, IPX] += (p.proc_noise_acc_xy * dt + p.proc_noise_vel) ** 2
        self.P[IPY, IPY] += (p.proc_noise_acc_xy * dt + p.proc_noise_vel) ** 2
        self.P[IPZ, IPZ] += (p.proc_noise_acc_z  * dt + p.proc_noise_vel) ** 2
        self.P[ID0, ID0] += (p.meas_noise_gyro_rp  * dt + p.proc_noise_att) ** 2
        self.P[ID1, ID1] += (p.meas_noise_gyro_rp  * dt + p.proc_noise_att) ** 2
        self.P[ID2, ID2] += (p.meas_noise_gyro_yaw * dt + p.proc_noise_att) ** 2
        self._clamp_covariance()

    def update_flow(self, dpixelx: float, dpixely: float, gyro_deg: np.ndarray, flow_dt: float):
        """
        Flow measurement update (two independent scalar updates for X and Y).

        dpixelx / dpixely : accumulated raw pixel counts (10x sensor scale)
        gyro_deg          : [wx, wy, wz] in deg/s — note: the firmware receives raw
                            gyro (deg/s) in this function and converts internally
        flow_dt           : integration time of this flow measurement (seconds)
        """
        Npix, thetapix = self.p.flow_npix, self.p.flow_theta_pix

        # Convert to rad/s (matching DEG_TO_RAD multiplication in mm_flow.c)
        wx = gyro_deg[0] * DEG_TO_RAD
        wy = gyro_deg[1] * DEG_TO_RAD
        wz = gyro_deg[2] * DEG_TO_RAD

        z_g = max(self.s[IZ], 0.1)   # clamp to avoid singularity
        vx_b, vy_b = self.s[IPX], self.s[IPY]
        R = self.R
        fdp = self.p.flowdeck_pos

        # Camera-point velocity (lever-arm correction for deck offset)
        v_cam_bx = vx_b + wy * fdp[2] - wz * fdp[1]
        v_cam_by = vy_b + wz * fdp[0] - wx * fdp[2]

        std_x = self.p.flow_std_fixed_x * self.p.flow_resolution
        std_y = self.p.flow_std_fixed_y * self.p.flow_resolution

        # X update
        pred_nx = (flow_dt * Npix / thetapix) * (v_cam_bx * R[2,2] / z_g - wy)
        meas_nx = dpixelx * self.p.flow_resolution

        h_x = np.zeros(N_STATES)
        h_x[IZ]  = (Npix * flow_dt / thetapix) * (R[2,2] * v_cam_bx / (-z_g * z_g))
        h_x[IPX] = (Npix * flow_dt / thetapix) * (R[2,2] / z_g)
        self._scalar_update(h_x, meas_nx - pred_nx, std_x)

        # Y update
        pred_ny = (flow_dt * Npix / thetapix) * (v_cam_by * R[2,2] / z_g + wx)
        meas_ny = dpixely * self.p.flow_resolution

        h_y = np.zeros(N_STATES)
        h_y[IZ]  = (Npix * flow_dt / thetapix) * (R[2,2] * v_cam_by / (-z_g * z_g))
        h_y[IPY] = (Npix * flow_dt / thetapix) * (R[2,2] / z_g)
        self._scalar_update(h_y, meas_ny - pred_ny, std_y)

        return pred_nx, meas_nx, pred_ny, meas_ny

    def update_tof(self, distance_m: float):
        """
        ToF (z-ranger) measurement update.

        distance_m : measured slant range in meters
        """
        r22 = self.R[2, 2]
        if r22 <= 0.1 or r22 <= 0:
            return  # too tilted; skip

        # Effective tilt angle (half cone-angle subtracted)
        # Clip to [-1, 1] to guard against float precision slightly exceeding
        # the valid range for arccos (quaternion‐derived R can produce |r22| > 1
        # by ~1e-15).
        angle = max(abs(np.arccos(np.clip(r22, -1.0, 1.0))) - np.radians(7.5), 0.0)
        cos_a = np.cos(angle)

        h = np.zeros(N_STATES)
        h[IZ] = 1.0 / cos_a
        pred_dist = self.s[IZ] / cos_a

        self._scalar_update(h, distance_m - pred_dist, self.p.tof_std_dev(distance_m))

    def finalize(self):
        """
        Incorporate attitude error into quaternion and update rotation matrix.
        Matches kalmanCoreFinalize() in the firmware.
        """
        v = self.s[ID0:ID0+3]
        mag = np.linalg.norm(v)

        if 1e-4 < mag < 10.0:
            angle = mag + self.EPS
            ca, sa = np.cos(angle / 2.0), np.sin(angle / 2.0)
            dq = np.array([ca, sa*v[0]/angle, sa*v[1]/angle, sa*v[2]/angle])
            q = self.q
            tmp = np.array([
                dq[0]*q[0] - dq[1]*q[1] - dq[2]*q[2] - dq[3]*q[3],
                dq[1]*q[0] + dq[0]*q[1] + dq[3]*q[2] - dq[2]*q[3],
                dq[2]*q[0] - dq[3]*q[1] + dq[0]*q[2] + dq[1]*q[3],
                dq[3]*q[0] + dq[2]*q[1] - dq[1]*q[2] + dq[0]*q[3],
            ])
            self.q = tmp / (np.linalg.norm(tmp) + self.EPS)

            # Rotate covariance to account for the attitude correction
            d0, d1, d2 = v / 2   # first-order Rodrigues approx
            Ac = np.eye(N_STATES)
            Ac[ID0, ID0] = 1 - d1*d1/2 - d2*d2/2;  Ac[ID0, ID1] = d2 + d0*d1/2;  Ac[ID0, ID2] = -d1 + d0*d2/2
            Ac[ID1, ID0] = -d2 + d0*d1/2;  Ac[ID1, ID1] = 1 - d0*d0/2 - d2*d2/2;  Ac[ID1, ID2] = d0 + d1*d2/2
            Ac[ID2, ID0] = d1 + d0*d2/2;   Ac[ID2, ID1] = -d0 + d1*d2/2;   Ac[ID2, ID2] = 1 - d0*d0/2 - d1*d1/2
            self.P = Ac @ self.P @ Ac.T

        # Update rotation matrix and reset attitude error states
        self.R = _quat_to_rot(self.q)
        self.s[ID0:ID0+3] = 0.0
        self._clamp_covariance()

    # ------------------------------------------------------------------
    # Derived quantities
    # ------------------------------------------------------------------

    @property
    def position(self) -> np.ndarray:
        return self.s[IX:IZ+1].copy()

    @property
    def velocity_body(self) -> np.ndarray:
        return self.s[IPX:IPZ+1].copy()

    @property
    def velocity_world(self) -> np.ndarray:
        return self.R @ self.s[IPX:IPZ+1]

    @property
    def euler_deg(self) -> np.ndarray:
        """Roll, pitch, yaw in degrees (ZYX convention, matching firmware)."""
        w, x, y, z = self.q
        roll  = np.degrees(np.arctan2(2*(y*z + w*x),  w*w - x*x - y*y + z*z))
        pitch = np.degrees(np.arcsin(np.clip(2*(x*z - w*y), -1, 1)))
        yaw   = np.degrees(np.arctan2(2*(x*y + w*z),  w*w + x*x - y*y - z*z))
        return np.array([roll, pitch, yaw])

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _scalar_update(self, h: np.ndarray, innovation: float, std_meas: float):
        """
        Single-measurement Kalman update (Joseph stabilised form).
        h         : measurement Jacobian row vector (length N_STATES)
        innovation: scalar (z_measured - z_predicted)
        std_meas  : measurement noise standard deviation
        """
        PH = self.P @ h
        R = std_meas ** 2
        HPHR = h @ PH + R
        K = PH / HPHR                    # Kalman gain vector

        self.s += K * innovation

        # Joseph form: P = (KH-I) P (KH-I)' + K R K'
        KH = np.outer(K, h)
        A = KH - np.eye(N_STATES)
        self.P = A @ self.P @ A.T + R * np.outer(K, K)
        self._clamp_covariance()

    def _clamp_covariance(self):
        """Enforce symmetry and bound the covariance matrix (matching firmware)."""
        P = 0.5 * (self.P + self.P.T)
        P = np.where(np.isnan(P) | (P > self.MAX_COV), self.MAX_COV, P)
        P[_DIAG] = np.maximum(P[_DIAG], self.MIN_COV)
        self.P = P


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _quat_to_rot(q: np.ndarray) -> np.ndarray:
    """Quaternion [w, x, y, z] → 3×3 rotation matrix (body→world)."""
    w, x, y, z = q
    return np.array([
        [w*w + x*x - y*y - z*z,  2*(x*y - w*z),           2*(x*z + w*y)],
        [2*(x*y + w*z),           w*w - x*x + y*y - z*z,   2*(y*z - w*x)],
        [2*(x*z - w*y),           2*(y*z + w*x),            w*w - x*x - y*y + z*z],
    ])


def _detect_flow_sensor(df: pd.DataFrame) -> str:
    """Detect which flow sensor is present from column names."""
    if "motion.deltaX" in df.columns and "motion.deltaY" in df.columns:
        return "pmw3901"
    elif "mtf02.dpixelX" in df.columns and "mtf02.dpixelY" in df.columns:
        return "mtf02"
    else:
        raise ValueError(
            "Cannot detect flow sensor: expected either motion.deltaX/Y "
            "(PMW3901) or mtf02.dpixelX/Y (mtf02) columns in CSV"
        )


def _flow_columns(sensor: str) -> tuple[str, str]:
    """Return (deltaX_col, deltaY_col) for the given sensor type."""
    if sensor == "pmw3901":
        return "motion.deltaX", "motion.deltaY"
    else:
        return "mtf02.dpixelX", "mtf02.dpixelY"


def _detect_new_measurements(df: pd.DataFrame, sensor: str) -> tuple[np.ndarray, np.ndarray]:
    """
    Return boolean arrays indicating rows where a new flow / ToF measurement
    arrived, detected by value changes from the previous row.

    Note: this heuristic misses cases where the sensor genuinely produces an
    identical reading twice in a row (e.g. zero optical flow over many cycles).
    """
    dx_col, dy_col = _flow_columns(sensor)
    flow_changed = (
        df[dx_col].diff().fillna(1).abs() > 0
    ) | (
        df[dy_col].diff().fillna(1).abs() > 0
    )
    tof_changed = df["range.zrange"].diff().fillna(1).abs() > 0
    return flow_changed.to_numpy(), tof_changed.to_numpy()


# ---------------------------------------------------------------------------
# Main replay loop
# ---------------------------------------------------------------------------

def run_ekf(
    csv_path: str | Path,
    imu_rate: float = 500.0,
    flying_height_threshold_m: float = 0.15,
    params: EKFParams | None = None,
    use_flow: bool = True,
    use_tof: bool = True,
) -> pd.DataFrame:
    """
    Load a CSV flight log and replay the Crazyflie EKF.

    Parameters
    ----------
    csv_path
        Path to the logged CSV file.
    imu_rate
        Sample rate of the IMU (and logging rate) in Hz.
    flying_height_threshold_m
        ToF reading above which the drone is considered airborne.
        This switches between the flying and non-flying prediction models.
    params
        EKF parameters; uses firmware defaults if not provided.
    use_flow
        If False, skip all optical-flow measurement updates.
    use_tof
        If False, skip all ToF/range measurement updates.

    Returns
    -------
    DataFrame with columns: time, x, y, z, vx, vy, vz, roll, pitch, yaw
    """
    if params is None:
        params = EKFParams()

    dt = 1.0 / imu_rate
    df = pd.read_csv(csv_path)

    # Trim data to the region where ToF measurements are still arriving.
    # If the sensor stops mid-flight the last portion has stale readings.
    tof_raw = df["range.zrange"]
    tof_changed = tof_raw.diff().abs() > 0
    last_tof_idx = tof_changed[tof_changed].index[-1] if tof_changed.any() else len(df) - 1
    if last_tof_idx < len(df) - 1:
        n_dropped = len(df) - 1 - last_tof_idx
        print(f"ToF stopped updating at index {last_tof_idx} — "
              f"trimming {n_dropped} samples ({n_dropped/imu_rate:.1f} s)")
        df = df.iloc[: last_tof_idx + 1].reset_index(drop=True)

    # Use the first valid locSrv sample as initial state (position + yaw).
    _LOC_SRV_INIT_COLS = ["locSrv.x", "locSrv.y", "locSrv.z",
                          "locSrv.qx", "locSrv.qy", "locSrv.qz", "locSrv.qw"]
    if all(c in df.columns for c in _LOC_SRV_INIT_COLS):
        _first = df[_LOC_SRV_INIT_COLS].dropna().iloc[0]
        _qw = _first["locSrv.qw"]
        _qx = _first["locSrv.qx"]
        _qy = _first["locSrv.qy"]
        _qz = _first["locSrv.qz"]
        _init_yaw = np.arctan2(2.0 * (_qx * _qy + _qw * _qz),
                               _qw*_qw + _qx*_qx - _qy*_qy - _qz*_qz)
        import dataclasses
        params = dataclasses.replace(
            params,
            initial_x=float(_first["locSrv.x"]),
            initial_y=float(_first["locSrv.y"]),
            initial_z=float(_first["locSrv.z"]),
            initial_yaw=float(_init_yaw),
        )
        print(f"Initialising EKF from locSrv: "
              f"x={params.initial_x:.3f} m, y={params.initial_y:.3f} m, "
              f"z={params.initial_z:.3f} m, yaw={np.degrees(_init_yaw):.1f} deg")

    # Unit conversions from Crazyflie log format
    #   range.zrange : uint16 in mm (rangeSet stores range_m * 1000)
    #   acc.*        : g-units  (logged raw; subsampler × 9.81 before EKF)
    #   gyro.*       : deg/s    (logged raw; subsampler × DEG_TO_RAD before predict,
    #                           but raw deg/s goes to flow update — handled below)
    tof_m = df["range.zrange"].to_numpy() / 1000.0   # mm → m

    acc_g = df[["acc.x", "acc.y", "acc.z"]].to_numpy()
    gyro_deg = df[["gyro.x", "gyro.y", "gyro.z"]].to_numpy()
    gyro_rad = gyro_deg * DEG_TO_RAD

    # Detect which flow sensor is present
    flow_sensor = _detect_flow_sensor(df)
    dx_col, dy_col = _flow_columns(flow_sensor)
    print(f"Flow sensor: {flow_sensor} (columns: {dx_col}, {dy_col})")

    # Both sensors use the same axis swap convention:
    #   accpx = -deltaY,  accpy = -deltaX
    # The logged values are raw sensor output, so we flip here.
    if flow_sensor == "pmw3901":
        dpixel_x = -df[dy_col].to_numpy()
        dpixel_y = -df[dx_col].to_numpy()
    elif flow_sensor == "mtf02":
        dpixel_x = df[dx_col].to_numpy()
        dpixel_y = df[dy_col].to_numpy()

    # Motion quality gate:
    #   PMW3901:  motion.motion == 0xB0 means valid motion
    #   mtf02:    mtf02.flowQual > 0 means valid (continuous quality score)
    if flow_sensor == "pmw3901" and "motion.motion" in df.columns:
        has_motion_flag = True
        motion_flag = df["motion.motion"].to_numpy()
    elif flow_sensor == "mtf02" and "mtf02.flowQual" in df.columns:
        has_motion_flag = True
        # flowQual > 0 indicates valid measurement; use a minimum threshold
        motion_flag = df["mtf02.flowQual"].to_numpy()
    else:
        has_motion_flag = False
        motion_flag = None

    # Use actual timestamps (ms → s) for per-step dt.
    timestamps = df["timestamp"].to_numpy() / 1000.0
    dt_steps = np.diff(timestamps, prepend=timestamps[0] - dt)
    dt_steps = np.clip(dt_steps, 1e-4, 0.1)   # guard against bad values

    new_flow, new_tof = _detect_new_measurements(df, flow_sensor)

    # Expected flow sensor period:
    #   PMW3901: ~100 Hz (10 ms period)
    #   mtf02:   ~50 Hz  (20 ms period)
    FLOW_DT_MAX = 0.01 if flow_sensor == "pmw3901" else 0.023

    # The firmware EKF predict step runs at PREDICT_RATE = 100 Hz (RATE_100_HZ),
    # using subsampled (averaged) IMU data. Running at the logged 500 Hz would give
    # a different covariance evolution.  We reproduce the firmware by accumulating
    # IMU samples between prediction windows and predicting once per window.
    PREDICT_RATE = 100.0  # Hz — must match kalman_core's PREDICT_RATE
    predict_period = 1.0 / PREDICT_RATE        # 10 ms
    acc_g_sum    = np.zeros(3)
    gyro_rad_sum = np.zeros(3)
    gyro_deg_last = np.zeros(3)   # last gyro sample for flow update (pass instantaneous)
    imu_count    = 0
    predict_dt   = 0.0
    next_predict_t = timestamps[0]

    n = len(df)
    ekf = CrazyflieEKF(params)

    # Output buffers
    out_time  = np.cumsum(dt_steps)
    out_pos   = np.zeros((n, 3))
    out_vel   = np.zeros((n, 3))
    out_vel_b = np.zeros((n, 3))
    out_euler = np.zeros((n, 3))
    out_pred_nx = np.full(n, np.nan)
    out_meas_nx = np.full(n, np.nan)
    out_pred_ny = np.full(n, np.nan)
    out_meas_ny = np.full(n, np.nan)

    last_flow_time = timestamps[0]

    for i in range(n):
        step_dt = dt_steps[i]
        t = timestamps[i]
        tof_reading = tof_m[i]
        quad_is_flying = tof_reading > flying_height_threshold_m

        # --- Accumulate IMU ---
        acc_g_sum    += acc_g[i]
        gyro_rad_sum += gyro_rad[i]
        gyro_deg_last = gyro_deg[i]
        imu_count    += 1
        predict_dt   += step_dt

        # --- Predict at 100 Hz, matching firmware PREDICT_RATE ---
        if t >= next_predict_t and imu_count > 0:
            avg_acc_g   = acc_g_sum   / imu_count
            avg_gyro_rad = gyro_rad_sum / imu_count
            ekf.predict(avg_acc_g, avg_gyro_rad, predict_dt, quad_is_flying)
            acc_g_sum    = np.zeros(3)
            gyro_rad_sum = np.zeros(3)
            imu_count  = 0
            predict_dt = 0.0
            next_predict_t += predict_period

        # --- Process noise every row (firmware calls addProcessNoise every EKF loop) ---
        ekf.add_process_noise(step_dt)

        if use_tof and new_tof[i]:
            ekf.update_tof(tof_reading)

        if use_flow and new_flow[i]:
            # Outlier filter: matches `if (abs(accpx) < OULIER_LIMIT && abs(accpy) < OULIER_LIMIT)`
            # in flowdeck_v1v2.c. Note: accpx = -deltaY, accpy = -deltaX, but abs() is sign-invariant.
            # lastTime is only updated for non-outlier reads (firmware behaviour).
            is_outlier = (
                abs(dpixel_x[i]) >= FLOW_OUTLIER_LIMIT or
                abs(dpixel_y[i]) >= FLOW_OUTLIER_LIMIT
            )
            if not is_outlier:
                flow_dt = min(t - last_flow_time, FLOW_DT_MAX)
                flow_dt = max(flow_dt, step_dt)   # guard against zero
                last_flow_time = t  # updated for all non-outlier reads, even without motion flag

                # Motion detection gate:
                #   PMW3901: motion.motion == 0xB0
                #   mtf02:   flowQual > 0 (always valid when quality is reported)
                if not has_motion_flag:
                    motion_ok = True
                elif flow_sensor == "pmw3901":
                    motion_ok = motion_flag[i] == 0xB0
                else:  # mtf02
                    motion_ok = motion_flag[i] > 0
                if motion_ok:
                    pred_nx, meas_nx, pred_ny, meas_ny = ekf.update_flow(
                        dpixel_x[i], dpixel_y[i], gyro_deg_last, flow_dt
                    )
                    out_pred_nx[i] = pred_nx
                    out_meas_nx[i] = meas_nx
                    out_pred_ny[i] = pred_ny
                    out_meas_ny[i] = meas_ny

        ekf.finalize()

        out_pos[i]   = ekf.position
        out_vel[i]   = ekf.velocity_world
        out_vel_b[i] = ekf.velocity_body
        out_euler[i] = ekf.euler_deg

    result = pd.DataFrame({
        "time":  out_time,
        "x":     out_pos[:, 0],
        "y":     out_pos[:, 1],
        "z":     out_pos[:, 2],
        "vx":    out_vel[:, 0],
        "vy":    out_vel[:, 1],
        "vz":    out_vel[:, 2],
        "vx_b":  out_vel_b[:, 0],
        "vy_b":  out_vel_b[:, 1],
        "vz_b":  out_vel_b[:, 2],
        "roll":  out_euler[:, 0],
        "pitch": out_euler[:, 1],
        "yaw":   out_euler[:, 2],
        # raw tof for reference
        "tof_m": tof_m,
        # raw optical-flow pixel counts for diagnostics
        "delta_x": df[dx_col].to_numpy(),
        "delta_y": df[dy_col].to_numpy(),
        # EKF flow innovation diagnostics
        "pred_nx": out_pred_nx,
        "meas_nx": out_meas_nx,
        "pred_ny": out_pred_ny,
        "meas_ny": out_meas_ny,
    })

    # Carry through on-board state estimates if logged on the SD card.
    _SE_COLS = {
        "stateEstimate.x":     "se_x",
        "stateEstimate.y":     "se_y",
        "stateEstimate.z":     "se_z",
        "stateEstimate.vx":    "se_vx",
        "stateEstimate.vy":    "se_vy",
        "stateEstimate.vz":    "se_vz",
        "stateEstimate.roll":  "se_roll",
        "stateEstimate.pitch": "se_pitch",
        "stateEstimate.yaw":   "se_yaw",
    }
    for src, dst in _SE_COLS.items():
        if src in df.columns:
            result[dst] = df[src].to_numpy()

    # Carry through external positioning (locSrv) if logged.
    _LOC_SRV_COLS = ["locSrv.x", "locSrv.y", "locSrv.z",
                     "locSrv.qx", "locSrv.qy", "locSrv.qz", "locSrv.qw"]
    if all(c in df.columns for c in _LOC_SRV_COLS):
        ls_x = df["locSrv.x"].to_numpy().copy()
        ls_y = df["locSrv.y"].to_numpy().copy()
        ls_z = df["locSrv.z"].to_numpy().copy()

        qx = df["locSrv.qx"].to_numpy().copy()
        qy = df["locSrv.qy"].to_numpy().copy()
        qz = df["locSrv.qz"].to_numpy().copy()
        qw = df["locSrv.qw"].to_numpy().copy()

        # --- Fix OptiTrack quaternion flips ---
        # OptiTrack occasionally confuses front/back of the marker pattern,
        # producing ~180° yaw jumps that last a few samples.  Two issues:
        #
        # 1) q and -q represent the same rotation (quaternion double cover).
        #    Fix by enforcing sign continuity: flip q when dot(q_i, q_{i-1}) < 0.
        #
        # 2) Genuine 180°-yaw glitches (different quaternion, not just sign).
        #    Detect via yaw jumps > 90° between consecutive *new* samples
        #    (accounting for zero-order hold) and replace the flipped segment
        #    with interpolated quaternions.

        # Step 1: quaternion sign continuity
        q_arr = np.column_stack([qw, qx, qy, qz])   # (N, 4)
        for i in range(1, len(q_arr)):
            if np.dot(q_arr[i], q_arr[i-1]) < 0:
                q_arr[i] = -q_arr[i]
        qw, qx, qy, qz = q_arr[:, 0], q_arr[:, 1], q_arr[:, 2], q_arr[:, 3]

        # Step 2: detect and replace 180°-yaw glitch segments
        _yaw = np.arctan2(2*(qx*qy + qw*qz),
                          qw*qw + qx*qx - qy*qy - qz*qz)
        _dyaw = np.diff(_yaw)
        _dyaw = (_dyaw + np.pi) % (2*np.pi) - np.pi   # wrap to [-π, π]

        _YAW_JUMP_THRESH = np.radians(90)
        _jump_idx = np.where(np.abs(_dyaw) > _YAW_JUMP_THRESH)[0]

        n_quat_fixed = 0
        if len(_jump_idx) >= 2:
            # Pair up jump-in / jump-out boundaries
            _used = np.zeros(len(q_arr), dtype=bool)
            j = 0
            while j < len(_jump_idx) - 1:
                enter = _jump_idx[j] + 1
                exit_ = _jump_idx[j + 1]
                # Only fix short glitch segments (< 50 samples ≈ 100 ms at 500 Hz)
                if exit_ - enter < 50:
                    _used[enter : exit_ + 1] = True
                    j += 2
                else:
                    j += 1

            if _used.any():
                _good = ~_used
                _idx = np.arange(len(q_arr))
                # SLERP is overkill for ~2-6 sample gaps; linear interp + renorm suffices
                for col in range(4):
                    q_arr[_used, col] = np.interp(_idx[_used], _idx[_good], q_arr[_good, col])
                # Re-normalise interpolated quaternions
                q_arr[_used] /= (np.linalg.norm(q_arr[_used], axis=1, keepdims=True) + 1e-12)
                qw, qx, qy, qz = q_arr[:, 0], q_arr[:, 1], q_arr[:, 2], q_arr[:, 3]
                n_quat_fixed = int(_used.sum())

        if n_quat_fixed:
            print(f"locSrv: fixed {n_quat_fixed} quaternion samples "
                  f"(OptiTrack ~180° yaw flips)")

        result["ls_x"] = ls_x
        result["ls_y"] = ls_y
        result["ls_z"] = ls_z

        # --- Compute world-frame velocity from OptiTrack positions ---
        # The locSrv positions are zero-order-held at the IMU rate (e.g. 500 Hz)
        # but OptiTrack only updates at ~180 Hz.  Using np.gradient on the held
        # signal produces velocity spikes at each update edge because dt is the
        # 500 Hz sample spacing, not the actual time between OptiTrack updates.
        #
        # Fix: identify true OptiTrack update instants (where position changes),
        # compute forward-difference velocity using the real inter-update dt,
        # reject any segment whose implied speed exceeds a physical limit
        # (OptiTrack tracking glitches), interpolate back to the full 500 Hz
        # timeline, and then low-pass filter.
        _VEL_LIMIT = 5.0   # m/s — well above any real Crazyflie speed

        def _optitrack_velocity(pos, ts):
            """Velocity from zero-order-held position, using true update timing."""
            dp = np.diff(pos)
            changed = dp != 0
            # Update indices: where the position value actually changes
            upd = np.concatenate([[0], np.where(changed)[0] + 1])
            if len(upd) < 3:
                return np.gradient(pos, ts)

            upd_ts  = ts[upd]
            upd_pos = pos[upd]

            # Forward-difference velocity between consecutive updates
            upd_dt   = np.diff(upd_ts)
            upd_dt   = np.where(upd_dt < 1e-6, 1e-6, upd_dt)
            upd_dpos = np.diff(upd_pos)
            fwd_vel  = upd_dpos / upd_dt        # length = len(upd) - 1

            # Reject segments with implausible speed (OptiTrack glitches)
            bad = np.abs(fwd_vel) > _VEL_LIMIT
            if bad.any():
                good = ~bad
                idx_v = np.arange(len(fwd_vel))
                fwd_vel[bad] = np.interp(idx_v[bad], idx_v[good], fwd_vel[good])

            # Assign each velocity to the midpoint of its interval for
            # time-accurate interpolation back to the full timeline
            mid_ts = (upd_ts[:-1] + upd_ts[1:]) / 2.0
            return np.interp(ts, mid_ts, fwd_vel)

        _b, _a = butter(4, 5.0 / (0.5 * imu_rate), btype="low")
        def _lpf(v): return filtfilt(_b, _a, v)
        ls_vx_w = _lpf(_optitrack_velocity(ls_x, timestamps))
        ls_vy_w = _lpf(_optitrack_velocity(ls_y, timestamps))
        ls_vz_w = _lpf(_optitrack_velocity(ls_z, timestamps))
        result["ls_vx"] = ls_vx_w
        result["ls_vy"] = ls_vy_w
        result["ls_vz"] = ls_vz_w

        # Rotate world velocities to body frame using the FULL rotation
        # matrix from the (cleaned) locSrv quaternion.
        # R columns (body→world): R[:,0], R[:,1], R[:,2]
        # v_body = R^T @ v_world  ⇒  v_body_i = R[0,i]*vx_w + R[1,i]*vy_w + R[2,i]*vz_w
        R00 = qw*qw + qx*qx - qy*qy - qz*qz
        R10 = 2*(qx*qy + qw*qz)
        R20 = 2*(qx*qz - qw*qy)
        R01 = 2*(qx*qy - qw*qz)
        R11 = qw*qw - qx*qx + qy*qy - qz*qz
        R21 = 2*(qy*qz + qw*qx)
        R02 = 2*(qx*qz + qw*qy)
        R12 = 2*(qy*qz - qw*qx)
        R22 = qw*qw - qx*qx - qy*qy + qz*qz
        result["ls_vx_b"] = R00*ls_vx_w + R10*ls_vy_w + R20*ls_vz_w
        result["ls_vy_b"] = R01*ls_vx_w + R11*ls_vy_w + R21*ls_vz_w
        result["ls_vz_b"] = R02*ls_vx_w + R12*ls_vy_w + R22*ls_vz_w

        # Convert quaternion to Euler angles in degrees.
        # Light smoothing on attitude — high cutoff just removes jitter.
        _b_att, _a_att = butter(4, 20.0 / (0.5 * imu_rate), btype="low")
        def _lpf_att(v): return filtfilt(_b_att, _a_att, v)
        result["ls_roll"]  = _lpf_att(np.degrees(np.arctan2(2*(qy*qz + qw*qx),  qw*qw - qx*qx - qy*qy + qz*qz)))
        result["ls_pitch"] = _lpf_att(np.degrees(np.arcsin(np.clip(2*(qx*qz - qw*qy), -1.0, 1.0))))
        result["ls_yaw"]   = _lpf_att(np.degrees(np.arctan2(2*(qx*qy + qw*qz),  qw*qw + qx*qx - qy*qy - qz*qz)))

    return result


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def _ylim_from_data(arrays, pct=1, margin=0.1):
    """Y-limits from the p–(100-p) percentile of all provided arrays, plus a margin."""
    combined = np.concatenate([a[np.isfinite(a)] for a in arrays])
    lo, hi = np.percentile(combined, [pct, 100 - pct])
    pad = max((hi - lo) * margin, 1e-3)
    return lo - pad, hi + pad


def _maximize_figure(fig):
    """Best-effort window maximization across common matplotlib backends."""
    try:
        mgr = fig.canvas.manager
        try:
            mgr.window.showMaximized()          # Qt5 / Qt6
        except AttributeError:
            try:
                mgr.resize(*mgr.window.maxsize())   # TkAgg
            except AttributeError:
                try:
                    mgr.frame.Maximize(True)        # WxAgg
                except AttributeError:
                    pass
    except Exception:
        pass


def _print_rmse(results: pd.DataFrame):
    """Print RMSE between replay estimates and locSrv ground truth."""
    has_locsrv = "ls_x" in results.columns
    has_ls_vb  = "ls_vx_b" in results.columns
    if not has_locsrv:
        print("No locSrv data available — skipping RMSE.")
        return

    def rmse(a, b):
        d = a - b
        return np.sqrt(np.nanmean(d * d))

    print("\n===== RMSE (replay vs locSrv) =====")

    # Position
    print(f"  pos  x: {rmse(results['x'].to_numpy(), results['ls_x'].to_numpy()):.4f} m")
    print(f"  pos  y: {rmse(results['y'].to_numpy(), results['ls_y'].to_numpy()):.4f} m")
    print(f"  pos  z: {rmse(results['z'].to_numpy(), results['ls_z'].to_numpy()):.4f} m")

    # Body-frame velocity
    if has_ls_vb:
        print(f"  body vx: {rmse(results['vx_b'].to_numpy(), results['ls_vx_b'].to_numpy()):.4f} m/s")
        print(f"  body vy: {rmse(results['vy_b'].to_numpy(), results['ls_vy_b'].to_numpy()):.4f} m/s")
        print(f"  body vz: {rmse(results['vz_b'].to_numpy(), results['ls_vz_b'].to_numpy()):.4f} m/s")

    # Attitude
    if "ls_roll" in results.columns:
        print(f"  roll:  {rmse(results['roll'].to_numpy(), results['ls_roll'].to_numpy()):.4f} deg")
        print(f"  pitch: {rmse(results['pitch'].to_numpy(), results['ls_pitch'].to_numpy()):.4f} deg")
        print(f"  yaw:   {rmse(results['yaw'].to_numpy(), results['ls_yaw'].to_numpy()):.4f} deg")

    print("===================================\n")


def plot_results(results: pd.DataFrame):
    t = results["time"]
    has_se_pos  = "se_x"    in results.columns
    has_se_vel  = "se_vx"   in results.columns
    has_se_att  = "se_roll" in results.columns
    has_locsrv  = "ls_x"    in results.columns

    # ---------------------------------------------------------------
    # Figure 1: 3 columns × 3 rows  —  Position | Velocity | Attitude
    # ---------------------------------------------------------------
    fig1, axes = plt.subplots(3, 3, figsize=(18, 10), sharex=True,
                              constrained_layout=True)
    _maximize_figure(fig1)
    fig1.suptitle("EKF Replay", fontsize=12)

    # --- Column 0: Positions (x, y, z) ---
    pos_info = [
        (0, "x",  "se_x",  "ls_x",  False, "x (m)"),
        (1, "y",  "se_y",  "ls_y",  False, "y (m)"),
        (2, "z",  "se_z",  "ls_z",  True,  "z (m)"),
    ]
    for row, rcol, se_col, ls_col, plot_tof, ylabel in pos_info:
        ax = axes[row, 0]
        ax.plot(t, results[rcol], "C0", label="replay")
        if has_se_pos and se_col in results.columns:
            ax.plot(t, results[se_col], "--", color="C1", alpha=0.7, label="onboard")
        if has_locsrv and ls_col in results.columns:
            ax.plot(t, results[ls_col], ":", color="C2", alpha=0.8, label="locSrv")
        if plot_tof:
            ax.plot(t, results["tof_m"], "k--", alpha=0.4, label="tof")
        _arrays = [results[rcol].to_numpy()]
        if has_se_pos and se_col in results.columns:
            _arrays.append(results[se_col].to_numpy())
        if has_locsrv and ls_col in results.columns:
            _arrays.append(results[ls_col].to_numpy())
        if plot_tof:
            _arrays.append(results["tof_m"].to_numpy())
        ax.set_ylim(_ylim_from_data(_arrays))
        ax.set_ylabel(ylabel)
        ax.legend(fontsize=7, ncol=2)
        ax.grid(True)
        if row == 0:
            ax.set_title("Position")

    # --- Column 1: Body-frame Velocities (vx_b, vy_b, vz_b) ---
    has_ls_vb = "ls_vx_b" in results.columns
    vel_info = [
        (0, "vx_b", "ls_vx_b", "vx body (m/s)"),
        (1, "vy_b", "ls_vy_b", "vy body (m/s)"),
        (2, "vz_b", "ls_vz_b", "vz body (m/s)"),
    ]
    for row, rcol, ls_col, ylabel in vel_info:
        ax = axes[row, 1]
        ax.plot(t, results[rcol], "C0", label="replay")
        if has_ls_vb and ls_col in results.columns:
            ax.plot(t, results[ls_col], ":", color="C2", alpha=0.8, label="locSrv")
        _arrays = [results[rcol].to_numpy()]
        if has_ls_vb and ls_col in results.columns:
            _arrays.append(results[ls_col].to_numpy())
        ax.set_ylim(_ylim_from_data(_arrays))
        ax.set_ylabel(ylabel)
        ax.legend(fontsize=7, ncol=2)
        ax.grid(True)
        if row == 0:
            ax.set_title("Velocity (body)")

    # --- Column 2: Attitudes (roll, pitch, yaw) ---
    att_info = [
        (0, "pitch", "se_pitch", "ls_pitch", "pitch (deg)"),
        (1, "roll",  "se_roll",  "ls_roll",  "roll (deg)"),
        (2, "yaw",   "se_yaw",   "ls_yaw",   "yaw (deg)"),
    ]
    for row, rcol, se_col, ls_col, ylabel in att_info:
        ax = axes[row, 2]
        ax.plot(t, results[rcol], "C0", label="replay")
        if has_se_att and se_col in results.columns:
            ax.plot(t, results[se_col], "--", color="C1", alpha=0.7, label="onboard")
        if has_locsrv and ls_col in results.columns:
            ax.plot(t, results[ls_col], ":", color="C2", alpha=0.8, label="locSrv")
        _arrays = [results[rcol].to_numpy()]
        if has_se_att and se_col in results.columns:
            _arrays.append(results[se_col].to_numpy())
        if has_locsrv and ls_col in results.columns:
            _arrays.append(results[ls_col].to_numpy())
        ax.set_ylim(_ylim_from_data(_arrays))
        ax.set_ylabel(ylabel)
        ax.legend(fontsize=7, ncol=2)
        ax.grid(True)
        if row == 0:
            ax.set_title("Attitude")

    for col in range(3):
        axes[2, col].set_xlabel("Time (s)")

    # ---------------------------------------------------------------
    # Figure 2: Optical-flow deltaX and deltaY
    # ---------------------------------------------------------------
    fig2, axes2 = plt.subplots(4, 1, figsize=(14, 12), sharex=True,
                               constrained_layout=True)
    _maximize_figure(fig2)
    fig2.suptitle("Optical Flow Measurements", fontsize=12)

    # Low-pass filter for flow delta values (cutoff 10 Hz)
    _fs = 1.0 / float(np.median(np.diff(t.to_numpy())))
    _b_flow, _a_flow = butter(4, 10.0 / (0.5 * _fs), btype="low")
    dx_filt = filtfilt(_b_flow, _a_flow, results["delta_x"].to_numpy())
    dy_filt = filtfilt(_b_flow, _a_flow, results["delta_y"].to_numpy())

    axes2[0].plot(t, results["delta_x"], color="C0", linewidth=0.5, alpha=0.3, label="deltaX raw")
    axes2[0].plot(t, dx_filt, color="C0", linewidth=1.2, label="deltaX filtered")
    axes2[0].set_ylim(_ylim_from_data([dx_filt]))
    axes2[0].set_ylabel("deltaX (raw pixels)")
    axes2[0].legend(fontsize=8)
    axes2[0].grid(True)
    axes2[0].axhline(0, color="k", linewidth=0.5, linestyle="--")

    axes2[1].plot(t, results["delta_y"], color="C1", linewidth=0.5, alpha=0.3, label="deltaY raw")
    axes2[1].plot(t, dy_filt, color="C1", linewidth=1.2, label="deltaY filtered")
    axes2[1].set_ylim(_ylim_from_data([dy_filt]))
    axes2[1].set_ylabel("deltaY (raw pixels)")
    axes2[1].legend(fontsize=8)
    axes2[1].grid(True)
    axes2[1].axhline(0, color="k", linewidth=0.5, linestyle="--")

    # --- Flow innovation: predicted vs measured nx / ny ---
    pred_nx = results["pred_nx"].to_numpy()
    meas_nx = results["meas_nx"].to_numpy()
    pred_ny = results["pred_ny"].to_numpy()
    meas_ny = results["meas_ny"].to_numpy()

    axes2[2].plot(t, meas_nx, ".", color="C0", linewidth=1.0, label="meas nx")
    axes2[2].plot(t, pred_nx, ".-", color="C3", linewidth=1.0, label="pred nx")
    _valid_nx = np.concatenate([meas_nx[np.isfinite(meas_nx)], pred_nx[np.isfinite(pred_nx)]])
    if len(_valid_nx):
        axes2[2].set_ylim(_ylim_from_data([_valid_nx]))
    axes2[2].set_ylabel("nx (m)")
    axes2[2].legend(fontsize=8)
    axes2[2].grid(True)
    axes2[2].axhline(0, color="k", linewidth=0.5, linestyle="--")
    axes2[2].set_title("Flow EKF: predicted vs measured (scaled pixels)", fontsize=9)

    axes2[3].plot(t, meas_ny, ".", color="C1", linewidth=1.0, label="meas ny")
    axes2[3].plot(t, pred_ny, ".-", color="C3", linewidth=1.0, label="pred ny")
    _valid_ny = np.concatenate([meas_ny[np.isfinite(meas_ny)], pred_ny[np.isfinite(pred_ny)]])
    if len(_valid_ny):
        axes2[3].set_ylim(_ylim_from_data([_valid_ny]))
    axes2[3].set_ylabel("ny (m)")
    axes2[3].set_xlabel("Time (s)")
    axes2[3].legend(fontsize=8)
    axes2[3].grid(True)
    axes2[3].axhline(0, color="k", linewidth=0.5, linestyle="--")

    plt.show()


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Replay Crazyflie EKF from a logged CSV flight."
    )
    parser.add_argument("csv", help="Path to the log CSV file")
    parser.add_argument("--imu-rate",     type=float, default=500.0,
                        help="IMU / logging rate in Hz (default: 500)")
    parser.add_argument("--fly-thresh",   type=float, default=0.15,
                        help="ToF threshold for 'quad is flying' in m (default: 0.15)")
    parser.add_argument("--initial-z",    type=float, default=0.0,
                        help="Initial Z position estimate in m (default: 0)")
    parser.add_argument("--initial-yaw",  type=float, default=0.0,
                        help="Initial yaw in degrees (default: 0)")
    parser.add_argument("--proc-acc-xy",  type=float, default=0.5)
    parser.add_argument("--proc-acc-z",   type=float, default=1.0)
    parser.add_argument("--no-plot",      action="store_true",
                        help="Skip plotting")
    parser.add_argument("--save",         type=str, default=None,
                        help="Save results to this CSV path")
    args = parser.parse_args()

    params = EKFParams(
        initial_z=args.initial_z,
        initial_yaw=args.initial_yaw * DEG_TO_RAD,
        cop=np.array([0.0, 0.0, 0.0]),
        flowdeck_pos=np.array([0.0, 0.0, 0.0]),
        proc_noise_acc_xy=args.proc_acc_xy,
        proc_noise_acc_z=args.proc_acc_z,
    )

    params = EKFParams(
        initial_z=args.initial_z,
        initial_yaw=args.initial_yaw * DEG_TO_RAD,

        # Only drag and bit of std
        # drag_x=4.2,
        # drag_y=1.8,
        # drag_z=0.3,
        # flow_std_fixed_x=2.0,
        # flow_std_fixed_y=4.0,

        # Tuned
        # cop=np.array([0.0, 0.0, 0.0]),
        # flowdeck_pos=np.array([0.0, 0.0, 0.0]),
        # proc_noise_acc_xy=1.42299,
        # proc_noise_acc_z=1.22339,
        # meas_noise_gyro_rp=0.0124584,
        # meas_noise_gyro_yaw=0.0239012,
        # drag_x=4.29181,
        # drag_y=2.54629,
        # drag_z=0.70963,
        # flow_std_fixed_x=0.76976,
        # flow_std_fixed_y=2.20616,
        # tof_exp_std_a=0.00176657,
        # flow_resolution=0.188428,


        # Tuned with cop and flowdeck lever arm
        proc_noise_acc_xy=1.05006,
        proc_noise_acc_z=0.604273,
        meas_noise_gyro_rp=0.0521776,
        meas_noise_gyro_yaw=0.116742,
        drag_x=4.39468,
        drag_y=2.88896,
        drag_z=0.0611769,
        flow_std_fixed_x=1.07615,
        flow_std_fixed_y=5.41112,
        flow_resolution=0.22987,
        cop=np.array([0.0, 0.0, 0.03]),
        flowdeck_pos=np.array([0.0, 0.0, -0.12]),
    )

    results = run_ekf(
        csv_path=args.csv,
        imu_rate=args.imu_rate,
        flying_height_threshold_m=args.fly_thresh, 
        params=params,
    )

    print(results.describe())
    _print_rmse(results)

    if args.save:
        results.to_csv(args.save, index=False)
        print(f"Saved results to {args.save}")

    if not args.no_plot:
        plot_results(results)


if __name__ == "__main__":
    main()
