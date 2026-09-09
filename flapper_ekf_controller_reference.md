# Flapper EKF + Controller Reference (flow-only flight)

Self-contained reference for the Crazyflie **Flapper** running flow-only estimation
(MTF-02 optical flow + downward ToF + IMU) with the **tuned** EKF parameters and the
**PID** controller cascade. Values are the mocap-tuned set determined on the `ekf_data`
branch and baked into the `ekf_playground` firmware compile defaults; the offline
replay (`ekf_replay.py`) uses the same numbers.

Firmware source of truth:
- Estimator: `src/modules/src/kalman_core/kalman_core.c`, `mm_flow.c`, `mm_tof.c`
- Estimator defaults: `src/modules/interface/kalman_core/kalman_core_params_defaults.h`,
  `src/platform/interface/platform_defaults_flapper.h`
- Flow driver: `src/deck/drivers/src/mtf02deck.c`
- Controller: `src/modules/src/controller/controller_pid.c`,
  `position_controller_pid.c`, `attitude_pid_controller.c`, `pid.c`

Pipeline: `Sensors -> EKF (estimator) -> Commander (setpoints) -> PID controller -> power distribution -> motors/servos`

---

## 1. EKF state vector (error-state Kalman, 9 states)

```
s = [ x,  y,  z,      # position, WORLD frame [m]
      vx, vy, vz,     # velocity, BODY frame [m/s]  (body origin ~ IMU/CoM)
      d0, d1, d2 ]    # attitude error, small-angle Rodrigues params [rad]
```
Attitude is carried separately as a quaternion `q=[w,x,y,z]`; `R` is the body->world
rotation matrix from `q`. The attitude error `(d0,d1,d2)` is folded back into `q` and
reset to zero each step ("finalize").

Reference-frame notes (important for mocap comparison):
- `z` is the **height of the downward sensor above the floor** — the ToF update has
  **no lever-arm term**, so `z` tracks the sensor, not the CoM (CoM is ~12 cm higher).
- `vx,vy,vz` are the velocity of the **body origin (IMU ~ CoM)** — the flow update
  applies the sensor lever arm, so flow is de-levered to CoM velocity.
- Mocap reports the **marker/rigid-body origin** (set in Motive), which differs from
  both the sensor and the CoM. Expect a constant ~8 cm z offset that is geometry,
  not error.

---

## 2. EKF prediction (IMU-driven), flying model

Inputs: accel `a` [m/s^2] (z used as `zacc`), gyro `omega=(wx,wy,wz)` [rad/s], `dt`, `g=9.81`.

Center-of-pressure (rotational drag) cross term, `r_cop = (drag_rx, drag_ry, drag_rz)`:
```
odr_x = wy*drag_rz - wz*drag_ry
odr_y = wz*drag_rx - wx*drag_rz
odr_z = wx*drag_ry - wy*drag_rx
```

Position update (body increments rotated to world):
```
dx = vx*dt
dy = vy*dt
dz = vz*dt + zacc*dt^2/2
x += R00*dx + R01*dy + R02*dz
y += R10*dx + R11*dy + R12*dz
z += R20*dx + R21*dy + R22*dz - g*dt^2/2
```

Body-velocity update (Coriolis + gravity projection + body drag + rotational drag):
```
vx += dt*( wz*vy - wy*vz - g*R20 - dragBx*vx - dragBx*odr_x )
vy += dt*(-wz*vx + wx*vz - g*R21 - dragBy*vy - dragBy*odr_y )
vz += dt*( zacc + wy*vx - wx*vy - g*R22 - dragBz*vz - dragBz*odr_z )
```
Attitude propagates from gyro; process noise added on the diagonal each step
(see params). When **not flying**, an accelerometer-driven model + attitude reversion
to the initial quaternion is used instead.

The state-transition Jacobian `A` mirrors the above (e.g. `A[vx,vx]=1-dt*dragBx`,
velocity<->attitude coupling via `R`), used for the covariance propagation
`P = A P A^T + Q`.

---

## 3. EKF measurement updates

### 3a. Downward ToF (height) — `mm_tof.c`
Tilt-corrected 1-D height measurement. `angle = |acos(R22)| - 7.5deg` (clamped >=0):
```
predictedDistance = z / cos(angle)
h[z] = 1 / cos(angle)
innovation = measuredDistance - predictedDistance
```
No lever arm -> `z` is the sensor height. Measurement std from the MTF-02 range model.

### 3b. Optical flow (velocity) — `mm_flow.c`
Camera constants: `Npix = 35`, `thetapix = 0.71674` (=2*sin(42deg/2)).
Gyro in rad/s; `z_g = max(z, 0.1)`.

Lever-arm (sensor offset `r_fd = flowdeck_pos`) adds camera-point velocity `omega x r`:
```
v_cam_bx = vx + (wy*FLOWDECK_POS_Z - wz*FLOWDECK_POS_Y)
v_cam_by = vy + (wz*FLOWDECK_POS_X - wx*FLOWDECK_POS_Z)
```

Predicted vs measured accumulated pixels:
```
predictedNX = (dt*Npix/thetapix) * ( v_cam_bx*R22/z_g - wy )
predictedNY = (dt*Npix/thetapix) * ( v_cam_by*R22/z_g + wx )
measuredNX  = dpixelx * FLOW_RESOLUTION
measuredNY  = dpixely * FLOW_RESOLUTION

h_x[z]  = (Npix*dt/thetapix) * ( R22*v_cam_bx / (-z_g^2) )
h_x[vx] = (Npix*dt/thetapix) * ( R22 / z_g )
h_y[z]  = (Npix*dt/thetapix) * ( R22*v_cam_by / (-z_g^2) )
h_y[vy] = (Npix*dt/thetapix) * ( R22 / z_g )

std_x = flowStdX * FLOW_RESOLUTION      # per-axis, X and Y independent scalar updates
std_y = flowStdY * FLOW_RESOLUTION
```
Only applied when flying and `z > 0.12 m`. Flow is fused as two independent scalar updates.

MTF-02 driver mapping (`mtf02deck.c`), raw sensor motion -> logged/fused pixels:
```
dpixelx = flowScale * (-motion_y)
dpixely = flowScale * (-motion_x)
```

---

## 4. EKF tuned parameters (the "dynamics model" values)

| Param (firmware / cfclient) | Tuned value | Meaning |
|---|---|---|
| `kalman.pNAcc_xy`   | **1.05006**   | process noise, accel XY |
| `kalman.pNAcc_z`    | **0.604273**  | process noise, accel Z |
| `kalman.pNVel`      | 0             | process noise, velocity |
| `kalman.pNPos`      | 0             | process noise, position |
| `kalman.pNAtt`      | 0             | process noise, attitude |
| `kalman.mNGyro_rollpitch` | **0.0521776** | gyro meas noise, roll/pitch [rad/s] |
| `kalman.mNGyro_yaw` | **0.116742**  | gyro meas noise, yaw [rad/s] |
| `kalman.mNBaro`     | 2.0           | baro meas noise (unused flow-only) |
| `kalman.dragBx`     | **4.39468**   | body drag X (velocity damping) |
| `kalman.dragBy`     | **2.88896**   | body drag Y |
| `dragBz` (compile only) | **0.0611769** | body drag Z (`EKF_DRAG_BZ`, not a runtime param) |
| `kalman.drag_rx`    | 0             | center-of-pressure offset X |
| `kalman.drag_ry`    | 0             | center-of-pressure offset Y |
| `kalman.drag_rz`    | **0.03**      | center-of-pressure offset Z (`EKF_DRAG_RZ`) |
| `FLOWDECK_POS_X/Y/Z` (compile only) | 0 / 0 / **-0.12** | flow-sensor lever arm from CoM [m] |
| `mtf02.flowStdX`    | **1.07615**   | flow meas std, X |
| `mtf02.flowStdY`    | **5.41112**   | flow meas std, Y (down-weighted ~5x; flapping corrupts Y) |
| `mtf02.flowScale`   | **2.3**       | raw-motion -> pixel scale (MTF-02) |
| `FLOW_RESOLUTION`   | 0.10          | pixel scale-back (effective flow scale = 2.3*0.10 = 0.23) |
| `Npix` / `thetapix` | 35 / 0.71674  | camera geometry |
| initial P: pos_xy / pos_z | 100 / 1 | initial position std [m] (xy unobservable) |
| initial P: vel / att_rp / att_yaw | 0.01 / 0.01 / 0.01 | initial std |
| `attitudeReversion` | 0.001         | attitude zero-reversion when not flying |

Notes:
- `dragBz` and `drag_z`, `FLOWDECK_POS_*` are **compile-time only** (set via firmware
  defaults, not runtime params). Everything else is runtime-tunable and PERSISTENT.
- These are PERSISTENT params: a stored EEPROM value overrides the firmware default at
  boot, and reflashing does NOT clear EEPROM. Verify the live drone matches this table.
- Old logs recorded with `flowScale=1.0` need the replay run with
  `--flow-resolution 0.22987` (0.23 effective) instead of the 0.10 default.

---

## 5. Controller: cascaded PID (`stabilizer.controller = 1` = PID)

Four nested loops, outer -> inner (each runs a `pid.c` PID; some add feed-forward `KFF`):

```
position setpoint  --[posCtlPid]-->  velocity setpoint
velocity setpoint  --[velCtlPid]-->  thrust + roll/pitch attitude setpoint
attitude setpoint  --[pid_attitude]--> body-rate setpoint
rate setpoint      --[pid_rate]-->    torques -> power distribution -> motors/servos
```
- Position/velocity loops: `position_controller_pid.c` (world-frame x/y/z; velocity
  errors produce desired lean angles + thrust). Rate limits `xVelMax/yVelMax/zVelMax`,
  angle limits `rLimit/pLimit`.
- Attitude/rate loops: `attitude_pid_controller.c`. Yaw uses a large rate feed-forward.
- Attitude is taken from the **Kalman** estimator (not the complementary filter).

---

## 6. Controller parameters (live values from the drone)

### Position PID — `posCtlPid` (position error -> velocity setpoint)
| axis | Kp | Ki | Kd | Kff | VelMax |
|---|---|---|---|---|---|
| x | 1.5 | 0   | 0 | 0 | 2 m/s |
| y | 1.5 | 0   | 0 | 0 | 2 m/s |
| z | 5   | 0.5 | 0 | 0 | 1 m/s |

Also: `rLimit=30 deg`, `pLimit=30 deg`, `thrustBase=40000`, `thrustMin=20000`.

### Velocity PID — `velCtlPid` (velocity error -> attitude/thrust)
| axis | Kp | Ki | Kd | Kff |
|---|---|---|---|---|
| vx | 15   | 1   | 0 | 15 |
| vy | 10   | 1   | 0 | 8  |
| vz | 12.5 | 0.5 | 0 | 0  |

### Attitude PID — `pid_attitude` (angle error -> rate setpoint)
| axis | Kp | Ki | Kd | Kff |
|---|---|---|---|---|
| roll  | 10 | 0 | 0.2  | 0 |
| pitch | 13 | 0 | 1.0  | 0 |
| yaw   | 8  | 0 | 0.35 | 0 |

Also: `yawMaxDelta=30`, `attFiltCut=15`, `attFiltEn=0`.

### Rate PID — `pid_rate` (rate error -> torque)
| axis | Kp | Ki | Kd | Kff |
|---|---|---|---|---|
| roll  | 50 | 0 | 0 | 0   |
| pitch | 50 | 0 | 0 | 0   |
| yaw   | 80 | 0 | 0 | 220 |

Also: `omxFiltCut=20`, `omyFiltCut=20`, `omzFiltCut=5`, `rateFiltEn=1`.

---

## 7. Flapper power distribution / trims (`flapper.*`)

| Param | Value | Meaning |
|---|---|---|
| `flapperMaxThrust` | 60000 | max thrust command |
| `motBiasRoll`      | 0     | motor roll bias trim |
| `servPitchNeutr`   | 50    | pitch servo neutral (%) |
| `servYawNeutr`     | 50    | yaw servo neutral (%) |

The Flapper uses servo-based pitch/yaw + differential motor thrust for roll (flapping-
wing mixing), distinct from a standard quad X mixer.

---

## 8. Known behavior / gotchas (flow-only)

- **Position and yaw are unobservable** flow-only (no absolute reference) -> they drift.
  Velocity is observable and well-estimated; position is its integral, so small velocity
  bias -> large position drift over time.
- **Control cannot fix estimation**: the controller drives the drone to its *estimate*.
  A clean-looking flight in estimate-space can still drift in the room by the estimation
  error. Position-mode flight hides this; velocity-mode (`box_vel`) exposes it.
- **vy is the worst channel**: flapping drives roll/yaw rates to +/-200-400 deg/s, which
  contaminate flow-Y through the lever arm -> `flowStdY` is ~5x `flowStdX` on purpose.
- The ~8 cm mocap-vs-onboard z offset is geometry (marker origin vs sensor), not error.
