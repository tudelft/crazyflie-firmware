# -*- coding: utf-8 -*-
#
# Fly the Flapper on the ONBOARD estimator (flowdeck + IMU + ToF) only, while
# streaming mocap in purely as GROUND TRUTH that gets recorded in the onboard
# USD log (locSrv.*). Nothing from the mocap is fused into the estimator.
#
# Requires firmware with the `locSrv.enExtPoseFuse` param (see
# crtp_localization_service.c) and an SD-card config.txt logging locSrv.*,
# stateEstimate.v*, mtf02.*, gyro.*, acc.*, range.zrange
# (see tools/usdlog/config_flapper_mocap.txt).
#
# Derived from the Bitcraze mocap example.
import math
import time
from threading import Thread

import motioncapture

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

# The host name or ip address of the mocap system
host_name = '192.168.209.81'

# The type of the mocap system
mocap_system_type = 'optitrack'

# The name of the rigid body that represents the Crazyflie
rigid_body_name = 'flapper_02'

# True: send position and orientation; False: send position only.
# NOTE: with fusion disabled this only affects what gets LOGGED as ground truth.
send_full_pose = True

# Which flight to run:
#   'box'      - centered box via absolute go_to waypoints (position control)
#   'box_vel'  - same box via body-frame velocity legs (send_hover_setpoint);
#                closes the loop on the well-estimated velocity, not the drifty
#                position estimate, so leg distance is v*T regardless of drift
#   'fig8_follow' - figure-8 via velocity streaming, nose follows velocity
#   'fig8_fixed'  - figure-8 via velocity streaming, heading held fixed
#   'velocity' - simple forward/back body-frame velocity streaming
#   'position' - x-axis go_to
SEQUENCE = 'box'

# Box geometry (metres). Room is 7x7, box is 2.5x2.5 -> half-size 1.25.
# Corners are ~1.77 m diagonal from the centre (well inside the mocap volume
# and far from the walls).
BOX_HALF = 1.25
BOX_Z = 1.0
BOX_SPEED = 0.5      # m/s cruise for each go_to leg
BOX_SETTLE = 1.5     # s to hover/stabilise at each waypoint
TAKEOFF_HOVER = 2.0  # s to hover after takeoff before moving

# Figure-8 (Gerono lemniscate) geometry
FIG8_A = 1.25              # x half-amplitude (m); y amplitude is A/2
FIG8_PERIOD = 12.0         # seconds per full figure-8 lap
FIG8_LAPS = 3              # number of laps to fly
FIG8_Z = 1.0               # height (m)
FIG8_YAWRATE_MAX = 120.0   # deg/s clamp for heading-follow mode

# time variables
t_start = 0


class MocapWrapper(Thread):
    def __init__(self, body_name):
        Thread.__init__(self)
        self.body_name = body_name
        self.on_pose = None
        self._stay_open = True
        self.start()

    def close(self):
        self._stay_open = False

    def run(self):
        print('Connecting to mocap system')
        mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})
        print('Connecting to optitrack successful')
        last_print = 0.0
        while self._stay_open:
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                if name == self.body_name:
                    if self.on_pose:
                        pos = obj.position
                        now = time.time()
                        if now - last_print > 0.5:
                            print(f"MOCAP {self.body_name}: "
                                  f"x={pos[0]:+.3f} y={pos[1]:+.3f} z={pos[2]:+.3f}")
                            last_print = now
                        self.on_pose([pos[0], pos[1], pos[2], obj.rotation])


def wait_for_position_estimator(scf):
    # With flow-only estimation these are the VELOCITY variances (position
    # variance is unbounded without an absolute reference, which is expected).
    print('Waiting for estimator (velocity variance) to converge...')
    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10
    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]
            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x, max_x = min(var_x_history), max(var_x_history)
            min_y, max_y = min(var_y_history), max(var_y_history)
            min_z, max_z = min(var_z_history), max(var_z_history)
            print('{} {} {}'.format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (max_y - min_y) < threshold and \
                    (max_z - min_z) < threshold:
                break


def send_extpose_quat(cf, x, y, z, quat):
    """
    Forward the mocap pose to the Crazyflie. With locSrv.enExtPoseFuse=0 this is
    NOT fused into the estimator; it only populates the locSrv.* log variables so
    the mocap is recorded as ground truth in the onboard USD log.
    """
    if send_full_pose:
        cf.extpos.send_extpose(z, x, y, quat.z, quat.x, quat.y, quat.w)
    else:
        cf.extpos.send_extpos(z, x, y)


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(cf)


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')


def configure_onboard_only_estimation(cf):
    """
    Fly on the onboard estimator (flowdeck + IMU + ToF) only:
      - locSrv.enExtPoseFuse = 0  -> mocap is logged (locSrv.*) but NOT fused
      - mtf02.flowDisable    = 0  -> optical flow enabled
    """
    print('Onboard-only estimation: disabling mocap fusion, enabling MTF-02 flow')
    cf.param.set_value('locSrv.enExtPoseFuse', '0')
    cf.param.set_value('mtf02.flowDisable', '0')


def start_onboard_logging(cf):
    cf.param.set_value('usd.logging', '1')


def stop_onboard_logging(cf):
    cf.param.set_value('usd.logging', '0')


def run_box_sequence(cf):
    """
    Fly a 5x5 m box centred on the takeoff point, facing forward (yaw=0)
    throughout, using absolute go_to waypoints in the ONBOARD-estimate frame.

    Traces the full square perimeter and returns to centre:
        centre -> forward -> left(corner) -> back -> right -> forward -> left
              -> back to centre -> land

    Body convention: forward = +x, left = +y. Legs alternate half-size (2.5 m,
    centre<->edge) and full-size (5 m, side-to-side). All waypoints are absolute
    positions in the estimate frame, so flow drift shows up as the physical box
    (in mocap) being distorted vs this clean commanded box.
    """
    global t_start
    H = BOX_HALF
    z = BOX_Z
    yaw = 0.0  # face forward the whole time

    # (x, y, label) absolute waypoints; starts/ends at centre (0, 0)
    legs = [
        (H,    0.0, "forward -> front-mid"),
        (H,    H,   "left    -> front-left corner"),
        (-H,   H,   "back    -> back-left corner"),
        (-H,  -H,   "right   -> back-right corner"),
        (H,   -H,   "forward -> front-right corner"),
        (H,    0.0, "left    -> front-mid"),
        (0.0,  0.0, "back    -> centre"),
    ]

    cf.platform.send_arming_request(True)
    time.sleep(3.0)
    hlc = cf.high_level_commander
    start_onboard_logging(cf)
    t_start = time.time()

    # Take off and hold a stable hover BEFORE any horizontal motion, so the
    # drone doesn't try to climb and translate at the same time.
    print(f'Takeoff to z={z} m')
    hlc.takeoff(z, 3.0)
    time.sleep(3.0)          # let the takeoff ramp complete
    print(f'Hover at centre for {TAKEOFF_HOVER:.1f}s')
    hlc.go_to(0.0, 0.0, z, 0.0, TAKEOFF_HOVER)  # hold centre, facing forward
    time.sleep(TAKEOFF_HOVER)

    prev = (0.0, 0.0)
    for (x, y, label) in legs:
        dist = math.hypot(x - prev[0], y - prev[1])
        dur = max(dist / BOX_SPEED, 1.0)
        print(f'go_to ({x:+.2f}, {y:+.2f}) [{label}] in {dur:.1f}s')
        hlc.go_to(x, y, z, yaw, dur)
        time.sleep(dur + BOX_SETTLE)
        prev = (x, y)

    print('Landing')
    stop_onboard_logging(cf)
    hlc.land(0.0, 2.5)
    time.sleep(4.0)
    hlc.stop()


def run_sequence(cf):
    global t_start
    x, y, z, yaw = -2, 0, 1, 0.0
    cf.platform.send_arming_request(True)
    time.sleep(3.0)
    commander = cf.high_level_commander
    start_onboard_logging(cf)
    t_start = time.time()
    commander.takeoff(1.0, 2.0)
    time.sleep(6.0)
    print('Going to starting position at x=-2')
    commander.go_to(x, y, z, yaw, 4)
    time.sleep(6.0)
    print('Moving to x=-1')
    commander.go_to(x + 1, y, z, yaw, 4)
    time.sleep(6.0)
    print('Moving back to x=-2')
    commander.go_to(x, y, z, yaw, 4)
    time.sleep(6.0)
    print('Landing')
    commander.land(0.0, 2.0)
    time.sleep(6.0)
    stop_onboard_logging(cf)
    commander.stop()


def run_box_velocity_sequence(cf):
    """
    Fly the same perimeter as run_box_sequence, but by streaming BODY-FRAME
    velocity setpoints (send_hover_setpoint) instead of absolute go_to
    waypoints. This closes the loop on the (well-estimated) velocity rather than
    the drifting position estimate, so each leg's distance is v*T independent of
    position drift.

    Caveats vs the position box: yaw drift still rotates the whole figure
    (send_hover_setpoint only commands zero yaw RATE, not absolute heading), a
    residual velocity bias still integrates over each leg (~bias*T), and there
    is no position hold — disturbances just walk off.

    Body convention: forward = +vx, left = +vy. Leg distances match the go_to
    box: half-size (H) centre<->edge, full-size (2H) side-to-side.
    """
    global t_start
    H = BOX_HALF
    z = BOX_Z
    S = BOX_SPEED
    setpoint_hz = 50.0
    dt = 1.0 / setpoint_hz

    # (vx, vy, distance, label) body-frame legs; same perimeter as the go_to box
    legs = [
        (S,    0.0, H,     "forward -> front-mid"),
        (0.0,  S,   H,     "left    -> front-left corner"),
        (-S,   0.0, 2 * H, "back    -> back-left corner"),
        (0.0, -S,   2 * H, "right   -> back-right corner"),
        (S,    0.0, 2 * H, "forward -> front-right corner"),
        (0.0,  S,   H,     "left    -> front-mid"),
        (-S,   0.0, H,     "back    -> centre"),
    ]

    cf.platform.send_arming_request(True)
    time.sleep(3.0)
    hlc = cf.high_level_commander
    start_onboard_logging(cf)
    t_start = time.time()

    print(f'Takeoff to z={z} m')
    hlc.takeoff(z, 3.0)
    time.sleep(3.0)          # let the takeoff ramp complete

    def stream(vx, vy, duration, label):
        n = max(int(duration * setpoint_hz), 1)
        print(f'{label}: vx={vx:+.2f} vy={vy:+.2f} m/s, hold z={z} m for {duration:.1f}s')
        for _ in range(n):
            cf.commander.send_hover_setpoint(vx, vy, 0.0, z)
            time.sleep(dt)

    # Stabilise in a hover (zero velocity, hold altitude) before moving.
    stream(0.0, 0.0, TAKEOFF_HOVER, 'Hover at centre')

    for (vx, vy, dist, label) in legs:
        dur = max(dist / S, 1.0)
        stream(vx, vy, dur, label)
        stream(0.0, 0.0, BOX_SETTLE, f'settle after [{label}]')

    # Hand control back to the high-level commander to land cleanly.
    cf.commander.send_notify_setpoint_stop()
    print('Landing')
    stop_onboard_logging(cf)
    hlc.land(0.0, 2.5)
    time.sleep(4.0)
    hlc.stop()


def run_figure8_sequence(cf, follow_heading):
    """
    Fly a figure-8 (Gerono lemniscate) by streaming body-frame velocity
    setpoints. Two heading modes:

      follow_heading=True  -> nose points along the direction of travel
                              (forward-only body velocity + commanded yaw rate);
                              flies the 8 "like a plane".
      follow_heading=False -> heading held fixed (yaw ~0); the 8 is traced with
                              combined vx/vy body velocity and zero yaw rate.

    Path (world/estimate frame, centred on the takeoff point):
        x(theta) = A sin(theta),  y(theta) = A sin(theta) cos(theta),
        theta = 2*pi * t / T
    Velocity is the analytic derivative; in follow mode the heading is the
    unwrapped path tangent, differentiated for yaw rate and clamped.

    Streaming velocity closes the loop on the well-estimated velocity rather
    than the drifting position, giving rich, continuously-varying excitation
    (good for EKF identification) while staying independent of position drift.
    """
    global t_start
    A = FIG8_A
    z = FIG8_Z
    T = FIG8_PERIOD
    w = 2.0 * math.pi / T          # dtheta/dt
    setpoint_hz = 50.0
    dt = 1.0 / setpoint_hz
    n_steps = int(FIG8_LAPS * T * setpoint_hz)

    def path_vel(theta):
        # world-frame velocity along the lemniscate (derivative of the path)
        vx = A * math.cos(theta) * w
        vy = A * math.cos(2.0 * theta) * w
        return vx, vy

    cf.platform.send_arming_request(True)
    time.sleep(3.0)
    hlc = cf.high_level_commander
    start_onboard_logging(cf)
    t_start = time.time()

    print(f'Takeoff to z={z} m')
    hlc.takeoff(z, 3.0)
    time.sleep(3.0)          # let the takeoff ramp complete
    print(f'Hover at centre for {TAKEOFF_HOVER:.1f}s')
    hlc.go_to(0.0, 0.0, z, 0.0, TAKEOFF_HOVER)
    time.sleep(TAKEOFF_HOVER)

    mode = 'heading-follows-velocity' if follow_heading else 'fixed-heading'
    print(f'Figure-8: {mode}, A={A} m, period={T}s, laps={FIG8_LAPS}')

    if follow_heading:
        # Pre-orient the nose to the initial path tangent for a clean start.
        vx0, vy0 = path_vel(0.0)
        psi0 = math.atan2(vy0, vx0)
        print(f'  Pre-orienting nose to initial tangent {math.degrees(psi0):+.0f} deg')
        hlc.go_to(0.0, 0.0, z, psi0, 2.0)
        time.sleep(2.5)

    psi_prev = None
    for k in range(n_steps):
        theta = w * (k * dt)
        vx, vy = path_vel(theta)
        if follow_heading:
            speed = math.hypot(vx, vy)
            psi = math.atan2(vy, vx)
            if psi_prev is None:
                yawrate = 0.0
            else:
                dpsi = psi - psi_prev
                dpsi = (dpsi + math.pi) % (2.0 * math.pi) - math.pi   # wrap
                yawrate = math.degrees(dpsi / dt)
                yawrate = max(-FIG8_YAWRATE_MAX, min(FIG8_YAWRATE_MAX, yawrate))
            psi_prev = psi
            # forward-only body velocity; steer heading with yaw rate
            cf.commander.send_hover_setpoint(speed, 0.0, yawrate, z)
        else:
            # fixed heading (yaw held ~0): body frame ~ world frame, so stream
            # the world-path velocity directly as body vx/vy with zero yaw rate
            cf.commander.send_hover_setpoint(vx, vy, 0.0, z)
        time.sleep(dt)

    # Settle in a hover, then hand back to the high-level commander to land.
    for _ in range(int(BOX_SETTLE * setpoint_hz)):
        cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z)
        time.sleep(dt)

    cf.commander.send_notify_setpoint_stop()
    print('Landing')
    stop_onboard_logging(cf)
    hlc.land(0.0, 2.5)
    time.sleep(4.0)
    hlc.stop()


def run_velocity_sequence(cf):
    """
    Body-frame vx/vy velocity via send_hover_setpoint while holding an absolute
    altitude (zdistance from ToF). Setpoints are streamed at ~50 Hz.
    """
    global t_start
    hover_z = 1.0
    fwd_speed = 0.5
    fwd_time = 4.0
    back_speed = 0.5
    back_time = 8.0
    setpoint_hz = 50.0
    dt = 1.0 / setpoint_hz

    cf.platform.send_arming_request(True)
    time.sleep(3.0)
    hlc = cf.high_level_commander
    start_onboard_logging(cf)
    t_start = time.time()

    print(f'Takeoff to z={hover_z} m')
    hlc.takeoff(hover_z, 3.0)
    time.sleep(5.0)

    def stream_hover(vx, duration, label):
        print(f'{label}: vx={vx:+.2f} m/s, hold z={hover_z} m for {duration:.1f} s')
        for _ in range(int(duration * setpoint_hz)):
            cf.commander.send_hover_setpoint(vx, 0.0, 0.0, hover_z)
            time.sleep(dt)

    stream_hover(0.0, 2.0, 'Takeover hover')
    stream_hover(fwd_speed, fwd_time, 'Forward')
    stream_hover(-back_speed, back_time, 'Backward')
    stream_hover(0.0, 2.0, 'Settle hover')

    cf.commander.send_notify_setpoint_stop()
    print('Landing')
    hlc.land(0.0, 2.5)
    time.sleep(4.0)
    stop_onboard_logging(cf)
    hlc.stop()


def reconnect_and_land():
    start_time = time.time()
    while time.time() - start_time < 10:
        print('Try to reconnect')
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                print('Recovered connection and stopping propellors')
                scf.cf.high_level_commander.stop()
        except Exception as e:
            print('Connection failed: ', e)
            time.sleep(1)


def connection_failed_link_error(link_uri, msg):
    print(f"Connection to {link_uri} failed: {msg}")
    reconnect_and_land()


def log_diag_callback(timestamp, data, logconf):
    print(f"[{time.time() - t_start:.2f}s] DIAG "
          f"vx: cmd={data['ctrltarget.vx']:+.2f} est={data['stateEstimate.vx']:+.2f} | "
          f"locSrv.x={data['locSrv.x']:+.2f} est.x={data['stateEstimate.x']:+.2f} | "
          f"yaw={data['stateEstimate.yaw']:+.1f}")


def add_diag_logconfig(cf):
    log_config = LogConfig(name='Diag', period_in_ms=500)
    log_config.add_variable('ctrltarget.vx', 'float')
    log_config.add_variable('stateEstimate.vx', 'float')
    log_config.add_variable('locSrv.x', 'float')
    log_config.add_variable('stateEstimate.x', 'float')
    log_config.add_variable('stateEstimate.yaw', 'float')
    log_config.data_received_cb.add_callback(log_diag_callback)
    cf.log.add_config(log_config)
    log_config.start()
    return log_config


def stop_diag_logconfig(logconfig):
    logconfig.stop()
    logconfig.data_received_cb.remove_callback(log_diag_callback)


def console_incoming(console_text):
    print(console_text, end='')


if __name__ == '__main__':
    print('initializing drivers')
    cflib.crtp.init_drivers()

    print('Initializing MocapWrapper')
    mocap_wrapper = MocapWrapper(rigid_body_name)

    print('Connect to the Crazyflie')
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        cf.connection_lost.add_callback(connection_failed_link_error)
        cf.console.receivedChar.add_callback(console_incoming)

        diag_config = add_diag_logconfig(cf)

        # Stream mocap in for LOGGING ONLY (locSrv.*); it is not fused.
        mocap_wrapper.on_pose = lambda pose: send_extpose_quat(cf, pose[0], pose[1], pose[2], pose[3])

        print('Activating the kalman estimator')
        activate_kalman_estimator(cf)

        # Fly on flow + IMU only; record mocap as ground truth.
        configure_onboard_only_estimation(cf)

        reset_estimator(cf)
        if SEQUENCE == 'box':
            run_box_sequence(cf)
        elif SEQUENCE == 'box_vel':
            run_box_velocity_sequence(cf)
        elif SEQUENCE == 'fig8_follow':
            run_figure8_sequence(cf, follow_heading=True)
        elif SEQUENCE == 'fig8_fixed':
            run_figure8_sequence(cf, follow_heading=False)
        elif SEQUENCE == 'velocity':
            run_velocity_sequence(cf)
        else:
            run_sequence(cf)
        time.sleep(1.0)
        stop_diag_logconfig(diag_config)

    mocap_wrapper.close()
