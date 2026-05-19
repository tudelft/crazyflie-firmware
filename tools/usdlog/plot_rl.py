#!/usr/bin/env python3
"""
Plot RL controller flight data from CSV log files
"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse

# Gate configuration (from examples/app_rl_controller/src/rl_controller.c)
GATE_R = 1.5
BASE_GATE_X = np.array([ GATE_R, 0.0, -GATE_R, 0.0,  GATE_R, 0.0, -GATE_R, 0.0])
BASE_GATE_Y = np.array([-GATE_R, 0.0,  GATE_R, 3.0,  GATE_R, 0.0, -GATE_R, -3.0])
BASE_GATE_YAW = np.array([
    np.pi / 2,  np.pi,  np.pi / 2, 0.0,
    -np.pi / 2, -np.pi, -np.pi / 2, 0.0,
])

def infer_nn_window(data, t, window_s=0.1, rl_rate_range=(100.0, 350.0)):
    """Return (start_idx, end_idx) of the NN-control window, or (None, None).

    The RL controller commands motors at ~200 Hz while the PID hover loop runs
    at ~500 Hz, so we count how often motor.m1 changes within a rolling time
    window and flag the region where the estimated update rate falls in the
    200 Hz band. The longest contiguous region that matches is returned.
    """
    if 'motor.m1' not in data.columns:
        return None, None
    m = data['motor.m1'].to_numpy()
    t_arr = np.asarray(t)
    n = len(m)
    if n < 2:
        return None, None
    changed = np.concatenate(([False], m[1:] != m[:-1]))
    # Two-pointer sweep: counts[hi] = number of PWM changes in (t[hi]-window_s, t[hi]]
    counts = np.zeros(n, dtype=np.int32)
    lo = 0
    total = 0
    for hi in range(n):
        if changed[hi]:
            total += 1
        while t_arr[hi] - t_arr[lo] > window_s:
            if changed[lo]:
                total -= 1
            lo += 1
        counts[hi] = total
    rate = counts / window_s
    rl_mask = (rate >= rl_rate_range[0]) & (rate <= rl_rate_range[1])
    if not np.any(rl_mask):
        return None, None
    idx = np.where(rl_mask)[0]
    # Largest contiguous run
    breaks = np.where(np.diff(idx) > 1)[0]
    if len(breaks):
        starts = np.concatenate(([idx[0]], idx[breaks + 1]))
        ends = np.concatenate((idx[breaks], [idx[-1]]))
    else:
        starts = np.array([idx[0]])
        ends = np.array([idx[-1]])
    best = int(np.argmax(ends - starts))
    # Shift start back by the window width so the edge lines up with the
    # first RL-rate sample rather than the window having fully filled.
    start = int(starts[best])
    t0 = t_arr[start] - window_s
    start = int(np.searchsorted(t_arr, t0, side='left'))
    return start, int(ends[best])


def mark_events(ax, t, crossings, tgt, nn_start_idx, nn_end_idx,
                label_gates=False):
    """Draw gate-crossing and NN-window vertical lines on a time-series axis."""
    for idx, k in enumerate(crossings):
        ax.axvline(t.iloc[k], color='k', linestyle='--', linewidth=0.8,
                   alpha=0.5, label='Gate crossing' if idx == 0 else None)
        if label_gates:
            ax.text(t.iloc[k], ax.get_ylim()[1], f' G{int(tgt[k])}',
                    fontsize=7, color='k', alpha=0.7, va='top')
    if nn_start_idx is not None:
        ax.axvline(t.iloc[nn_start_idx], color='red', linestyle='-',
                   linewidth=1.2, alpha=0.7, label='NN start')
    if nn_end_idx is not None:
        ax.axvline(t.iloc[nn_end_idx], color='red', linestyle=':',
                   linewidth=1.2, alpha=0.7, label='NN end')


def infer_gate_crossings(data, origin_x=0.0, origin_y=0.0):
    """Replicate checkGatePassing() from rl_controller.c on logged positions.

    Returns an array the same length as `data` giving the current target-gate
    index at each sample (like `rlapp.tgtGate` would).
    """
    sim_x = data['stateEstimate.x'].to_numpy()
    sim_y = -data['stateEstimate.y'].to_numpy()  # firmware uses NED sim frame
    gx = BASE_GATE_X + origin_x
    gy = BASE_GATE_Y + origin_y
    n = len(sim_x)
    tgt = np.zeros(n, dtype=np.uint8)
    cur = 0
    for k in range(1, n):
        nx, ny = np.cos(BASE_GATE_YAW[cur]), np.sin(BASE_GATE_YAW[cur])
        old_proj = (sim_x[k - 1] - gx[cur]) * nx + (sim_y[k - 1] - gy[cur]) * ny
        new_proj = (sim_x[k]     - gx[cur]) * nx + (sim_y[k]     - gy[cur]) * ny
        if old_proj < 0.0 and new_proj >= 0.0:
            cur = (cur + 1) % len(BASE_GATE_X)
        tgt[k] = cur
    return tgt


def body_velocity(data):
    """Return (vx_body, vy_body, vz_body, source) in firmware body frame
    (x forward, y left, z up).

    Prefers kalman.statePX/PY/PZ (logged directly from the EKF body-frame
    velocity state). Falls back to rotating stateEstimate.v{x,y,z} (world
    ENU) into the body frame using stateEstimate.{roll,pitch,yaw}.
    """
    if {'kalman.statePX', 'kalman.statePY', 'kalman.statePZ'}.issubset(data.columns):
        return (data['kalman.statePX'].to_numpy(),
                data['kalman.statePY'].to_numpy(),
                data['kalman.statePZ'].to_numpy(),
                'kalman.stateP')

    roll = np.deg2rad(data['stateEstimate.roll'].to_numpy())
    pitch = np.deg2rad(data['stateEstimate.pitch'].to_numpy())
    yaw = np.deg2rad(data['stateEstimate.yaw'].to_numpy())
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    vx = data['stateEstimate.vx'].to_numpy()
    vy = data['stateEstimate.vy'].to_numpy()
    vz = data['stateEstimate.vz'].to_numpy()
    # body = R_wb^T @ world, with R_wb = Rz(yaw) Ry(pitch) Rx(roll)
    bx = cy*cp*vx + sy*cp*vy - sp*vz
    by = (cy*sp*sr - sy*cr)*vx + (sy*sp*sr + cy*cr)*vy + cp*sr*vz
    bz = (cy*sp*cr + sy*sr)*vx + (sy*sp*cr - cy*sr)*vy + cp*cr*vz
    return bx, by, bz, 'rotated stateEstimate.v'


def plot_gates(ax, origin_x=0.0, origin_y=0.0, half_width=1.0):
    """Overlay the figure-8 gates on a top-view (X, -Y) trajectory plot."""
    gx = BASE_GATE_X + origin_x
    gy = BASE_GATE_Y + origin_y
    for i, (x, y, yaw) in enumerate(zip(gx, gy, BASE_GATE_YAW)):
        # Gate plane is perpendicular to the normal (cos(yaw), sin(yaw))
        tx, ty = -np.sin(yaw), np.cos(yaw)
        x0, x1 = x - half_width * tx, x + half_width * tx
        y0, y1 = y - half_width * ty, y + half_width * ty
        ax.plot([x0, x1], [-y0, -y1], 'k-', linewidth=2, alpha=0.7)
        ax.scatter(x, -y, c='orange', s=40, marker='s',
                   edgecolors='k', zorder=4,
                   label='Gates' if i == 0 else None)
        ax.annotate(str(i), (x, -y), textcoords='offset points',
                    xytext=(6, 6), fontsize=8, color='darkorange')

def plot_rl_log(csv_file):
    # Read CSV data
    data = pd.read_csv(csv_file)

    # Convert timestamp to relative time in seconds
    t = (data['timestamp'] - data['timestamp'].iloc[0]) / 1000.0

    # Gate-crossing indices (use logged tgtGate when available, else replay
    # the firmware's plane-crossing check) and NN-control window (inferred
    # from the motor-PWM update rate: 200 Hz in RL mode, ~500 Hz otherwise).
    if 'rlapp.tgtGate' in data.columns:
        tgt = data['rlapp.tgtGate'].to_numpy()
    else:
        tgt = infer_gate_crossings(data)
    crossings = np.where(np.diff(tgt) != 0)[0] + 1
    nn_start_idx, nn_end_idx = infer_nn_window(data, t)

    # Body-frame velocity (kalman.statePX/PY/PZ if logged, else rotated)
    bvx, bvy, bvz, vel_src = body_velocity(data)

    # Set up figure with white background
    plt.rcParams['figure.facecolor'] = 'w'
    fig = plt.figure(figsize=(16, 12))

    # 1. Position trajectory
    ax1 = plt.subplot(3, 3, 1)
    ax1.plot(data['stateEstimate.x'], -data['stateEstimate.y'], 'b-', linewidth=0.5)
    ax1.scatter(data['stateEstimate.x'].iloc[0], -data['stateEstimate.y'].iloc[0],
                c='g', s=100, marker='o', label='Start', zorder=5)
    ax1.scatter(data['stateEstimate.x'].iloc[-1], -data['stateEstimate.y'].iloc[-1],
                c='r', s=100, marker='x', label='End', zorder=5)
    plot_gates(ax1)
    ax1.set_xlabel('X Position [m]')
    ax1.set_ylabel('Y Position [m]')
    ax1.set_title('2D Trajectory (Top View)')
    ax1.legend()
    ax1.set_ylim([4, -4])
    ax1.set_xlim([-4, 4])
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')

    # 2. Position vs Time
    ax2 = plt.subplot(3, 3, 2)
    ax2.plot(t, data['stateEstimate.x'], label='X', linewidth=1)
    ax2.plot(t, -data['stateEstimate.y'], label='Y', linewidth=1)
    ax2.plot(t, -data['stateEstimate.z'], label='Z', linewidth=1)
    mark_events(ax2, t, crossings, tgt, nn_start_idx, nn_end_idx,
                label_gates=True)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Position [m]')
    ax2.set_title('Position vs Time')
    ax2.legend(loc='best')
    ax2.grid(True, alpha=0.3)

    # 3. Body velocity vs Time
    ax3 = plt.subplot(3, 3, 3, sharex=ax2)
    ax3.plot(t, bvx, label='Vx (fwd)', linewidth=1)
    ax3.plot(t, bvy, label='Vy (left)', linewidth=1)
    ax3.plot(t, bvz, label='Vz (up)', linewidth=1)
    mark_events(ax3, t, crossings, tgt, nn_start_idx, nn_end_idx)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Velocity [m/s]')
    ax3.set_title(f'Body velocity ({vel_src})')
    ax3.legend(loc='best')
    ax3.grid(True, alpha=0.3)

    # 4. Attitude (Roll, Pitch, Yaw)
    ax4 = plt.subplot(3, 3, 4, sharex=ax2)
    ax4.plot(t, data['stateEstimate.roll'], label='Roll', linewidth=1)
    ax4.plot(t, data['stateEstimate.pitch'], label='Pitch', linewidth=1)
    ax4.plot(t, data['stateEstimate.yaw'], label='Yaw', linewidth=1)
    mark_events(ax4, t, crossings, tgt, nn_start_idx, nn_end_idx)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Angle [rad]')
    ax4.set_title('Attitude')
    ax4.legend(loc='best')
    ax4.grid(True, alpha=0.3)

    # 5. Gyro data
    ax5 = plt.subplot(3, 3, 5, sharex=ax2)
    ax5.plot(t, data['gyro.x'], label='Roll rate', linewidth=0.5)
    ax5.plot(t, -data['gyro.y'], label='Pitch rate', linewidth=0.5)
    ax5.plot(t, data['gyro.z'], label='Yaw rate', linewidth=0.5)
    mark_events(ax5, t, crossings, tgt, nn_start_idx, nn_end_idx)
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Angular Velocity [rad/s]')
    ax5.set_title('Gyroscope Data')
    ax5.legend(loc='best')
    ax5.grid(True, alpha=0.3)

    # 6. Control commands
    ax6 = plt.subplot(3, 3, 6, sharex=ax2)
    # ax6.plot(t, data['controller.cmd_roll'], label='Roll Cmd', linewidth=1)
    # ax6.plot(t, data['controller.cmd_pitch'], label='Pitch Cmd', linewidth=1)
    # ax6.plot(t, data['controller.cmd_yaw'], label='Yaw Cmd', linewidth=1)
    ax6.plot(t, data['rlapp.nnOut2'], label='nnOut2', linewidth=0.5)
    ax6.plot(t, data['rlapp.nnOut0'], label='nnOut0', linewidth=0.5)
    ax6.plot(t, data['rlapp.nnOut3'], label='nnOut3', linewidth=0.5)
    ax6.plot(t, data['rlapp.nnOut1'], label='nnOut1', linewidth=0.5)
    mark_events(ax6, t, crossings, tgt, nn_start_idx, nn_end_idx)
    ax6.set_xlabel('Time [s]')
    ax6.set_ylabel('Command Value')
    ax6.set_title('NN Commands')
    ax6.legend(loc='best')
    ax6.grid(True, alpha=0.3)

    # 7. Motor PWM outputs
    ax7 = plt.subplot(3, 3, 7, sharex=ax2)
    # ax7.plot(t, data['rlapp.nnOut0'], label='M1', linewidth=0.5)
    # ax7.plot(t, data['rlapp.nnOut1'], label='M2', linewidth=0.5)
    # ax7.plot(t, data['rlapp.nnOut2'], label='M3', linewidth=0.5)
    # ax7.plot(t, data['rlapp.nnOut3'], label='M4', linewidth=0.5)
    ax7.plot(t, data['motor.m1'], label='M1', linewidth=0.5)
    ax7.plot(t, data['motor.m2'], label='M2', linewidth=0.5)
    ax7.plot(t, data['motor.m3'], label='M3', linewidth=0.5)
    ax7.plot(t, data['motor.m4'], label='M4', linewidth=0.5)
    mark_events(ax7, t, crossings, tgt, nn_start_idx, nn_end_idx)
    ax7.set_xlabel('Time [s]')
    ax7.set_ylabel('PWM Value')
    ax7.set_title('Motor Outputs')
    ax7.legend(loc='best')
    ax7.grid(True, alpha=0.3)

    # 8. Altitude vs Time
    ax8 = plt.subplot(3, 3, 8, sharex=ax2)
    ax8.plot(t, -data['stateEstimate.z'], 'b-', linewidth=1)
    ax8.fill_between(t, 0, -data['stateEstimate.z'], alpha=0.3)
    mark_events(ax8, t, crossings, tgt, nn_start_idx, nn_end_idx)
    ax8.set_xlabel('Time [s]')
    ax8.set_ylabel('Altitude [m]')
    ax8.set_title('Altitude vs Time')
    ax8.grid(True, alpha=0.3)

    # 9. Speed (magnitude of body velocity)
    ax9 = plt.subplot(3, 3, 9, sharex=ax2)
    speed = np.sqrt(bvx**2 + bvy**2 + bvz**2)
    ax9.plot(t, speed, 'purple', linewidth=1)
    ax9.fill_between(t, 0, speed, alpha=0.3, color='purple')
    mark_events(ax9, t, crossings, tgt, nn_start_idx, nn_end_idx)
    ax9.set_xlabel('Time [s]')
    ax9.set_ylabel('Speed [m/s]')
    ax9.set_title(f'Total speed ({vel_src})')
    ax9.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot RL controller flight data")
    parser.add_argument("logfile", default="log00.csv", nargs='?',
                        help="CSV log file to plot (default: log00.csv)")
    args = parser.parse_args()

    plot_rl_log(args.logfile)
