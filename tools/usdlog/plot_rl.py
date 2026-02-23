#!/usr/bin/env python3
"""
Plot RL controller flight data from CSV log files
"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse

def plot_rl_log(csv_file):
    # Read CSV data
    data = pd.read_csv(csv_file)

    # Convert timestamp to relative time in seconds
    t = (data['timestamp'] - data['timestamp'].iloc[0]) / 1000.0

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
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Position [m]')
    ax2.set_title('Position vs Time')
    ax2.legend(loc='best')
    ax2.grid(True, alpha=0.3)

    # 3. Velocity vs Time
    ax3 = plt.subplot(3, 3, 3, sharex=ax2)
    ax3.plot(t, data['stateEstimate.vx'], label='Vx', linewidth=1)
    ax3.plot(t, -data['stateEstimate.vy'], label='Vy', linewidth=1)
    ax3.plot(t, -data['stateEstimate.vz'], label='Vz', linewidth=1)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Velocity [m/s]')
    ax3.set_title('Velocity vs Time')
    ax3.legend(loc='best')
    ax3.grid(True, alpha=0.3)

    # 4. Attitude (Roll, Pitch, Yaw)
    ax4 = plt.subplot(3, 3, 4, sharex=ax2)
    ax4.plot(t, data['stateEstimate.roll'], label='Roll', linewidth=1)
    ax4.plot(t, data['stateEstimate.pitch'], label='Pitch', linewidth=1)
    ax4.plot(t, -data['stateEstimate.yaw'], label='Yaw', linewidth=1)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Angle [rad]')
    ax4.set_title('Attitude')
    ax4.legend(loc='best')
    ax4.grid(True, alpha=0.3)

    # 5. Gyro data
    ax5 = plt.subplot(3, 3, 5, sharex=ax2)
    ax5.plot(t, data['gyro.x'], label='Roll rate', linewidth=0.5)
    ax5.plot(t, -data['gyro.z'], label='Pitch rate', linewidth=0.5)
    ax5.plot(t, -data['gyro.y'], label='Yaw rate', linewidth=0.5)
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Angular Velocity [rad/s]')
    ax5.set_title('Gyroscope Data')
    ax5.legend(loc='best')
    ax5.grid(True, alpha=0.3)

    # 6. Control commands
    ax6 = plt.subplot(3, 3, 6, sharex=ax2)
    ax6.plot(t, data['controller.cmd_roll'], label='Roll Cmd', linewidth=1)
    ax6.plot(t, data['controller.cmd_pitch'], label='Pitch Cmd', linewidth=1)
    ax6.plot(t, data['controller.cmd_yaw'], label='Yaw Cmd', linewidth=1)
    ax6.set_xlabel('Time [s]')
    ax6.set_ylabel('Command Value')
    ax6.set_title('Control Commands')
    ax6.legend(loc='best')
    ax6.grid(True, alpha=0.3)

    # 7. Motor PWM outputs
    ax7 = plt.subplot(3, 3, 7, sharex=ax2)
    ax7.plot(t, data['rlapp.pwmM1'], label='M1', linewidth=0.5)
    ax7.plot(t, data['rlapp.pwmM2'], label='M2', linewidth=0.5)
    ax7.plot(t, data['rlapp.pwmM3'], label='M3', linewidth=0.5)
    ax7.plot(t, data['rlapp.pwmM4'], label='M4', linewidth=0.5)
    ax7.set_xlabel('Time [s]')
    ax7.set_ylabel('PWM Value')
    ax7.set_title('Motor Outputs')
    ax7.legend(loc='best')
    ax7.grid(True, alpha=0.3)

    # 8. Altitude vs Time
    ax8 = plt.subplot(3, 3, 8, sharex=ax2)
    ax8.plot(t, -data['stateEstimate.z'], 'b-', linewidth=1)
    ax8.fill_between(t, 0, -data['stateEstimate.z'], alpha=0.3)
    ax8.set_xlabel('Time [s]')
    ax8.set_ylabel('Altitude [m]')
    ax8.set_title('Altitude vs Time')
    ax8.grid(True, alpha=0.3)

    # 9. Speed (magnitude of velocity)
    ax9 = plt.subplot(3, 3, 9, sharex=ax2)
    speed = np.sqrt(data['stateEstimate.vx']**2 + data['stateEstimate.vy']**2 + data['stateEstimate.vz']**2)
    ax9.plot(t, speed, 'purple', linewidth=1)
    ax9.fill_between(t, 0, speed, alpha=0.3, color='purple')
    ax9.set_xlabel('Time [s]')
    ax9.set_ylabel('Speed [m/s]')
    ax9.set_title('Total Speed')
    ax9.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot RL controller flight data")
    parser.add_argument("logfile", default="log00.csv", nargs='?',
                        help="CSV log file to plot (default: log00.csv)")
    args = parser.parse_args()

    plot_rl_log(args.logfile)
