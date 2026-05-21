/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB & Flapper Drones (https://flapper-drones.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * platform_defaults_flapper.h - platform specific default values for the Flapper Nimble+
 */

#pragma once

#ifndef __INCLUDED_FROM_PLATFORM_DEFAULTS__
    #pragma GCC error "Do not include this file directly, include platform_defaults.h instead."
#endif

// Defines for default values in the flapper platform

// Default values for battery limits
#define DEFAULT_BAT_LOW_VOLTAGE                   6.4f
#define DEFAULT_BAT_CRITICAL_LOW_VOLTAGE          6.0f
#define DEFAULT_BAT_LOW_DURATION_TO_TRIGGER_SEC   5

// Default value for system shutdown in minutes after radio silence.
// Requires kbuild config ENABLE_AUTO_SHUTDOWN to be activated.
#define DEFAULT_SYSTEM_SHUTDOWN_TIMEOUT_MIN       5

// Default PID gains
#define PID_ROLL_RATE_KP  50.0
#define PID_ROLL_RATE_KI  0.0
#define PID_ROLL_RATE_KD  0.0
#define PID_ROLL_RATE_KFF 0.0
#define PID_ROLL_RATE_INTEGRATION_LIMIT    33.3

#define PID_PITCH_RATE_KP  50.0
#define PID_PITCH_RATE_KI  0.0
#define PID_PITCH_RATE_KD  0.0
#define PID_PITCH_RATE_KFF 0.0
#define PID_PITCH_RATE_INTEGRATION_LIMIT   33.3

#define PID_YAW_RATE_KP  80.0
#define PID_YAW_RATE_KI  0.0
#define PID_YAW_RATE_KD  0.0
#define PID_YAW_RATE_KFF 220.0
#define PID_YAW_RATE_INTEGRATION_LIMIT     166.7

#define PID_ROLL_KP  10.0
#define PID_ROLL_KI  0.0
#define PID_ROLL_KD  0.2
#define PID_ROLL_KFF 0.0
#define PID_ROLL_INTEGRATION_LIMIT    20.0

#define PID_PITCH_KP  13.0
#define PID_PITCH_KI  0.0
#define PID_PITCH_KD  1.0
#define PID_PITCH_KFF 0.0
#define PID_PITCH_INTEGRATION_LIMIT   20.0

#define PID_YAW_KP  8.0
#define PID_YAW_KI  0.0
#define PID_YAW_KD  0.35
#define PID_YAW_KFF 0.0
#define PID_YAW_INTEGRATION_LIMIT     360.0

#define PID_VEL_X_KP 15.0f
#define PID_VEL_X_KI 1.0f
#define PID_VEL_X_KD 0.0f
#define PID_VEL_X_KFF 15.0f

#define PID_VEL_Y_KP 10.0f
#define PID_VEL_Y_KI 1.0f
#define PID_VEL_Y_KD 0.0f
#define PID_VEL_Y_KFF 8.0f

#define PID_VEL_Z_KP 12.5f
#define PID_VEL_Z_KI 0.5f
#define PID_VEL_Z_KD 0.0f
#define PID_VEL_Z_KFF 0.0f

#define PID_VEL_Z_KP_BARO_Z_HOLD 4.0f
#define PID_VEL_Z_KI_BARO_Z_HOLD 3.0f
#define PID_VEL_Z_KD_BARO_Z_HOLD 1.0f
#define PID_VEL_Z_KFF_BARO_Z_HOLD 0.0f

#define PID_VEL_ROLL_MAX 30.0f
#define PID_VEL_PITCH_MAX 30.0f
#define PID_VEL_THRUST_BASE 40000.0f
#define PID_VEL_THRUST_BASE_BARO_Z_HOLD 40000.0f
#define PID_VEL_THRUST_MIN 20000.0f

#define PID_POS_X_KP 1.5f
#define PID_POS_X_KI 0.0f
#define PID_POS_X_KD 0.0f
#define PID_POS_X_KFF 0.0f

#define PID_POS_Y_KP 1.5f
#define PID_POS_Y_KI 0.0f
#define PID_POS_Y_KD 0.0f
#define PID_POS_Y_KFF 0.0f

#define PID_POS_Z_KP 5.0f
#define PID_POS_Z_KI 0.5f
#define PID_POS_Z_KD 0.0f
#define PID_POS_Z_KFF 0.0f

#define PID_POS_VEL_X_MAX 2.0f
#define PID_POS_VEL_Y_MAX 2.0f
#define PID_POS_VEL_Z_MAX 1.0f

// PID filter configuration
#define ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ 20.0f
#define ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ 20.0f
#define ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ 5.0f
#define ATTITUDE_RATE_LPF_ENABLE true
#define PID_VEL_XY_FILT_CUTOFF 10.0f
#define PID_VEL_Z_FILT_CUTOFF 10.0f

// IMU alignment
//////////////////////////////////
#if CONFIG_POWER_DISTRIBUTION_FLAPPER_REVB
    #define IMU_PHI                   0.0f
    #define IMU_THETA                 90.0f
    #define IMU_PSI                   180.0f
#else
    #define IMU_PHI                   0.0f
    #define IMU_THETA                -90.0f
    #define IMU_PSI                   180.0f
#endif

// Tumble check settings //
///////////////////////////
#define SUPERVISOR_TUMBLE_CHECK_ACCEPTED_TILT_ACCZ 0.0f
#define SUPERVISOR_TUMBLE_CHECK_ACCEPTED_TILT_TIME 2000
#define SUPERVISOR_TUMBLE_CHECK_ACCEPTED_UPSIDEDOWN_ACCZ -0.5f
#define SUPERVISOR_TUMBLE_CHECK_ACCEPTED_UPSIDEDOWN_TIME 500

#define YAW_MAX_DELTA 30.0


// Drag and center of pressure
// NOTE: For X, these values are not constant but depends on dihedral angle. Influence seems to be negligible.
// Values from offline EKF replay tuning (ekf_data branch: "Tuned with cop and flowdeck lever arm").
#define DRAG_B_X 4.39468f
#define DRAG_B_Y 2.88896f
#define DRAG_B_Z 0.0611769f
#define CENTER_OF_PRESSURE_X 0.0f
#define CENTER_OF_PRESSURE_Y 0.0f
#define CENTER_OF_PRESSURE_Z 0.03f

// EKF process / measurement noise — offline replay tuning (ekf_data branch)
#define PROC_NOISE_ACC_XY 1.05006f
#define PROC_NOISE_ACC_Z 0.604273f
#define MEAS_NOISE_GYRO_ROLLPITCH 0.0521776f
#define MEAS_NOISE_GYRO_YAW 0.116742f

// NOTE: the replay's flow tuning (flow_std_fixed_x/y, flow_resolution) was fit
// on MTF-02 logs, so it is applied in mtf02deck.c (flowScale + per-axis std),
// not here. The shared mm_flow.c FLOW_RESOLUTION stays at 0.10 for the PMW3901
// standard flow deck.

// Flow deck position offset (in meters) — flapper-specific: flowdeck mounted below the legs
// Assumes placement per https://github.com/flapper-drones/3Dmodels/blob/main/Bitcraze_flowdeck_support%20v17.stl
#define FLOWDECK_POS_X 0.0f
#define FLOWDECK_POS_Y 0.0f
#define FLOWDECK_POS_Z -0.12f
