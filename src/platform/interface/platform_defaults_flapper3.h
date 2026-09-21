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
 * platform_defaults_flapper3.h - platform specific default values for the Flapper Nimble Triple
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

#define PID_YAW_RATE_KP  20.0
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
#define IMU_PHI                   0.0f
#define IMU_THETA                -90.0f
#define IMU_PSI                   180.0f

// Tumble check settings //
///////////////////////////
#define SUPERVISOR_TUMBLE_CHECK_ACCEPTED_TILT_ACCZ 0.0f
#define SUPERVISOR_TUMBLE_CHECK_ACCEPTED_TILT_TIME 2000
#define SUPERVISOR_TUMBLE_CHECK_ACCEPTED_UPSIDEDOWN_ACCZ -0.5f
#define SUPERVISOR_TUMBLE_CHECK_ACCEPTED_UPSIDEDOWN_TIME 200

#define YAW_MAX_DELTA 30.0

// ---------------------------------------------------------------------------
// Flow-only EKF tuning — CARRIED OVER from the 2-wing Nimble+ (IMAV2026 work).
// These are the compile-time boot defaults; all are live-tunable / persistent
// via the kalman.dragB* / kalman.drag_r* params and set_ekf_params.py, so the
// runtime values win after that script runs.
//
// NOTE: drag and the flowdeck lever arm are PHYSICAL properties of the airframe.
// These numbers were tuned on the 2-wing Nimble+; the 3-wing Triple has
// different aero and (possibly) a different flowdeck mounting height, so treat
// them as a starting point and RE-VALIDATE with ekf_replay.py on Triple data.
// (Process/gyro noise live in kalman_core_params_defaults.h and flow std/scale
// in mtf02deck.c — both global, so they already carry over unchanged.)
// ---------------------------------------------------------------------------
#ifndef EKF_DRAG_BX
    #define EKF_DRAG_BX 4.39468f
#endif
#ifndef EKF_DRAG_BY
    #define EKF_DRAG_BY 2.88896f
#endif
#ifndef EKF_DRAG_BZ
    #define EKF_DRAG_BZ 0.0611769f
#endif
#ifndef EKF_DRAG_RX
    #define EKF_DRAG_RX 0.0f
#endif
#ifndef EKF_DRAG_RY
    #define EKF_DRAG_RY 0.0f
#endif
#ifndef EKF_DRAG_RZ
    #define EKF_DRAG_RZ 0.03f
#endif

#define FLOWDECK_POS_X 0.0f
#define FLOWDECK_POS_Y 0.0f
#define FLOWDECK_POS_Z -0.12f
