/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
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
 */

#include "mm_flow.h"
#include "log.h"
#include "param.h"
#include "platform_defaults.h"
#include "FreeRTOS.h"
#include "task.h"

#define FLOW_RESOLUTION 0.10f //We do get the measurements in 10x the motion pixels (experimentally measured)

// ============================================================================
// Gyro history buffer for delay compensation
// ============================================================================
#define GYRO_HISTORY_SIZE 32

typedef struct {
  Axis3f gyro;
  uint32_t timestamp;  // ms
} GyroHistoryEntry;

static GyroHistoryEntry gyroHistory[GYRO_HISTORY_SIZE];
static uint8_t gyroHistoryIdx = 0;
static bool gyroHistoryFull = false;

// Configurable delay compensation (ms)
static float flowDelayMs = 30.0f;
// Explicit enable switch for delay compensation (overrides delayMs when 0)
static uint8_t flowDelayEnabled = 1;

// Flow deck position relative to CoG in body frame (meters)
static Axis3f flowdeckPos = { .axis = { FLOWDECK_POS_X, FLOWDECK_POS_Y, FLOWDECK_POS_Z } };

// Add a gyro sample to history
void mmFlowAddGyroSample(const Axis3f *gyro, uint32_t timestampMs) {
  gyroHistory[gyroHistoryIdx].gyro = *gyro;
  gyroHistory[gyroHistoryIdx].timestamp = timestampMs;
  gyroHistoryIdx = (gyroHistoryIdx + 1) % GYRO_HISTORY_SIZE;
  if (gyroHistoryIdx == 0) {
    gyroHistoryFull = true;
  }
}

// Get the gyro reading from a specific time ago (with interpolation)
static Axis3f getGyroAtDelay(float delayMs, const Axis3f *currentGyro) {
  if (!gyroHistoryFull && gyroHistoryIdx == 0) {
    // No history yet, use current
    return *currentGyro;
  }

  uint32_t now = xTaskGetTickCount();
  uint32_t targetTime = now - (uint32_t)delayMs;

  // Search for closest sample
  uint8_t count = gyroHistoryFull ? GYRO_HISTORY_SIZE : gyroHistoryIdx;
  int bestIdx = -1;
  uint32_t bestDiff = UINT32_MAX;

  for (uint8_t i = 0; i < count; i++) {
    uint32_t diff = (gyroHistory[i].timestamp > targetTime) ? 
                    (gyroHistory[i].timestamp - targetTime) : 
                    (targetTime - gyroHistory[i].timestamp);
    if (diff < bestDiff) {
      bestDiff = diff;
      bestIdx = i;
    }
  }

  if (bestIdx >= 0 && bestDiff < 50) {  // Only use if within 50ms
    return gyroHistory[bestIdx].gyro;
  }

  // Fallback to current
  return *currentGyro;
}

// TODO remove the temporary test variables (used for logging)
static float predictedNX;
static float predictedNY;
static float measuredNX;
static float measuredNY;

void kalmanCoreUpdateWithFlow(kalmanCoreData_t* this, const flowMeasurement_t *flow, const Axis3f *gyro, const bool isFlying)
{
  // Inclusion of flow measurements in the EKF done by two scalar updates

  // Get historical gyro if delay compensation is enabled
  Axis3f gyroCompensated;
  if (flowDelayEnabled && flowDelayMs > 0.0f) {
    gyroCompensated = getGyroAtDelay(flowDelayMs, gyro);
  } else {
    gyroCompensated = *gyro;
  }

  // ~~~ Camera constants ~~~
  // The angle of aperture is guessed from the raw data register and thankfully look to be symmetric
  float Npix = 35.0;         // [pixels] (same in x and y)
  float thetapix = 0.71674f; // [rad] 2*sin(42/2); 42 degrees is the angle of aperture

  // ~~~ Body rates ~~~ (delay-compensated when flowDelayEnabled)
  float omegax_b = gyroCompensated.x * DEG_TO_RAD;
  float omegay_b = gyroCompensated.y * DEG_TO_RAD;
  float omegaz_b = gyroCompensated.z * DEG_TO_RAD;

  // Velocities in body frame
  float dx_b = this->S[KC_STATE_PX];
  float dy_b = this->S[KC_STATE_PY];

  // Saturate height in prediction and correction to avoid singularities
  float z_g = 0.0;
  if ( this->S[KC_STATE_Z] < 0.1f ) {
      z_g = 0.1;
  } else {
      z_g = this->S[KC_STATE_Z];
  }

  // Lever-arm induced translational velocity at the camera point: omega x r
  float v_cam_bx_add = omegay_b * flowdeckPos.z - omegaz_b * flowdeckPos.y;
  float v_cam_by_add = omegaz_b * flowdeckPos.x - omegax_b * flowdeckPos.z;

  // Effective camera-point velocities in body frame
  float v_cam_bx = dx_b + v_cam_bx_add;
  float v_cam_by = dy_b + v_cam_by_add;

  // ~~~ X velocity prediction and update ~~~
  // predicts the number of accumulated pixels in the x-direction
  float hx[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 Hx = {1, KC_STATE_DIM, hx};
  predictedNX = (flow->dt * Npix / thetapix) * ((v_cam_bx * this->R[2][2] / z_g) - omegay_b);
  measuredNX = flow->dpixelx*FLOW_RESOLUTION;

  // derive measurement equation with respect to dx (and z?)
  hx[KC_STATE_Z]  = (Npix * flow->dt / thetapix) * ((this->R[2][2] * v_cam_bx) / (-z_g * z_g));
  hx[KC_STATE_PX] = (Npix * flow->dt / thetapix) * (this->R[2][2] / z_g);

  //First update
  if (!isFlying) {
    kalmanCoreScalarUpdate(this, &Hx, (0.0f-predictedNX), 0.0f);
  }

  if (isFlying && (this->S[KC_STATE_Z] > 0.12f)) {
    kalmanCoreScalarUpdate(this, &Hx, (measuredNX-predictedNX), flow->stdDevX*FLOW_RESOLUTION);
  }

  // ~~~ Y velocity prediction and update ~~~
  float hy[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 Hy = {1, KC_STATE_DIM, hy};
  predictedNY = (flow->dt * Npix / thetapix) * ((v_cam_by * this->R[2][2] / z_g) + omegax_b);
  measuredNY = flow->dpixely*FLOW_RESOLUTION;

  // derive measurement equation with respect to dy (and z?)
  hy[KC_STATE_Z]  = (Npix * flow->dt / thetapix) * ((this->R[2][2] * v_cam_by) / (-z_g * z_g));
  hy[KC_STATE_PY] = (Npix * flow->dt / thetapix) * (this->R[2][2] / z_g);

  if (!isFlying) {
    kalmanCoreScalarUpdate(this, &Hy, (0.0f-predictedNY), 0.0f);
  }

  if (isFlying && (this->S[KC_STATE_Z] > 0.12f)) {
    kalmanCoreScalarUpdate(this, &Hy, (measuredNY-predictedNY), flow->stdDevY*FLOW_RESOLUTION);
  }
}

/**
 * Predicted and measured values of the X and Y direction of the flowdeck
 */
LOG_GROUP_START(kalman_pred)

/**
 * @brief Flow sensor predicted dx  [pixels/frame]
 * 
 *  note: rename to kalmanMM.flowX?
 */
  LOG_ADD(LOG_FLOAT, predNX, &predictedNX)
/**
 * @brief Flow sensor predicted dy  [pixels/frame]
 * 
 *  note: rename to kalmanMM.flowY?
 */
  LOG_ADD(LOG_FLOAT, predNY, &predictedNY)
/**
 * @brief Flow sensor measured dx  [pixels/frame]
 * 
 *  note: This is the same as motion.deltaX, so perhaps remove this?
 */
  LOG_ADD(LOG_FLOAT, measNX, &measuredNX)
/**
 * @brief Flow sensor measured dy  [pixels/frame]
 * 
 *  note: This is the same as motion.deltaY, so perhaps remove this?
 */
  LOG_ADD(LOG_FLOAT, measNY, &measuredNY)
LOG_GROUP_STOP(kalman_pred)

/**
 * Parameters for flow delay compensation
 */
PARAM_GROUP_START(motion)
/**
 * @brief Flow sensor delay compensation in milliseconds
 *
 * Set to the estimated delay between when the flow sensor captures
 * the image and when the measurement is processed by the EKF.
 * This uses historical gyro data to improve the prediction.
 */
  PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, delayMs, &flowDelayMs)
/**
 * @brief Enable/disable flow delay compensation (1 = on, 0 = off).
 *
 * When disabled, the current gyro sample is used regardless of delayMs.
 */
  PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, delayEnabled, &flowDelayEnabled)
PARAM_GROUP_STOP(motion)

/**
 * Flowdeck properties
 */
PARAM_GROUP_START(flowdeck)
  /**
   * @brief Flow deck X position relative to center of mass, body frame (meters)
   */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, flowdeckPos_x, &flowdeckPos.x)
  /**
   * @brief Flow deck Y position relative to center of mass, body frame (meters)
   */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, flowdeckPos_y, &flowdeckPos.y)
  /**
   * @brief Flow deck Z position relative to center of mass, body frame (meters)
   */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, flowdeckPos_z, &flowdeckPos.z)
PARAM_GROUP_STOP(flowdeck)
