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

#include "mm_tof.h"
#include "param.h"
#include "log.h"

// ---- Innovation gate for the down-range (ToF) update ----
// Rejects brief, physically-impossible jumps in the ToF reading (e.g. an obstacle
// or ledge/gate/window sill passing under the drone) so the EKF's z COASTS on the
// accelerometer prediction through the glitch instead of lurching (the "jumping"
// on gates/windows). A change that PERSISTS past tofGateHold samples is treated as
// a real terrain step and accepted. Set tofGate=0 to disable.
// The reference is the LAST ACCEPTED reading, with an allowance that grows at a
// physical vertical speed:
//   |measured - lastAccepted| > gate + rate*(time since last accept)  -> reject (up to hold)
// A ramp is rejected (a real descent never moves 0.15 m in 25 ms); the floor is
// accepted the instant it is back, since it matches the reference.
static float    tofGate     = 0.10f;  // [m] base allowance vs the last accepted reading (0 = off)
static float    tofRate     = 0.30f;  // [m/s] the allowance grows at this rate while rejecting
static uint16_t tofGateHold = 40;     // accept after this many consecutive rejects (~real step; 40 = 1 s)
static float    tofStab     = 0.05f;  // [m] a reading must also be within this of the PREVIOUS raw sample (a blend ramp is not)
static uint16_t tofGateHoldDn = 200;  // give-up for a CLOSER reading (obstacle under the drone): 5 s vs tofGateHold for farther
static uint8_t  tofFlowSkip = 1;      // 1 = the flow update is skipped while this gate rejects (same obstacle in the flow camera)
static float    tofPrev     = -1.0f;  // previous raw reading [m]
static float    tofRef      = -1.0f;  // last accepted reading [m]
static float    tofPinStd   = 2.0f;   // while rejecting, pin z to tofRef with this x the reading's stdDev (0 = coast)
static uint32_t tofRefMs    = 0;      // ...and when
static uint16_t tofRejects  = 0;      // running count of consecutive rejects
static uint8_t  tofGated    = 0;      // 1 = last ToF update was rejected (for logging)
static float    tofInnov    = 0.0f;   // last innovation [m] (for tuning tofGate)

void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof, const bool isFlying)
{
  // Updates the filter with a measured distance in the zb direction using the
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[2][2] -> 0)
  if (fabs(this->R[2][2]) > 0.1 && this->R[2][2] > 0){
    float angle = fabsf(acosf(this->R[2][2])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }
    float predictedDistance = this->S[KC_STATE_Z] / cosf(angle);
    float measuredDistance = tof->distance; // [m]

    /*
    The sensor model (Pg.95-96, https://lup.lub.lu.se/student-papers/search/publication/8905295)
    
    h = z/((R*z_b).z_b) = z/cos(alpha)
    
    Here,
    h (Measured variable)[m] = Distance given by TOF sensor. This is the closest point from any surface to the sensor in the measurement cone
    z (Estimated variable)[m] = THe actual elevation of the crazyflie
    z_b = Basis vector in z direction of body coordinate system
    R = Rotation matrix made from ZYX Tait-Bryan angles. Assumed to be stationary
    alpha = angle between [line made by measured point <---> sensor] and [the intertial z-axis] 
    */

    h[KC_STATE_Z] = 1 / cosf(angle); // This just acts like a gain for the sensor model. Further updates are done in the scalar update function below

    if (!isFlying) {
      // When not flying, we want to correct the height to zero
      kalmanCoreScalarUpdate(this, &H, (0.02f-predictedDistance), 0.0f);
    }
    // Scalar update (with innovation gate while flying — see note at top)
    if (isFlying) {
      float innovation = measuredDistance - predictedDistance;
      tofInnov = innovation;
      uint32_t now = tof->timestamp;                       // ticks = ms
      if (measuredDistance < 0.02f) { return; }            // no target / saturated: neither accept nor count
      if (tofRef < 0.0f) { tofRef = measuredDistance; tofRefMs = now; tofPrev = measuredDistance; }
      float allowed = tofGate + tofRate * (float)(now - tofRefMs) * 0.001f;
      // Blends are transitional: they differ from the previous raw sample by 0.1-1 m,
      // real motion by < 15 mm per sample. So a reading must ALSO be stable vs the
      // previous sample to be accepted (rejects a slowly-ramping blend edge).
      bool stable = fabsf(measuredDistance - tofPrev) < tofStab;
      tofPrev = measuredDistance;
      uint16_t hold = (measuredDistance < tofRef) ? tofGateHoldDn : tofGateHold;   // closer = obstacle: be patient
      if (tofGate > 0.0001f && (fabsf(measuredDistance - tofRef) > allowed || !stable) && tofRejects < hold) {
        tofRejects++;
        tofGated = 1;
        // Glitch: pin the filter to the last good reading (the drone is not commanded
        // in z while crossing an obstacle, so "height unchanged" is the truthful
        // measurement). Softer than a real reading. tofPinStd=0 -> let z coast.
        if (tofPinStd > 0.0f) kalmanCoreScalarUpdate(this, &H, tofRef - predictedDistance, tof->stdDev * tofPinStd);
      } else {
        tofRejects = 0;  // plausible, or a persisted real change -> accept & re-converge
        tofGated = 0;
        tofRef = measuredDistance; tofRefMs = now;
        kalmanCoreScalarUpdate(this, &H, innovation, tof->stdDev);
      }
    }
  }
}

/**
 * Down-range (ToF) innovation gate — rejects obstacle/gate/window height glitches
 * before they reach the EKF, so the z estimate does not jump. tofGate.gate=0 = off.
 */
PARAM_GROUP_START(tofGate)
/**
 * @brief Base allowance [m] vs the LAST ACCEPTED reading. 0 = off. (default 0.10)
 */
PARAM_ADD(PARAM_FLOAT, gate, &tofGate)
/**
 * @brief The allowance grows at this vertical rate [m/s] while rejecting. (default 0.30)
 */
PARAM_ADD(PARAM_FLOAT, rate, &tofRate)
/**
 * @brief While rejecting, pin z to the last accepted reading with this x its stdDev; 0 = let z coast. (default 2.0)
 */
PARAM_ADD(PARAM_FLOAT, pin, &tofPinStd)
/**
 * @brief Accept a gated change after this many consecutive rejects (real terrain step). (default 40 = 1 s)
 */
PARAM_ADD(PARAM_UINT16, hold, &tofGateHold)
/**
 * @brief Give-up for a reading CLOSER than the reference (obstacle under the drone), in samples. (default 200 = 5 s)
 */
PARAM_ADD(PARAM_UINT16, holdDn, &tofGateHoldDn)
/**
 * @brief A reading must be within this [m] of the previous raw sample to be accepted (rejects blend ramps). (default 0.05)
 */
PARAM_ADD(PARAM_FLOAT, stab, &tofStab)
/**
 * @brief 1 = skip the optical-flow update while the ToF gate rejects (the flow camera sees the same obstacle). (default 1)
 */
PARAM_ADD(PARAM_UINT8, flowSkip, &tofFlowSkip)
PARAM_GROUP_STOP(tofGate)

LOG_GROUP_START(tofGate)
/**
 * @brief 1 = the most recent ToF update was rejected by the gate
 */
LOG_ADD(LOG_UINT8, gated, &tofGated)
/**
 * @brief Most recent ToF innovation (measured - predicted) [m]
 */
LOG_ADD(LOG_FLOAT, innov, &tofInnov)
/**
 * @brief Consecutive rejects (climbs to tofGate.hold then forces accept)
 */
LOG_ADD(LOG_UINT16, rej, &tofRejects)
/**
 * @brief The reference: last accepted ToF reading [m]
 */
LOG_ADD(LOG_FLOAT, ref, &tofRef)
LOG_GROUP_STOP(tofGate)

bool kalmanTofGateRejecting(void) { return tofFlowSkip && tofGated; }
