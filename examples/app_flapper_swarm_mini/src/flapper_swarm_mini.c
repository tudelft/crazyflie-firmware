/*
 * Mini Flapper Swarm App
 * 
 * Simple circular flight around a beacon with collision avoidance.
 * 
 * Drone behavior:
 *   - During initial hover, measures average distance to beacon
 *   - Uses measured distance as circle radius, calculates yaw rate accordingly
 *   - Both drones fly CW circles around beacon
 *   - If drones get too close, enter DANCE state:
 *     - Drones alternate who lands (drone 1 first, then drone 2, etc.)
 *     - Landing drone waits for peer to be far enough, then takes off
 *     - Other drone continues circling
 *   - Emergency land if distance to beacon exceeds outer bound
 * 
 * States: CIRCLE, DANCE, (emergency land handled separately)
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "MINIFLAPPERSWARM"
#include "debug.h"

#include "log.h"
#include "param.h"
#include "param_logic.h"
#include "commander.h"
#include "stabilizer_types.h"
#include "supervisor.h"
#include "configblock.h"

// ============================================================================
// Timestamp helper
// ============================================================================
static inline float getTimestamp(void) {
  return (float)(xTaskGetTickCount() * portTICK_PERIOD_MS) / 1000.0f;
}

// ============================================================================
// Drone ID (derived from radio address at startup)
// ============================================================================
static uint8_t droneId = 0;

// ============================================================================
// Runtime-configurable flight parameters
// ============================================================================
static float targetHeightM = 1.0f;           // Flight height (m)
static float circleSpeedMps = 0.5f;          // Forward speed while circling (m/s)
static uint16_t outerBoundMm = 2000U;        // Emergency land if d0 > this (mm)
static uint16_t peerCloseMm = 1000U;         // Enter DANCE if peer distance < this (mm)
static uint16_t peerSafeMm = 1500U;          // Exit DANCE when peer distance > this (mm)
static uint32_t demoTimeMs = 60000U;         // Demo duration (ms)
static uint8_t confirmCount = 3;             // Samples to confirm state transitions

// ============================================================================
// Calculated circle parameters (determined during hover)
// ============================================================================
static float measuredRadiusM = 1.5f;         // Measured radius during hover (m)
static float circleYawRateDps = 20.0f;       // Calculated yaw rate (deg/s)

// ============================================================================
// Fixed parameters
// ============================================================================
#define RAMP_TIME_MS 1500U
#define HOVER_BEFORE_CIRCLE_MS 1500U  // Reduced from 3000 to minimize drift
#define LAND_VZ_MPS 0.4f
#define CUT_Z_M 0.04f

// Radius measurement parameters
#define RADIUS_SAMPLE_INTERVAL_MS 50  // Sample distance every 50ms during hover
#define MIN_RADIUS_M 0.5f             // Minimum allowed radius
#define MAX_RADIUS_M 2.5f             // Maximum allowed radius

// Drone 1: RC trigger (active low)
#define AUX_RC_ACTIVE_THRESH 1400
// Drone 2+: UWB trigger (active high)
#define AUX_UWB_ACTIVE_THRESHOLD 0U

// Height threshold for "landed"
#define PEER_LANDED_HEIGHT_M 0.1f

// Dance state constants
#define PEER_LANDED_CONFIRM_COUNT 3
#define REJOIN_CONFIRM_COUNT 3

// ============================================================================
// Log variable IDs
// ============================================================================
static logVarId_t idDistance0   = (logVarId_t)0xFFFF;  // distance to beacon
static logVarId_t idZ           = (logVarId_t)0xFFFF;

// Drone 1 specific
static logVarId_t idCppmAux0    = (logVarId_t)0xFFFF;  // RC trigger
static logVarId_t idDistance2   = (logVarId_t)0xFFFF;  // distance to drone 2
static logVarId_t idHeight2     = (logVarId_t)0xFFFF;  // height of drone 2

// Drone 2+ specific
static logVarId_t idRangingAux1 = (logVarId_t)0xFFFF;  // UWB trigger
static logVarId_t idRangingAux2 = (logVarId_t)0xFFFF;  // UWB kill switch
static logVarId_t idDistance1   = (logVarId_t)0xFFFF;  // distance to drone 1
static logVarId_t idHeight1     = (logVarId_t)0xFFFF;  // height of drone 1

// ============================================================================
// State machine
// ============================================================================
typedef enum {
  STATE_CIRCLE = 0,
  STATE_DANCE  = 1
} FlightState;

static uint8_t currentState = STATE_CIRCLE;  // Exposed for logging

// ============================================================================
// State variables
// ============================================================================
static uint8_t outerBoundCount = 0;
static uint8_t peerCloseCount = 0;
static uint8_t peerFarCount = 0;
static uint8_t peerLandedCount = 0;
static uint8_t rejoinCount = 0;

// Dance state tracking
static bool danceHasLanded = false;
static bool danceHasTakenOff = false;
static bool myTurnToLand = false;  // Alternates each DANCE
static uint8_t danceCount = 0;     // Counts dance encounters for alternating

// Emergency abort flag
static volatile bool seqAbort = false;

// ============================================================================
// Utility functions
// ============================================================================
static inline void ensureLogId(logVarId_t* id, const char* group, const char* name) {
  if (!logVarIdIsValid(*id)) {
    *id = logGetVarId(group, name);
  }
}

// ============================================================================
// Calculate yaw rate for circular motion
// ============================================================================
// For circular motion: yawRate (deg/s) = (speed / radius) * (180 / PI)
static float calculateYawRate(float speedMps, float radiusM) {
  // radiusM is already in meters here
  float yawRateRad = speedMps / radiusM;
  float yawRateDeg = yawRateRad * (180.0f / 3.14159265f);
  return yawRateDeg;
}

// ============================================================================
// Drone-specific behavior
// ============================================================================
static inline bool isTriggerActive(void) {
  if (droneId == 1) {
    const int16_t v = logGetInt(idCppmAux0);
    return (v > 0) && (v < AUX_RC_ACTIVE_THRESH);
  } else {
    const uint32_t v = logGetUint(idRangingAux1);
    return v > AUX_UWB_ACTIVE_THRESHOLD;
  }
}

static inline bool isKillActive(void) {
  if (droneId == 1) {
    return false;
  } else {
    const uint32_t v = logGetUint(idRangingAux2);
    return v > AUX_UWB_ACTIVE_THRESHOLD;
  }
}

static inline uint32_t getPeerDistance(void) {
  if (droneId == 1) {
    return logGetUint(idDistance2);
  } else {
    return logGetUint(idDistance1);
  }
}

static inline float getPeerHeight(void) {
  if (droneId == 1) {
    return logGetFloat(idHeight2);
  } else {
    return logGetFloat(idHeight1);
  }
}

static inline bool isPeerLanded(void) {
  float peerHeight = getPeerHeight();
  return (peerHeight >= 0.0f && peerHeight < PEER_LANDED_HEIGHT_M);
}

static inline bool checkKillAndDisarm(void) {
  if (isKillActive()) {
    supervisorRequestArming(false);
    commanderRelaxPriority();
    return true;
  }
  return false;
}

// ============================================================================
// Command helpers
// ============================================================================
static void sendHover(float vx, float vy, float z, float yawRateDeg) {
  setpoint_t sp;
  memset(&sp, 0, sizeof(sp));

  sp.mode.z = modeAbs;
  sp.position.z = z;

  sp.mode.x = modeVelocity;
  sp.mode.y = modeVelocity;
  sp.velocity_body = true;
  sp.velocity.x = vx;
  sp.velocity.y = vy;
  sp.mode.yaw = modeVelocity;
  sp.attitudeRate.yaw = yawRateDeg;

  commanderSetSetpoint(&sp, 3);
}

// ============================================================================
// Emergency check
// ============================================================================
static bool checkEmergencyLand(void) {
  const uint32_t d0 = logGetUint(idDistance0);
  
  if (d0 > 0) {
    // Convert slant distance to horizontal distance
    float slantDistM = (float)d0 / 1000.0f;
    float slantDistSq = slantDistM * slantDistM;
    float heightSq = targetHeightM * targetHeightM;
    
    float horizontalDistM = 0.0f;
    if (slantDistSq > heightSq) {
      horizontalDistM = sqrtf(slantDistSq - heightSq);
    }
    
    // Convert outer bound to meters for comparison
    float outerBoundM = (float)outerBoundMm / 1000.0f;
    
    if (horizontalDistM > outerBoundM) {
      if (++outerBoundCount >= confirmCount) {
        if (!seqAbort) {
          DEBUG_PRINT("[%.2f] Emergency: horizontal dist=%.2f m > %.2f m (outer bound)\n",
                      (double)getTimestamp(), (double)horizontalDistM, (double)outerBoundM);
        }
        seqAbort = true;
        return true;
      }
    } else {
      outerBoundCount = 0;
    }
  }
  
  return false;
}

// ============================================================================
// Landing and takeoff
// ============================================================================
static void landToZero(void) {
  setpoint_t sp;
  memset(&sp, 0, sizeof(sp));
  sp.mode.x = modeVelocity; sp.velocity.x = 0;
  sp.mode.y = modeVelocity; sp.velocity.y = 0;
  sp.mode.z = modeVelocity; sp.velocity.z = -LAND_VZ_MPS;
  sp.mode.yaw = modeVelocity; sp.attitudeRate.yaw = 0;
  sp.velocity_body = true;

  while (1) {
    float z = logGetFloat(idZ);
    if (z >= 0.0f && z <= CUT_Z_M) break;
    commanderSetSetpoint(&sp, 3);
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  // Cut controllers
  setpoint_t cut;
  memset(&cut, 0, sizeof(cut));
  cut.mode.x = modeDisable;
  cut.mode.y = modeDisable;
  cut.mode.z = modeDisable;
  cut.mode.yaw = modeDisable;
  cut.thrust = 0;
  for (int i = 0; i < 50; i++) {
    commanderSetSetpoint(&cut, 3);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void rampToHeight(float zTarget, uint32_t rampMs) {
  const uint32_t dtMs = 20;
  const uint32_t steps = (rampMs / dtMs) ? (rampMs / dtMs) : 1;
  
  for (uint32_t i = 0; i <= steps; i++) {
    if (checkKillAndDisarm()) return;
    if (checkEmergencyLand()) return;
    const float z = (zTarget * (float)i) / (float)steps;
    sendHover(0.0f, 0.0f, z, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }
}

// ============================================================================
// Hover and measure radius
// ============================================================================
static void hoverAndMeasureRadius(uint32_t durationMs) {
  const uint32_t dtMs = 20;
  const uint32_t steps = durationMs / dtMs;
  const uint32_t sampleInterval = RADIUS_SAMPLE_INTERVAL_MS / dtMs;
  
  float distanceSum = 0.0f;
  uint32_t sampleCount = 0;
  
  for (uint32_t i = 0; i < steps; i++) {
    if (checkKillAndDisarm()) return;
    if (checkEmergencyLand()) return;
    
    sendHover(0.0f, 0.0f, targetHeightM, 0.0f);
    
    // Sample distance at intervals
    if ((i % sampleInterval) == 0) {
      uint32_t d0 = logGetUint(idDistance0);
      if (d0 > 0) {
        // Convert to meters
        float slantDistM = (float)d0 / 1000.0f;
        
        // UWB gives slant range (3D distance). Extract horizontal component.
        // Beacon is on ground (height=0), drone is at targetHeightM.
        // slantDist^2 = horizontalDist^2 + height^2
        // horizontalDist = sqrt(slantDist^2 - height^2)
        float slantDistSq = slantDistM * slantDistM;
        float heightSq = targetHeightM * targetHeightM;
        
        if (slantDistSq > heightSq) {
          float horizontalDistM = sqrtf(slantDistSq - heightSq);
          distanceSum += horizontalDistM;
          sampleCount++;
        }
        // else: slant distance is less than height (shouldn't happen normally)
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }
  
  // Calculate average radius
  if (sampleCount > 0) {
    measuredRadiusM = distanceSum / (float)sampleCount;
    
    // Clamp to valid range
    if (measuredRadiusM < MIN_RADIUS_M) measuredRadiusM = MIN_RADIUS_M;
    if (measuredRadiusM > MAX_RADIUS_M) measuredRadiusM = MAX_RADIUS_M;
    
    // Calculate yaw rate for this radius
    circleYawRateDps = calculateYawRate(circleSpeedMps, measuredRadiusM);
    
    DEBUG_PRINT("[%.2f] Measured horizontal radius: %.2f m (avg of %lu samples), yaw rate: %.1f deg/s\n",
                (double)getTimestamp(), (double)measuredRadiusM, 
                (unsigned long)sampleCount, (double)circleYawRateDps);
  } else {
    // Fallback to default if no samples
    measuredRadiusM = 1.5f;
    circleYawRateDps = calculateYawRate(circleSpeedMps, measuredRadiusM);
    DEBUG_PRINT("[%.2f] No distance samples, using default radius: %.2f m\n",
                (double)getTimestamp(), (double)measuredRadiusM);
  }
}

// ============================================================================
// State transition checks
// ============================================================================
static bool shouldEnterDance(void) {
  const uint32_t peerDist = getPeerDistance();
  
  // Don't enter dance if peer already landed
  if (isPeerLanded()) {
    peerCloseCount = 0;
    return false;
  }
  
  if (peerDist > 0 && peerDist < peerCloseMm) {
    if (++peerCloseCount >= confirmCount) {
      peerCloseCount = 0;
      return true;
    }
  } else {
    peerCloseCount = 0;
  }
  return false;
}

static bool shouldExitDance(void) {
  // The drone that's circling exits when peer has landed
  if (!myTurnToLand) {
    if (isPeerLanded()) {
      if (++peerLandedCount >= PEER_LANDED_CONFIRM_COUNT) {
        peerLandedCount = 0;
        return true;
      }
    } else {
      peerLandedCount = 0;
    }
  }
  
  // The drone that landed exits after taking off again
  if (myTurnToLand && danceHasTakenOff) {
    return true;
  }
  
  return false;
}

// ============================================================================
// State handlers
// ============================================================================
static void onEnterCircle(void) {
  DEBUG_PRINT("[%.2f] Enter CIRCLE (radius=%.2f m, yawRate=%.1f deg/s)\n", 
              (double)getTimestamp(), (double)measuredRadiusM, (double)circleYawRateDps);
  peerCloseCount = 0;
}

static void executeCircle(void) {
  // Fly forward while yawing CW to create circular motion
  sendHover(circleSpeedMps, 0.0f, targetHeightM, circleYawRateDps);
}

static void onEnterDance(void) {
  // Determine if it's my turn to land (alternates each dance)
  // danceCount even: drone 1 lands, danceCount odd: drone 2 lands
  if (droneId == 1) {
    myTurnToLand = (danceCount % 2 == 0);
  } else {
    myTurnToLand = (danceCount % 2 == 1);
  }
  
  danceCount++;
  
  DEBUG_PRINT("[%.2f] Enter DANCE #%u (drone %u, %s)\n", 
              (double)getTimestamp(), danceCount, droneId, 
              myTurnToLand ? "I land" : "I circle");
  
  peerLandedCount = 0;
  rejoinCount = 0;
  danceHasLanded = false;
  danceHasTakenOff = false;
}

static void executeDance(void) {
  if (myTurnToLand) {
    // This drone lands, waits for peer to be far enough, then takes off
    
    if (!danceHasLanded) {
      // Pause briefly then land
      DEBUG_PRINT("[%.2f] DANCE: drone %u pausing then landing\n", (double)getTimestamp(), droneId);
      sendHover(0.0f, 0.0f, targetHeightM, 0.0f);
      vTaskDelay(pdMS_TO_TICKS(500));  // Brief pause
      
      landToZero();
      danceHasLanded = true;
      DEBUG_PRINT("[%.2f] DANCE: drone %u landed\n", (double)getTimestamp(), droneId);
      
    } else if (!danceHasTakenOff) {
      // Wait for peer to be far enough
      // Send idle setpoint to prevent watchdog timeout
      setpoint_t idle;
      memset(&idle, 0, sizeof(idle));
      idle.mode.x = modeDisable;
      idle.mode.y = modeDisable;
      idle.mode.z = modeDisable;
      idle.mode.yaw = modeDisable;
      idle.thrust = 0;
      commanderSetSetpoint(&idle, 3);
      
      const uint32_t peerDist = getPeerDistance();
      
      if (peerDist >= peerSafeMm) {
        if (++rejoinCount >= REJOIN_CONFIRM_COUNT) {
          DEBUG_PRINT("[%.2f] DANCE: drone %u taking off (peer dist=%lu >= %u)\n",
                      (double)getTimestamp(), droneId, (unsigned long)peerDist, peerSafeMm);
          rampToHeight(targetHeightM, RAMP_TIME_MS);
          danceHasTakenOff = true;
          DEBUG_PRINT("[%.2f] DANCE: drone %u airborne\n", (double)getTimestamp(), droneId);
        }
      } else {
        rejoinCount = 0;
      }
    }
    
  } else {
    // This drone continues circling while waiting for peer to land
    executeCircle();
  }
}

static void onExitDance(void) {
  DEBUG_PRINT("[%.2f] Exit DANCE (drone %u)\n", (double)getTimestamp(), droneId);
  peerLandedCount = 0;
  rejoinCount = 0;
  danceHasLanded = false;
  danceHasTakenOff = false;
}

// ============================================================================
// Velocity Controller Gains Configuration
// ============================================================================
static void setVelocityControllerGains(void) {
  // X velocity gains
  paramVarId_t vxKpId = paramGetVarId("velCtlPid", "vxKp");
  paramVarId_t vxKiId = paramGetVarId("velCtlPid", "vxKi");
  paramVarId_t vxKdId = paramGetVarId("velCtlPid", "vxKd");
  paramVarId_t vxKFFId = paramGetVarId("velCtlPid", "vxKFF");

  // Y velocity gains
  paramVarId_t vyKpId = paramGetVarId("velCtlPid", "vyKp");
  paramVarId_t vyKiId = paramGetVarId("velCtlPid", "vyKi");
  paramVarId_t vyKdId = paramGetVarId("velCtlPid", "vyKd");
  paramVarId_t vyKFFId = paramGetVarId("velCtlPid", "vyKFF");

  // Z velocity gains
  paramVarId_t vzKpId = paramGetVarId("velCtlPid", "vzKp");
  paramVarId_t vzKiId = paramGetVarId("velCtlPid", "vzKi");
  paramVarId_t vzKdId = paramGetVarId("velCtlPid", "vzKd");
  paramVarId_t vzKFFId = paramGetVarId("velCtlPid", "vzKFF");

  // Set X velocity gains
  if (PARAM_VARID_IS_VALID(vxKFFId)) paramSetFloat(vxKFFId, 30.0f);
  if (PARAM_VARID_IS_VALID(vxKdId)) paramSetFloat(vxKdId, 0.0f);
  if (PARAM_VARID_IS_VALID(vxKiId)) paramSetFloat(vxKiId, 5.0f);
  if (PARAM_VARID_IS_VALID(vxKpId)) paramSetFloat(vxKpId, 20.0f);

  // Set Y velocity gains
  if (PARAM_VARID_IS_VALID(vyKFFId)) paramSetFloat(vyKFFId, 4.0f);
  if (PARAM_VARID_IS_VALID(vyKdId)) paramSetFloat(vyKdId, 0.0f);
  if (PARAM_VARID_IS_VALID(vyKiId)) paramSetFloat(vyKiId, 15.0f);
  if (PARAM_VARID_IS_VALID(vyKpId)) paramSetFloat(vyKpId, 30.0f);

  // Set Z velocity gains
  if (PARAM_VARID_IS_VALID(vzKFFId)) paramSetFloat(vzKFFId, 0.0f);
  if (PARAM_VARID_IS_VALID(vzKdId)) paramSetFloat(vzKdId, 0.0f);
  if (PARAM_VARID_IS_VALID(vzKiId)) paramSetFloat(vzKiId, 0.5f);
  if (PARAM_VARID_IS_VALID(vzKpId)) paramSetFloat(vzKpId, 20.0f);

  DEBUG_PRINT("[%.2f] Velocity controller gains set\n", (double)getTimestamp());
}

// ============================================================================
// Main flight sequence
// ============================================================================
static void runSequence(void) {
  seqAbort = false;
  outerBoundCount = 0;
  peerCloseCount = 0;
  peerFarCount = 0;
  peerLandedCount = 0;
  rejoinCount = 0;
  danceHasLanded = false;
  danceHasTakenOff = false;
  danceCount = 0;  // Reset dance counter for each sequence
  
  currentState = STATE_CIRCLE;

  uint32_t startTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

  if (checkKillAndDisarm()) return;

  // Takeoff
  rampToHeight(targetHeightM, RAMP_TIME_MS);
  if (seqAbort) { landToZero(); return; }

  // Hover and measure radius
  DEBUG_PRINT("[%.2f] Hovering for %u ms to measure radius\n", (double)getTimestamp(), HOVER_BEFORE_CIRCLE_MS);
  hoverAndMeasureRadius(HOVER_BEFORE_CIRCLE_MS);
  if (seqAbort) { landToZero(); return; }

  DEBUG_PRINT("[%.2f] Drone %u: measured radius=%.2f m, yawRate=%.1f, outer=%u, peer close=%u, peer safe=%u\n",
              (double)getTimestamp(), droneId, (double)measuredRadiusM, (double)circleYawRateDps,
              outerBoundMm, peerCloseMm, peerSafeMm);

  // Enter CIRCLE state
  onEnterCircle();

  const uint32_t dtMs = 10;

  while (!seqAbort) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if ((now - startTime) > demoTimeMs) {
      DEBUG_PRINT("[%.2f] Demo time elapsed\n", (double)getTimestamp());
      break;
    }

    if (checkKillAndDisarm()) break;

    // Emergency check (skip during landed phase of DANCE)
    if (!(currentState == STATE_DANCE && myTurnToLand && danceHasLanded && !danceHasTakenOff)) {
      if (checkEmergencyLand()) break;
    }

    // State machine
    switch (currentState) {
      case STATE_CIRCLE:
        if (shouldEnterDance()) {
          DEBUG_PRINT("[%.2f] CIRCLE -> DANCE: peer too close\n", (double)getTimestamp());
          onEnterDance();
          currentState = STATE_DANCE;
        } else {
          executeCircle();
        }
        break;
        
      case STATE_DANCE:
        if (shouldExitDance()) {
          DEBUG_PRINT("[%.2f] DANCE -> CIRCLE\n", (double)getTimestamp());
          onExitDance();
          onEnterCircle();
          currentState = STATE_CIRCLE;
        } else {
          executeDance();
        }
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }

  landToZero();
  if (seqAbort) {
    DEBUG_PRINT("[%.2f] Emergency landing complete\n", (double)getTimestamp());
  } else {
    DEBUG_PRINT("[%.2f] Demo complete\n", (double)getTimestamp());
  }
  commanderRelaxPriority();
}

// ============================================================================
// App entry point
// ============================================================================
void appMain(void) {
  droneId = (uint8_t)(configblockGetRadioAddress() & 0xF);
  DEBUG_PRINT("[%.2f] Mini Flapper Swarm App started, droneId=%u\n", (double)getTimestamp(), droneId);

  // Resolve common log IDs
  while (!logVarIdIsValid(idDistance0) || !logVarIdIsValid(idZ)) {
    ensureLogId(&idDistance0, "ranging", "distance0");
    ensureLogId(&idZ,         "stateEstimate", "z");
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Drone-specific IDs
  if (droneId == 1) {
    while (!logVarIdIsValid(idCppmAux0) || !logVarIdIsValid(idDistance2) || !logVarIdIsValid(idHeight2)) {
      ensureLogId(&idCppmAux0,   "cppm", "aux0");
      ensureLogId(&idDistance2,  "ranging", "distance2");
      ensureLogId(&idHeight2,    "ranging", "height2");
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    DEBUG_PRINT("[%.2f] Drone 1: RC trigger, watching drone 2\n", (double)getTimestamp());
  } else {
    while (!logVarIdIsValid(idRangingAux1) || !logVarIdIsValid(idRangingAux2) ||
           !logVarIdIsValid(idDistance1) || !logVarIdIsValid(idHeight1)) {
      ensureLogId(&idRangingAux1, "ranging", "aux1");
      ensureLogId(&idRangingAux2, "ranging", "aux2");
      ensureLogId(&idDistance1,   "ranging", "distance1");
      ensureLogId(&idHeight1,     "ranging", "height1");
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    DEBUG_PRINT("[%.2f] Drone %u: UWB trigger, watching drone 1\n", (double)getTimestamp(), droneId);
  }
      
  bool wasActive = false;

  if (droneId != 0) {
    while (1) {
      const bool active = isTriggerActive();
  
      if (checkEmergencyLand()) {
        landToZero();
      }
      
      // Start on trigger rising edge if beacon is visible
      if ((logGetUint(idDistance0) > 0) && active && !wasActive) {
        setVelocityControllerGains();
        runSequence();
      }
      
      wasActive = active;
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}

// ============================================================================
// Parameters
// ============================================================================
PARAM_GROUP_START(miniswarm)
  PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, height, &targetHeightM)
  PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, speed, &circleSpeedMps)
  PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, outerBound, &outerBoundMm)
  PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, peerClose, &peerCloseMm)
  PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, peerSafe, &peerSafeMm)
  PARAM_ADD(PARAM_UINT32 | PARAM_PERSISTENT, demoTime, &demoTimeMs)
  PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, confirm, &confirmCount)
PARAM_GROUP_STOP(miniswarm)

// ============================================================================
// Logs
// ============================================================================
LOG_GROUP_START(miniswarm)
  LOG_ADD(LOG_UINT8, state, &currentState)
  LOG_ADD(LOG_FLOAT, radius, &measuredRadiusM)
  LOG_ADD(LOG_FLOAT, yawRate, &circleYawRateDps)
  LOG_ADD(LOG_UINT8, danceNum, &danceCount)
LOG_GROUP_STOP(miniswarm)