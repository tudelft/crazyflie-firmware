/*
 * Unified Flapper Swarm App
 * 
 * This app supports multiple drones in a swarm with a single codebase.
 * The drone ID is derived from the radio address (last nibble of URI):
 *   - droneId = 1: Primary drone, triggered by RC remote (cppm.aux0), avoids using distance2, CW yaw
 *   - droneId = 2: Secondary drone, triggered via UWB (ranging.aux1), avoids using distance1, CCW yaw
 * 
 * Common behavior:
 *   - All drones monitor distance0 (beacon) for outer boundary emergency land
 *   - All drones perform the same flight pattern (STRAIGHT -> TURN -> RECOVER)
 *   - Kill switch via ranging.aux2 (for droneId >= 2)
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "FLAPPERSWARM"
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
static uint8_t droneId = 0;  // Will be set from radio address in appMain()

// ============================================================================
// Runtime-configurable flight parameters (can be changed from client)
// ============================================================================
static float targetHeightM = 1.0f;
static float fwdSpeedMps = 0.5f;
// Post-takeoff hover duration (ms) before the demo starts. Live-tunable and
// persistent via swarm.hoverMs. Default 2000 preserves the original behaviour.
static uint16_t takeoffHoverMs = 2000;
static uint16_t dist0AbortMm = 4200U;        // outer emergency bound to beacon (mm)
static uint16_t innerBoundMm = 1750U;        // Inner bound to start turning
static uint16_t innerBoundHysteresisMm = 100U; // Hysteresis: exit TURN when d0 <= innerBoundMm - hysteresis
static float turnYawRateDps = 40.0f;         // Yaw rate while turning (deg/s)
static uint16_t peerCloseMm = 2000U;         // near-limit on peer distance (2m)
static uint8_t abortConfirmCount = 2;        // Require N consecutive samples to trigger thresholds
static uint8_t turnExitConfirmCount = 4;     // Require more samples to exit TURN (reduce noise-triggered exits)
static uint8_t avoidEnterConfirmCount = 2;
static uint8_t avoidExitConfirmCount = 4;
static uint16_t avoidMinLandMm = 600U;       // Drone min land distance
static float avoidSpeedFactor = 1.0f;        // full speed during avoidance
static float avoidYawRateMagnitude = 70.0f;  // absolute yaw rate for avoidance (deg/s)
static uint32_t demoTimeMs = 60000U;         // time of the demo in ms

// Obstacle avoidance (VL53L8CX forward sensor)
static uint16_t obstTrigMm = 1200U;          // Middle-col avg trigger distance (mm)
static uint16_t obstBackDurMs = 500U;        // Backup phase duration (ms)
static float obstBackVx = -0.3f;             // Backup body-x velocity (m/s, negative = rearward)
static float obstRotDeg = 80.0f;             // Rotation angle after backup (degrees)
static float obstRotRate = 50.0f;            // Yaw rate during rotation (deg/s)
static uint8_t obstEnterConfirmCount = 2U;   // Confirmation samples to enter OBSTACLE

// ============================================================================
// Fixed parameters (not runtime configurable)
// ============================================================================
#define RAMP_TIME_MS 1500U
#define LAND_VZ_MPS 0.4f            // descent speed
#define CUT_Z_M 0.04f               // cut controllers below this altitude
#define DERIV_SAMPLE_INTERVAL_MS 10
#define D0_BUFFER_SIZE 15
#define DES_DERIV -1200.0f          // desired derivative wrt middle beacon
#define RECOVER_YAWRATE 50.0f       // yawrate at 0 derivative
#define RECOVER_DEADZONE 30.0f      // stop rotating within this target yawrate

// Drone 1 specific: RC trigger threshold (active low)
#define AUX_RC_ACTIVE_THRESH 1400
// Drone 2+ specific: UWB trigger threshold (active high)
#define AUX_UWB_ACTIVE_THRESHOLD 0U

// Height threshold below which we consider a drone "landed" (meters)
#define PEER_LANDED_HEIGHT_M 0.1f

// Dance state constants
#define PEER_LANDED_CONFIRM_COUNT 3
#define REJOIN_EXTRA_MM 50U
#define REJOIN_CONFIRM_COUNT 3

// OBSTACLE state constants
#define OBSTACLE_YAW_TOLERANCE 5.0f         // Degrees tolerance for obstacle rotation completion

// UTURN state constants
// Middle bound = dist0AbortMm - middleBoundOffsetMm. Runtime-tunable via
// cfclient (swarm.middleOffset). If set >= dist0Abort, UTURN is effectively
// disabled (see getMiddleBoundMm safety).
static uint16_t middleBoundOffsetMm = 400U;
#define UTURN_CONFIRM_COUNT 2               // Samples to confirm middle bound exceeded
#define UTURN_YAW_RATE_DPS 60.0f            // Yaw rate for 180° turn (deg/s)
#define UTURN_YAW_TOLERANCE 5.0f            // Degrees tolerance for completing turn
#define UTURN_COOLDOWN_MS 2000U             // Cooldown before another UTURN can be triggered

// ============================================================================
// Log variable IDs
// ============================================================================
// Common
static logVarId_t idDistance0   = (logVarId_t)0xFFFF;  // distance to beacon
static logVarId_t idZ           = (logVarId_t)0xFFFF;
static logVarId_t idYaw         = (logVarId_t)0xFFFF;

// Debug: optic flow and z-ranger
static logVarId_t idMotionDeltaX = (logVarId_t)0xFFFF;
static logVarId_t idMotionDeltaY = (logVarId_t)0xFFFF;
static logVarId_t idZrange       = (logVarId_t)0xFFFF;  // raw z-ranger reading (mm)

// Obstacle: VL53L8CX column averages (optional, requires zranger3 deck)
static logVarId_t idObstColL     = (logVarId_t)0xFFFF;  // left column avg (mm)
static logVarId_t idObstColML    = (logVarId_t)0xFFFF;  // mid-left column avg (mm)
static logVarId_t idObstColMR    = (logVarId_t)0xFFFF;  // mid-right column avg (mm)
static logVarId_t idObstColR     = (logVarId_t)0xFFFF;  // right column avg (mm)

// Drone 1 specific
static logVarId_t idCppmAux0    = (logVarId_t)0xFFFF;  // RC trigger

// Drone 2+ specific (triggered via UWB on aux1, killed via aux2)
static logVarId_t idRangingAux1 = (logVarId_t)0xFFFF;  // UWB trigger
static logVarId_t idRangingAux2 = (logVarId_t)0xFFFF;  // UWB kill switch

// Peer distance/height logs (resolved at startup for whichever peers != droneId).
// Up to 3 drones; each drone reads the distance/height to the OTHER two.
static logVarId_t idDistance1   = (logVarId_t)0xFFFF;  // distance to drone 1
static logVarId_t idHeight1     = (logVarId_t)0xFFFF;  // height of drone 1
static logVarId_t idDistance2   = (logVarId_t)0xFFFF;  // distance to drone 2
static logVarId_t idHeight2     = (logVarId_t)0xFFFF;  // height of drone 2
static logVarId_t idDistance3   = (logVarId_t)0xFFFF;  // distance to drone 3
static logVarId_t idHeight3     = (logVarId_t)0xFFFF;  // height of drone 3

// ============================================================================
// State machine
// ============================================================================
typedef enum {
  STATE_STRAIGHT  = 0,
  STATE_TURN      = 1,
  STATE_AVOID     = 2,
  STATE_RECOVER   = 3,
  STATE_DANCE     = 4,
  STATE_UTURN     = 5,   // Emergency 180° turn near outer bound (one-shot)
  STATE_UTURN_FLY = 6,   // Flying straight after UTURN, trying to reach inner bound
  STATE_OBSTACLE  = 7    // Forward obstacle detected: back up then rotate to clear side
} FlightState;

static uint8_t currentState = STATE_STRAIGHT;  // Exposed for logging
static FlightState prevState = STATE_STRAIGHT; // Track previous state for transitions

// ============================================================================
// State variables
// ============================================================================
static uint8_t abortOverCount   = 0;
static uint8_t innerOverCount   = 0;
static uint8_t innerUnderCount  = 0;

// Avoidance confirmation counters
static uint8_t approachCount = 0;
static uint8_t departCount = 0;

// Dance state counters
static uint8_t rejoinCount = 0;

// Dance state tracking
static bool danceHasLanded = false;   // any drone: tracks if we've completed landing
static bool danceHasTakenOff = false; // any drone: tracks if we've completed takeoff

// Dance role: chosen on entry so two drones don't land simultaneously and drift
// into each other on the way down. PRIMARY descends; SECONDARY hovers until the
// PRIMARY has touched down, then promotes itself to PRIMARY and lands too.
typedef enum {
  DANCE_ROLE_NONE      = 0,
  DANCE_ROLE_SANDWICH  = 1,  // 3-drone case: I'm the middle, I land
  DANCE_ROLE_PRIMARY   = 2,  // 2-drone safety: tie-breaker says I land first
  DANCE_ROLE_SECONDARY = 3,  // 2-drone safety: I hover, land after PRIMARY is down
} DanceRole;
static DanceRole danceRole = DANCE_ROLE_NONE;
static uint8_t   danceCriticalPeer = 0; // for SECONDARY: which peer is landing first

// UTURN state variables
static uint8_t middleBoundOverCount = 0;    // Confirmation counter for middle bound
static float uturnYawStart = 0.0f;          // Yaw at start of UTURN
static float uturnTargetYaw = 0.0f;         // Target yaw (180° from start)
static bool uturnActive = false;            // Whether we're actively tracking the 180° turn
static uint32_t uturnCooldownEndTime = 0;   // Timestamp when UTURN cooldown ends

// OBSTACLE state variables
static int8_t  obstFreeDir = 1;             // +1 = positive yaw, -1 = negative yaw
static uint8_t obstPhase = 0;               // 0 = backup, 1 = rotate
static uint32_t obstPhaseStartMs = 0;       // Tick timestamp when current phase started
static float obstRotYawStart = 0.0f;        // Yaw at start of rotation phase
static float obstRotTargetYaw = 0.0f;       // Target yaw for rotation
static bool obstComplete = false;           // True when both phases complete
static uint8_t obstEnterCount = 0;          // Confirmation counter for OBSTACLE entry

// Sequence abort flag set by emergency check
static volatile bool seqAbort = false;

// Recover variables
static const float recoverFactor = - ( RECOVER_YAWRATE / DES_DERIV);

// ============================================================================
// Derivative buffer for d0
// ============================================================================
#define PEER_BUFFER_SIZE 15

typedef struct {
  uint32_t samples[D0_BUFFER_SIZE];
  uint8_t writeIndex;
  uint8_t count;
  uint32_t lastSampleTime;
} D0Buffer;

static D0Buffer d0Buffer;

// ============================================================================
// Derivative buffer for peer distance (used in AVOID state)
// ============================================================================
typedef struct {
  uint32_t samples[PEER_BUFFER_SIZE];
  uint8_t writeIndex;
  uint8_t count;
  uint32_t lastSampleTime;
} PeerDistBuffer;

static PeerDistBuffer peerDistBuffer;

static void d0BufferReset(void) {
  memset(&d0Buffer, 0, sizeof(d0Buffer));
}

static void d0BufferAdd(uint32_t d0, uint32_t now) {
  if ((now - d0Buffer.lastSampleTime) >= DERIV_SAMPLE_INTERVAL_MS) {
    d0Buffer.samples[d0Buffer.writeIndex] = d0;
    d0Buffer.writeIndex = (d0Buffer.writeIndex + 1) % D0_BUFFER_SIZE;
    if (d0Buffer.count < D0_BUFFER_SIZE) {
      d0Buffer.count++;
    }
    d0Buffer.lastSampleTime = now;
  }
}

// Compute derivative using linear regression (least squares fit)
// Returns derivative in mm/s, positive = moving away, negative = moving toward
static float d0BufferGetDerivative(void) {
  if (d0Buffer.count < 2) {
    return 0.0f;
  }
  
  uint8_t n = d0Buffer.count;
  
  float sumT = 0.0f;
  float sumY = 0.0f;
  float sumTY = 0.0f;
  float sumT2 = 0.0f;
  
  for (uint8_t i = 0; i < n; i++) {
    uint8_t idx;
    if (d0Buffer.count < D0_BUFFER_SIZE) {
      idx = i;
    } else {
      idx = (d0Buffer.writeIndex + i) % D0_BUFFER_SIZE;
    }
    
    float t = (float)i;
    float y = (float)d0Buffer.samples[idx];
    
    sumT += t;
    sumY += y;
    sumTY += t * y;
    sumT2 += t * t;
  }
  
  float denom = (float)n * sumT2 - sumT * sumT;
  if (fabsf(denom) < 1e-6f) {
    return 0.0f;
  }
  
  float slopePerInterval = ((float)n * sumTY - sumT * sumY) / denom;
  float intervalS = (float)DERIV_SAMPLE_INTERVAL_MS / 1000.0f;
  
  return slopePerInterval / intervalS;
}

// ============================================================================
// Peer distance buffer functions (for AVOID state derivative)
// ============================================================================
static void peerDistBufferReset(void) {
  memset(&peerDistBuffer, 0, sizeof(peerDistBuffer));
}

static void peerDistBufferAdd(uint32_t peerDist, uint32_t now) {
  if ((now - peerDistBuffer.lastSampleTime) >= DERIV_SAMPLE_INTERVAL_MS) {
    peerDistBuffer.samples[peerDistBuffer.writeIndex] = peerDist;
    peerDistBuffer.writeIndex = (peerDistBuffer.writeIndex + 1) % PEER_BUFFER_SIZE;
    if (peerDistBuffer.count < PEER_BUFFER_SIZE) {
      peerDistBuffer.count++;
    }
    peerDistBuffer.lastSampleTime = now;
  }
}

// Compute derivative using linear regression (least squares fit)
// Returns derivative in mm/s, positive = moving apart, negative = moving closer
static float peerDistBufferGetDerivative(void) {
  if (peerDistBuffer.count < 2) {
    return 0.0f;
  }
  
  uint8_t n = peerDistBuffer.count;
  
  float sumT = 0.0f;
  float sumY = 0.0f;
  float sumTY = 0.0f;
  float sumT2 = 0.0f;
  
  for (uint8_t i = 0; i < n; i++) {
    uint8_t idx;
    if (peerDistBuffer.count < PEER_BUFFER_SIZE) {
      idx = i;
    } else {
      idx = (peerDistBuffer.writeIndex + i) % PEER_BUFFER_SIZE;
    }
    
    float t = (float)i;
    float y = (float)peerDistBuffer.samples[idx];
    
    sumT += t;
    sumY += y;
    sumTY += t * y;
    sumT2 += t * t;
  }
  
  float denom = (float)n * sumT2 - sumT * sumT;
  if (fabsf(denom) < 1e-6f) {
    return 0.0f;
  }
  
  float slopePerInterval = ((float)n * sumTY - sumT * sumY) / denom;
  float intervalS = (float)DERIV_SAMPLE_INTERVAL_MS / 1000.0f;
  
  return slopePerInterval / intervalS;
}

// ============================================================================
// State machine context (shared between state handlers)
// ============================================================================
typedef struct {
  // Arc tracking (TURN state)
  bool arcActive;
  bool arcCooldown;
  float arcYawStart;
  float targetYaw;
  int rotationDirection;
  
  // Counters
  uint8_t routines;
  
  // Current sensor readings (updated each loop)
  uint32_t d0;
  float d0Deriv;
  uint32_t peerDist;
  float peerDistDeriv;
} StateContext;

static StateContext ctx;

// ============================================================================
// Utility functions
// ============================================================================
static inline void ensureLogId(logVarId_t* id, const char* group, const char* name) {
  if (!logVarIdIsValid(*id)) {
    *id = logGetVarId(group, name);
  }
}

static inline float normalizeAngle(float a) {
  while (a <= -180) a += 360;
  while (a > 180) a -= 360;
  return a;
}


// ============================================================================
// Drone-specific behavior (runtime selection based on droneId)
// ============================================================================

// Check if the trigger is active (start signal)
static inline bool isTriggerActive(void) {
  if (droneId == 1) {
    // Drone 1: RC trigger via cppm.aux0 (active low)
    const int16_t v = logGetInt(idCppmAux0);
    return (v > 0) && (v < AUX_RC_ACTIVE_THRESH);
  } else {
    // Drone 2+: UWB trigger via ranging.aux1 (active high)
    const uint32_t v = logGetUint(idRangingAux1);
    return v > AUX_UWB_ACTIVE_THRESHOLD;
  }
}

// Check if the kill switch is active (drone 2+ only)
static inline bool isKillActive(void) {
  if (droneId == 1) {
    // Drone 1 has no kill switch
    return false;
  } else {
    // Drone 2+: kill via ranging.aux2 (active high)
    const uint32_t v = logGetUint(idRangingAux2);
    return v > AUX_UWB_ACTIVE_THRESHOLD;
  }
}

// --- Multi-peer accessors (up to 3 drones) ---
// Read distance/height log for a specific peer ID (1, 2, or 3).
static inline uint32_t peerDistanceById(uint8_t peerId) {
  logVarId_t id = (peerId == 1) ? idDistance1
                : (peerId == 2) ? idDistance2
                : (peerId == 3) ? idDistance3
                : (logVarId_t)0xFFFF;
  return logVarIdIsValid(id) ? logGetUint(id) : 0U;
}

static inline float peerHeightById(uint8_t peerId) {
  logVarId_t id = (peerId == 1) ? idHeight1
                : (peerId == 2) ? idHeight2
                : (peerId == 3) ? idHeight3
                : (logVarId_t)0xFFFF;
  return logVarIdIsValid(id) ? logGetFloat(id) : -1.0f;
}

static inline bool isPeerLandedById(uint8_t peerId) {
  float h = peerHeightById(peerId);
  return (h >= 0.0f && h < PEER_LANDED_HEIGHT_M);
}

// Return the minimum distance to any non-landed peer (0 if no valid flying peer).
static uint32_t getPeerDistance(void) {
  uint32_t minD = 0U;
  for (uint8_t i = 1; i <= 3; i++) {
    if (i == droneId) continue;
    if (isPeerLandedById(i)) continue;
    uint32_t d = peerDistanceById(i);
    if (d == 0U) continue;
    if (minD == 0U || d < minD) minD = d;
  }
  return minD;
}

// Count non-landed peers whose distance is <= threshold.
static uint8_t countFlyingPeersWithin(uint32_t threshold) {
  uint8_t count = 0;
  for (uint8_t i = 1; i <= 3; i++) {
    if (i == droneId) continue;
    if (isPeerLandedById(i)) continue;
    uint32_t d = peerDistanceById(i);
    if (d > 0U && d <= threshold) count++;
  }
  return count;
}

// True if every non-landed peer is at least `threshold` away
// (used for take-off rejoin: don't lift off while any peer is still near).
static bool areAllFlyingPeersFar(uint32_t threshold) {
  for (uint8_t i = 1; i <= 3; i++) {
    if (i == droneId) continue;
    if (isPeerLandedById(i)) continue;
    uint32_t d = peerDistanceById(i);
    if (d == 0U || d < threshold) return false;
  }
  return true;
}

// Legacy "is the (any) peer landed?" check, retained for the DANCE-takeoff
// branch where we just want to know "is at least one peer down".
static inline bool isPeerLanded(void) {
  for (uint8_t i = 1; i <= 3; i++) {
    if (i == droneId) continue;
    if (isPeerLandedById(i)) return true;
  }
  return false;
}

// Get the avoidance yaw rate (sign depends on drone)
static inline float getAvoidYawRate(void) {
  if (droneId == 1) {
    return avoidYawRateMagnitude;   // CW (positive)
  } else {
    return -avoidYawRateMagnitude;  // CCW (negative)
  }
}

// Check kill switch and disarm if active
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
// Emergency and avoidance logic
// ============================================================================
static bool checkAndMaybeEmergencyLand(void) {
  bool trigger = false;

  const uint32_t d0 = logGetUint(idDistance0);
  if (d0 > dist0AbortMm) {
    if (abortOverCount < 0xFF) abortOverCount++;
    if (abortOverCount >= abortConfirmCount) {
      if (!seqAbort) {
        DEBUG_PRINT("[%.2f] Emergency FAR: distance0=%lu mm (> %u)\n",
                    (double)getTimestamp(), (unsigned long)d0, dist0AbortMm);
      }
      seqAbort = true;
      trigger = true;
    }
  } else {
    abortOverCount = 0;
  }

  return trigger;
}

// Check if any flying peer is too close and we should enter AVOID.
// (getPeerDistance() returns the min over non-landed peers, or 0 if none.)
static bool shouldEnterAvoid(void) {
  const uint32_t peerDist = getPeerDistance();

  if (peerDist == 0U) {
    // No flying peer to avoid (all peers landed or out of range).
    approachCount = 0;
    return false;
  }

  if (peerDist <= peerCloseMm) {
    if (++approachCount >= avoidEnterConfirmCount) {
      approachCount = 0;
      departCount = 0;
      return true;
    }
  } else {
    approachCount = 0;
  }
  return false;
}

// Check if all flying peers are far enough and we should exit AVOID.
static bool shouldExitAvoid(void) {
  const uint32_t peerDist = getPeerDistance();

  // No flying peer → nothing to avoid, exit immediately.
  if (peerDist == 0U) {
    departCount = 0;
    approachCount = 0;
    return true;
  }

  if (peerDist >= peerCloseMm) {
    if (++departCount >= avoidExitConfirmCount) {
      departCount = 0;
      approachCount = 0;
      return true;
    }
  } else {
    departCount = 0;
  }
  return false;
}

// Compute the DANCE role for the *current* moment (does not mutate state).
// Returns DANCE_ROLE_NONE if there's no reason to enter / stay in DANCE.
// On non-NONE return, also writes the partner peer's id into *outCriticalPeer.
//
// Tie-breaker for "who lands first" is lower droneId — matches the original
// drone-1-lands-first behavior, generalized to 3 drones.
//
//   SANDWICH:  >= 2 flying peers in peerCloseMm AND I have the lowest droneId
//              among us → I land first.
//   SECONDARY: >= 2 flying peers close but I'm NOT the lowest → hover until
//              the lowest-id close peer is down, then land too.
//              (Also: safety-override case where my droneId > critical peer.)
//   PRIMARY:   exactly one peer within avoidMinLandMm and my droneId < peer's
//              → I land first.
static DanceRole computeDanceRole(uint8_t *outCriticalPeer) {
  if (currentState != STATE_AVOID && currentState != STATE_DANCE) {
    return DANCE_ROLE_NONE;
  }

  // Find close flying peers and the danger-zone peer (if any).
  uint8_t closeCount = 0;
  uint8_t lowestClose = 0xFF;
  uint8_t safetyPeer = 0;
  for (uint8_t i = 1; i <= 3; i++) {
    if (i == droneId) continue;
    if (isPeerLandedById(i)) continue;
    uint32_t d = peerDistanceById(i);
    if (d == 0U) continue;
    if (d <= peerCloseMm) {
      closeCount++;
      if (i < lowestClose) lowestClose = i;
    }
    if (d <= avoidMinLandMm && safetyPeer == 0) {
      safetyPeer = i;
    }
  }

  // SANDWICH: at least 2 flying peers in close range.
  if (closeCount >= 2) {
    if (droneId < lowestClose) {
      // I have the lowest droneId among the close cluster → I'm the lander.
      if (outCriticalPeer) *outCriticalPeer = lowestClose;
      return DANCE_ROLE_SANDWICH;
    }
    // Someone with a lower droneId is in the cluster — wait for them to land.
    if (outCriticalPeer) *outCriticalPeer = lowestClose;
    return DANCE_ROLE_SECONDARY;
  }

  // SAFETY: a single peer in the danger zone.
  if (safetyPeer == 0) return DANCE_ROLE_NONE;
  if (outCriticalPeer) *outCriticalPeer = safetyPeer;
  return (droneId < safetyPeer) ? DANCE_ROLE_PRIMARY : DANCE_ROLE_SECONDARY;
}

static bool shouldEnterDance(void) {
  return computeDanceRole(NULL) != DANCE_ROLE_NONE;
}

// ============================================================================
// Obstacle helper
// ============================================================================

// Returns true when the middle two column averages are both below obstTrigMm
// (with confirmation). Requires the zranger3 deck to be present.
static bool shouldEnterObstacle(void) {
  if (!logVarIdIsValid(idObstColML) || !logVarIdIsValid(idObstColMR)) {
    obstEnterCount = 0;
    return false;
  }
  int32_t midL = logGetInt(idObstColML);
  int32_t midR = logGetInt(idObstColMR);
  float midAvg = (float)(midL + midR) / 2.0f;
  if (midAvg > 0.0f && midAvg < (float)obstTrigMm) {
    if (++obstEnterCount >= obstEnterConfirmCount) {
      obstEnterCount = 0;
      return true;
    }
  } else {
    obstEnterCount = 0;
  }
  return false;
}

// ============================================================================
// UTURN helper functions
// ============================================================================
static inline uint16_t getMiddleBoundMm(void) {
  // Guard against the user setting offset >= dist0Abort: if invalid, collapse
  // the middle ring onto the outer ring so the UTURN check (d0 >= middle &&
  // d0 < outer) can never fire by accident.
  return (middleBoundOffsetMm < dist0AbortMm)
       ? (uint16_t)(dist0AbortMm - middleBoundOffsetMm)
       : dist0AbortMm;
}

static bool shouldEnterUturn(void) {
  // Check cooldown - can't enter UTURN until cooldown expires
  uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
  if (now < uturnCooldownEndTime) {
    return false;
  }
  
  // Check if we've exceeded the middle bound
  const uint16_t middleBound = getMiddleBoundMm();
  if (ctx.d0 >= middleBound && ctx.d0 < dist0AbortMm) {
    if (++middleBoundOverCount >= UTURN_CONFIRM_COUNT) {
      return true;
    }
  } else {
    middleBoundOverCount = 0;
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

  // Cut controllers and thrust
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
    if (checkAndMaybeEmergencyLand()) return;
    const float z = (zTarget * (float)i) / (float)steps;
    sendHover(0.0f, 0.0f, z, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }
  // Hover for a bit before starting the demo (swarm.hoverMs, live-tunable).
  const uint32_t hoverSteps = (takeoffHoverMs / dtMs) ? (takeoffHoverMs / dtMs) : 1;
  for (uint32_t i = 0; i <= hoverSteps; i++) {
    if (checkKillAndDisarm()) return;
    if (checkAndMaybeEmergencyLand()) return;
    sendHover(0.0f, 0.0f, zTarget, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }
}

// ============================================================================
// State handlers: onEnter, onExit, execute, checkTransition
// ============================================================================

// --- STRAIGHT state ---
static void onEnterStraight(void) {
  innerOverCount = 0;
  middleBoundOverCount = 0;  // Reset UTURN confirmation counter
  DEBUG_PRINT("[%.2f] Enter STRAIGHT\n", (double)getTimestamp());
  
  // If we just came from DANCE state (drone 1 after takeoff), log it
  if (prevState == STATE_DANCE && droneId == 1) {
    DEBUG_PRINT("[%.2f] STRAIGHT: rejoined swarm after DANCE\n", (double)getTimestamp());
  }
  
  // If we came from UTURN_FLY, we successfully made it back
  if (prevState == STATE_UTURN_FLY) {
    DEBUG_PRINT("[%.2f] STRAIGHT: UTURN recovery successful!\n", (double)getTimestamp());
  }
}

static void onExitStraight(void) {
  // Nothing special to clean up
}

static FlightState checkTransitionStraight(void) {
  // STRAIGHT -> AVOID: peer too close
  if (shouldEnterAvoid()) {
    DEBUG_PRINT("[%.2f] STRAIGHT -> AVOID: peer too close\n", (double)getTimestamp());
    return STATE_AVOID;
  }

  // STRAIGHT -> OBSTACLE: forward obstacle detected
  if (shouldEnterObstacle()) {
    DEBUG_PRINT("[%.2f] STRAIGHT -> OBSTACLE: middle cols avg < %u mm\n",
                (double)getTimestamp(), obstTrigMm);
    return STATE_OBSTACLE;
  }

  // STRAIGHT -> UTURN: approaching outer bound (one-shot emergency turn)
  if (shouldEnterUturn()) {
    DEBUG_PRINT("[%.2f] STRAIGHT -> UTURN: middle bound exceeded (d0=%lu >= %u)\n",
                (double)getTimestamp(), (unsigned long)ctx.d0, getMiddleBoundMm());
    return STATE_UTURN;
  }
  
  // STRAIGHT -> TURN: reached outer bound (only if not in arc cooldown)
  if (ctx.d0 >= innerBoundMm && !ctx.arcCooldown) {
    if (++innerOverCount >= abortConfirmCount) {
      DEBUG_PRINT("[%.2f] STRAIGHT -> TURN (d0=%lu)\n", (double)getTimestamp(), (unsigned long)ctx.d0);
      return STATE_TURN;
    }
  } else if (ctx.d0 < innerBoundMm) {
    innerOverCount = 0;
  }
  
  // STRAIGHT -> RECOVER: outside inner bound and moving away from beacon
  if (ctx.d0 >= innerBoundMm && ctx.d0Deriv > 0 && ctx.arcCooldown) {
    DEBUG_PRINT("[%.2f] STRAIGHT -> RECOVER: outside bound and deriv=%.2f (moving away)\n", (double)getTimestamp(), (double)ctx.d0Deriv);
    return STATE_RECOVER;
  }
  
  return STATE_STRAIGHT;  // No transition
}

static void executeStraight(void) {
  sendHover(fwdSpeedMps, 0.0f, targetHeightM, 0.0f);
}

// --- TURN state ---
static void onEnterTurn(void) {
  innerUnderCount = 0;
  ctx.arcYawStart = logGetFloat(idYaw);
  ctx.targetYaw = normalizeAngle(ctx.arcYawStart - 130.0f);
  ctx.arcActive = isfinite(ctx.arcYawStart);
  DEBUG_PRINT("[%.2f] Enter TURN: arcActive=%d, startYaw=%.3f deg, targetYaw=%.3f deg\n",
              (double)getTimestamp(), ctx.arcActive ? 1 : 0, (double)ctx.arcYawStart, (double)ctx.targetYaw);
}

static void onExitTurn(void) {
  ctx.arcActive = false;
}

static FlightState checkTransitionTurn(void) {
  // TURN -> AVOID: peer too close (one way, cannot return to TURN)
  if (shouldEnterAvoid()) {
    DEBUG_PRINT("[%.2f] TURN -> AVOID: peer too close\n", (double)getTimestamp());
    return STATE_AVOID;
  }

  // TURN -> OBSTACLE: forward obstacle detected
  if (shouldEnterObstacle()) {
    DEBUG_PRINT("[%.2f] TURN -> OBSTACLE: middle cols avg < %u mm\n",
                (double)getTimestamp(), obstTrigMm);
    return STATE_OBSTACLE;
  }

  // TURN -> UTURN: approaching outer bound (one-shot emergency turn)
  if (shouldEnterUturn()) {
    DEBUG_PRINT("[%.2f] STRAIGHT -> UTURN: middle bound exceeded (d0=%lu >= %u)\n",
                (double)getTimestamp(), (unsigned long)ctx.d0, getMiddleBoundMm());
    return STATE_UTURN;
  }
  
  // TURN -> STRAIGHT: back inside inner bound
  if (ctx.d0 <= innerBoundMm) {
    if (++innerUnderCount >= abortConfirmCount) {
      ctx.routines++;
      DEBUG_PRINT("[%.2f] TURN -> STRAIGHT: routine %u complete (d0=%lu, deriv=%.1f)\n", 
                  (double)getTimestamp(), ctx.routines, (unsigned long)ctx.d0, (double)ctx.d0Deriv);
      return STATE_STRAIGHT;
    }
  } else {
    innerUnderCount = 0;
  }
  
  // Arc tracking: when 270° reached, decide STRAIGHT or RECOVER
  if (ctx.arcActive) {
    float curYaw = logGetFloat(idYaw);
    if (isfinite(curYaw)) {
      const bool reached = (fabsf(curYaw - ctx.targetYaw) <= 3.0f) ||
                           (fabsf(curYaw - ctx.targetYaw - 360.0f) <= 3.0f);
      if (reached) {
        ctx.arcActive = false;
        ctx.arcCooldown = true;
        if (ctx.d0Deriv <= 0) {
          DEBUG_PRINT("[%.2f] TURN -> STRAIGHT: arc complete, deriv=%.2f (approaching)\n", (double)getTimestamp(), (double)ctx.d0Deriv);
          return STATE_STRAIGHT;
        } else {
          DEBUG_PRINT("[%.2f] TURN -> RECOVER: arc complete, deriv=%.2f (moving away)\n", (double)getTimestamp(), (double)ctx.d0Deriv);
          return STATE_RECOVER;
        }
      }
    } else {
      ctx.arcActive = false;
      DEBUG_PRINT("[%.2f] TURN arc: yaw unavailable, stopping arc tracking\n", (double)getTimestamp());
    }
  }
  
  return STATE_TURN;  // No transition
}

static void executeTurn(void) {
  float yawRateCmd = turnYawRateDps;
  if (ctx.d0Deriv < -50.0f) {
    yawRateCmd *= 0.5f;
  }
  DEBUG_PRINT("[%.2f] TURN: d0Deriv=%.2f, yawRateCmd=%.2f\n", (double)getTimestamp(), (double)ctx.d0Deriv, (double)yawRateCmd);
  sendHover(fwdSpeedMps, 0.0f*fwdSpeedMps, targetHeightM, yawRateCmd);
}

// --- AVOID state ---
static void onEnterAvoid(void) {
  DEBUG_PRINT("[%.2f] Enter AVOID\n", (double)getTimestamp());
}

static void onExitAvoid(void) {
  approachCount = 0;
  departCount = 0;
}

static FlightState checkTransitionAvoid(void) {
  // AVOID -> DANCE: peer dangerously close (emergency zone)
  if (shouldEnterDance()) {
    DEBUG_PRINT("[%.2f] AVOID -> DANCE: peer in danger zone (dist=%lu <= %u)\n",
                (double)getTimestamp(), (unsigned long)getPeerDistance(), avoidMinLandMm);
    return STATE_DANCE;
  }

  // AVOID -> OBSTACLE: forward obstacle detected
  if (shouldEnterObstacle()) {
    DEBUG_PRINT("[%.2f] AVOID -> OBSTACLE: middle cols avg < %u mm\n",
                (double)getTimestamp(), obstTrigMm);
    return STATE_OBSTACLE;
  }

  // AVOID -> UTURN: approaching outer bound (emergency turn)
  if (shouldEnterUturn()) {
    DEBUG_PRINT("[%.2f] AVOID -> UTURN: middle bound exceeded (d0=%lu >= %u)\n",
                (double)getTimestamp(), (unsigned long)ctx.d0, getMiddleBoundMm());
    return STATE_UTURN;
  }
  
  // AVOID -> STRAIGHT or RECOVER: peer far enough
  if (shouldExitAvoid()) {
    if (ctx.d0 < innerBoundMm) {
      DEBUG_PRINT("[%.2f] AVOID -> STRAIGHT: peer far, inside bound (d0=%lu)\n", (double)getTimestamp(), (unsigned long)ctx.d0);
      return STATE_STRAIGHT;
    } else {
      DEBUG_PRINT("[%.2f] AVOID -> RECOVER: peer far, outside bound (d0=%lu)\n", (double)getTimestamp(), (unsigned long)ctx.d0);
      return STATE_RECOVER;
    }
  }
  
  return STATE_AVOID;  // No transition
}

static void executeAvoid(void) {
  // If derivative is positive (drones moving apart), use half yaw rate
  // to prevent excessive circling when drones are already separating
  float yawRate = getAvoidYawRate();
  if (ctx.peerDistDeriv > 50.0f) {
    yawRate *= 0.5f;
  }
  sendHover(fwdSpeedMps * avoidSpeedFactor, 0.0f, targetHeightM, yawRate);
}

// --- RECOVER state ---
static uint8_t recoverDebugCounter = 0;  // For throttled debug output

static void onEnterRecover(void) {
  recoverDebugCounter = 0;  // Reset debug counter on entry
  middleBoundOverCount = 0;  // Reset UTURN confirmation counter
  if (droneId != 1) {
    if ((prevState == STATE_AVOID) && (ctx.d0 > dist0AbortMm - 800)) { // subject to tuning
      DEBUG_PRINT("[%.2f] Keeping direction after avoid\n", (double)getTimestamp());
      ctx.rotationDirection = -1;
    }
  }
  DEBUG_PRINT("[%.2f] Enter RECOVER: d0=%lu, d0Deriv=%.1f, rotDir=%d\n",
              (double)getTimestamp(), (unsigned long)ctx.d0, (double)ctx.d0Deriv, ctx.rotationDirection);
}

static void onExitRecover(void) {
  DEBUG_PRINT("[%.2f] Exit RECOVER: d0=%lu, d0Deriv=%.1f\n",
              (double)getTimestamp(), (unsigned long)ctx.d0, (double)ctx.d0Deriv);
  ctx.rotationDirection = 1;
}

static FlightState checkTransitionRecover(void) {
  // RECOVER -> AVOID: peer too close
  if (shouldEnterAvoid()) {
    DEBUG_PRINT("[%.2f] RECOVER -> AVOID: peer too close\n", (double)getTimestamp());
    return STATE_AVOID;
  }

  // RECOVER -> OBSTACLE: forward obstacle detected
  if (shouldEnterObstacle()) {
    DEBUG_PRINT("[%.2f] RECOVER -> OBSTACLE: middle cols avg < %u mm\n",
                (double)getTimestamp(), obstTrigMm);
    return STATE_OBSTACLE;
  }

  // RECOVER -> UTURN: approaching outer bound (one-shot emergency turn)
  if (shouldEnterUturn()) {
    DEBUG_PRINT("[%.2f] RECOVER -> UTURN: middle bound exceeded (d0=%lu >= %u)\n",
                (double)getTimestamp(), (unsigned long)ctx.d0, getMiddleBoundMm());
    return STATE_UTURN;
  }
  
  // RECOVER -> STRAIGHT: back inside inner bound
  if (ctx.d0 > 0 && ctx.d0 <= innerBoundMm) {
    DEBUG_PRINT("[%.2f] RECOVER -> STRAIGHT: success! d0=%lu <= %u, final deriv=%.1f\n",
                (double)getTimestamp(), (unsigned long)ctx.d0, innerBoundMm, (double)ctx.d0Deriv);
    return STATE_STRAIGHT;
  }
  
  return STATE_RECOVER;  // No transition
}

static void executeRecover(void) {
  float yawCommand = recoverFactor * fabsf(ctx.d0Deriv - DES_DERIV);
  float yawCommandRaw = yawCommand;  // Store pre-deadzone value for debugging
  yawCommand = yawCommand > RECOVER_DEADZONE ? yawCommand : 0.0f;
  yawCommand = fminf(yawCommand, 70.0f);
  float yawCommandFinal = yawCommand * ctx.rotationDirection;
  
  // Throttled debug output
  if (++recoverDebugCounter >= 25) {
    recoverDebugCounter = 0;
    DEBUG_PRINT("[%.2f] RECOVER: d0=%lu innerB=%u | deriv=%.1f desDeriv=%.1f | yawRaw=%.1f yawFinal=%.1f rotDir=%d\n",
                (double)getTimestamp(), (unsigned long)ctx.d0, innerBoundMm,
                (double)ctx.d0Deriv, (double)DES_DERIV,
                (double)yawCommandRaw, (double)yawCommandFinal,
                ctx.rotationDirection);
  }
  
  sendHover(fwdSpeedMps, 0.0f, targetHeightM, yawCommandFinal);
}

// --- DANCE state ---
static void onEnterDance(void) {
  danceCriticalPeer = 0;
  danceRole = computeDanceRole(&danceCriticalPeer);
  const char *roleName =
      (danceRole == DANCE_ROLE_SANDWICH)  ? "SANDWICH" :
      (danceRole == DANCE_ROLE_PRIMARY)   ? "PRIMARY"  :
      (danceRole == DANCE_ROLE_SECONDARY) ? "SECONDARY": "NONE";
  DEBUG_PRINT("[%.2f] Enter DANCE (drone %u, role=%s, peer=%u)\n",
              (double)getTimestamp(), droneId, roleName, danceCriticalPeer);
  rejoinCount = 0;
  danceHasLanded = false;
  danceHasTakenOff = false;
}

static void onExitDance(void) {
  DEBUG_PRINT("[%.2f] Exit DANCE (drone %u)\n", (double)getTimestamp(), droneId);
  rejoinCount = 0;
  danceHasLanded = false;
  danceHasTakenOff = false;
  danceRole = DANCE_ROLE_NONE;
  danceCriticalPeer = 0;
}

static FlightState checkTransitionDance(void) {
  // Early exit for SECONDARY: while still hovering (haven't landed), if the
  // critical peer has moved well clear AND hasn't landed itself, the danger
  // is over — go straight back to normal flight instead of also landing.
  if (danceRole == DANCE_ROLE_SECONDARY && !danceHasLanded) {
    if (!isPeerLandedById(danceCriticalPeer)) {
      uint32_t d = peerDistanceById(danceCriticalPeer);
      if (d == 0U || d > peerCloseMm) {
        DEBUG_PRINT("[%.2f] DANCE -> %s: drone %u SECONDARY clear (peer %u dist=%lu)\n",
                    (double)getTimestamp(),
                    (ctx.d0 < innerBoundMm) ? "STRAIGHT" : "RECOVER",
                    droneId, danceCriticalPeer, (unsigned long)d);
        return (ctx.d0 < innerBoundMm) ? STATE_STRAIGHT : STATE_RECOVER;
      }
    }
  }

  if (!danceHasTakenOff) {
    return STATE_DANCE;  // still hovering / landing / waiting
  }

  // Take-off complete; rejoin via STRAIGHT or RECOVER based on d0.
  if (ctx.d0 < innerBoundMm) {
    DEBUG_PRINT("[%.2f] DANCE -> STRAIGHT: drone %u rejoining (d0=%lu)\n",
                (double)getTimestamp(), droneId, (unsigned long)ctx.d0);
    return STATE_STRAIGHT;
  } else {
    DEBUG_PRINT("[%.2f] DANCE -> RECOVER: drone %u rejoining outside bound (d0=%lu)\n",
                (double)getTimestamp(), droneId, (unsigned long)ctx.d0);
    return STATE_RECOVER;
  }
}

static void executeDance(void) {
  // Behavior by role:
  //   SANDWICH / PRIMARY: land immediately, wait for peers far, take off.
  //   SECONDARY: hover in place (no forward, no yaw) until the PRIMARY (the
  //              critical peer) is detected on the ground; then promote myself
  //              to PRIMARY and land. Sequencing the two landings prevents the
  //              mutual-drift collision seen when both descend together.
  //
  //   Phase 1: land  |  Phase 2: wait until ALL flying peers far  |  Phase 3: take off
  if (danceRole == DANCE_ROLE_SECONDARY && !danceHasLanded) {
    // Hold high and motionless while the PRIMARY descends.
    sendHover(0.0f, 0.0f, targetHeightM, 0.0f);

    if (isPeerLandedById(danceCriticalPeer)) {
      DEBUG_PRINT("[%.2f] DANCE: drone %u SECONDARY -> PRIMARY (peer %u is down)\n",
                  (double)getTimestamp(), droneId, danceCriticalPeer);
      danceRole = DANCE_ROLE_PRIMARY;  // fall through next iteration into landing
    }
    return;
  }

  if (!danceHasLanded) {
    sendHover(0.0f, 0.0f, targetHeightM, 0.0f);  // momentary freeze
    DEBUG_PRINT("[%.2f] DANCE: drone %u landing (role=%d, peers close=%u)\n",
                (double)getTimestamp(), droneId, (int)danceRole,
                countFlyingPeersWithin(peerCloseMm));
    landToZero();
    danceHasLanded = true;
    DEBUG_PRINT("[%.2f] DANCE: drone %u landed, waiting for peers to clear\n",
                (double)getTimestamp(), droneId);
    return;
  }

  if (!danceHasTakenOff) {
    // Keep the commander watchdog happy with a no-op setpoint while motors are off.
    setpoint_t idle;
    memset(&idle, 0, sizeof(idle));
    idle.mode.x = modeDisable;
    idle.mode.y = modeDisable;
    idle.mode.z = modeDisable;
    idle.mode.yaw = modeDisable;
    idle.thrust = 0;
    commanderSetSetpoint(&idle, 3);

    const uint32_t rejoinThresh = peerCloseMm + REJOIN_EXTRA_MM;
    if (areAllFlyingPeersFar(rejoinThresh)) {
      if (rejoinCount < 0xFF) rejoinCount++;
    } else {
      rejoinCount = 0;
    }

    if (rejoinCount >= REJOIN_CONFIRM_COUNT) {
      DEBUG_PRINT("[%.2f] DANCE: drone %u taking off (all peers >= %lu mm)\n",
                  (double)getTimestamp(), droneId, (unsigned long)rejoinThresh);
      rampToHeight(targetHeightM, RAMP_TIME_MS);
      danceHasTakenOff = true;
      DEBUG_PRINT("[%.2f] DANCE: drone %u airborne, rejoining swarm\n",
                  (double)getTimestamp(), droneId);
    }
  }
}

// --- OBSTACLE state ---
// Phase 0: back up for obstBackDurMs, then Phase 1: rotate obstRotDeg toward free side.
// Free side is determined once on entry by comparing left vs right column averages.

static void onEnterObstacle(void) {
  uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

  // Decide turn direction: whichever outer column has larger average distance is "freer"
  int32_t colL = logVarIdIsValid(idObstColL) ? logGetInt(idObstColL) : 4000;
  int32_t colR = logVarIdIsValid(idObstColR) ? logGetInt(idObstColR) : 4000;
  obstFreeDir = (colR > colL) ? 1 : -1;

  obstPhase = 0;
  obstPhaseStartMs = now;
  obstComplete = false;

  DEBUG_PRINT("[%.2f] Enter OBSTACLE: colL=%ld colR=%ld freeDir=%d\n",
              (double)getTimestamp(), (long)colL, (long)colR, (int)obstFreeDir);
}

static void onExitObstacle(void) {
  obstEnterCount = 0;
  obstComplete = false;
  DEBUG_PRINT("[%.2f] Exit OBSTACLE\n", (double)getTimestamp());
}

static FlightState checkTransitionObstacle(void) {
  // OBSTACLE -> UTURN: approaching outer bound
  if (shouldEnterUturn()) {
    DEBUG_PRINT("[%.2f] OBSTACLE -> UTURN: middle bound exceeded (d0=%lu >= %u)\n",
                (double)getTimestamp(), (unsigned long)ctx.d0, getMiddleBoundMm());
    return STATE_UTURN;
  }

  // OBSTACLE -> STRAIGHT or RECOVER: both phases complete
  if (obstComplete) {
    if (ctx.d0 < innerBoundMm) {
      DEBUG_PRINT("[%.2f] OBSTACLE -> STRAIGHT: complete, inside bound (d0=%lu)\n",
                  (double)getTimestamp(), (unsigned long)ctx.d0);
      return STATE_STRAIGHT;
    } else {
      DEBUG_PRINT("[%.2f] OBSTACLE -> RECOVER: complete, outside bound (d0=%lu)\n",
                  (double)getTimestamp(), (unsigned long)ctx.d0);
      return STATE_RECOVER;
    }
  }

  return STATE_OBSTACLE;
}

static void executeObstacle(void) {
  uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

  if (obstPhase == 0) {
    // Phase 0: backup
    if ((now - obstPhaseStartMs) < (uint32_t)obstBackDurMs) {
      sendHover(obstBackVx, 0.0f, targetHeightM, 0.0f);
      return;
    }
    // Transition to rotation phase
    obstPhase = 1;
    obstPhaseStartMs = now;
    obstRotYawStart = logGetFloat(idYaw);
    obstRotTargetYaw = normalizeAngle(obstRotYawStart + (float)obstFreeDir * obstRotDeg);
    DEBUG_PRINT("[%.2f] OBSTACLE phase1: startYaw=%.1f targetYaw=%.1f dir=%d\n",
                (double)getTimestamp(), (double)obstRotYawStart,
                (double)obstRotTargetYaw, (int)obstFreeDir);
  }

  // Phase 1: rotate toward free side
  float curYaw = logGetFloat(idYaw);
  float yawDiff = fabsf(normalizeAngle(curYaw - obstRotTargetYaw));
  if (yawDiff <= OBSTACLE_YAW_TOLERANCE) {
    obstComplete = true;
    sendHover(0.0f, 0.0f, targetHeightM, 0.0f);
    DEBUG_PRINT("[%.2f] OBSTACLE: rotation done, curYaw=%.1f\n",
                (double)getTimestamp(), (double)curYaw);
  } else {
    sendHover(0.0f, 0.0f, targetHeightM, (float)obstFreeDir * obstRotRate);
  }
}

// --- UTURN state (turning phase) ---
static void onEnterUturn(void) {
  DEBUG_PRINT("[%.2f] Enter UTURN (drone %u)\n", (double)getTimestamp(), droneId);
  
  // Capture starting yaw and compute target (180° turn)
  uturnYawStart = logGetFloat(idYaw);
  uturnTargetYaw = normalizeAngle(uturnYawStart + 130.0f);
  uturnActive = isfinite(uturnYawStart);
  
  middleBoundOverCount = 0;
  
  DEBUG_PRINT("[%.2f] UTURN: startYaw=%.1f, targetYaw=%.1f, active=%d\n",
              (double)getTimestamp(), (double)uturnYawStart, (double)uturnTargetYaw, uturnActive ? 1 : 0);
}

static void onExitUturn(void) {
  DEBUG_PRINT("[%.2f] Exit UTURN (drone %u)\n", (double)getTimestamp(), droneId);
  // Set cooldown end time
  uturnCooldownEndTime = (xTaskGetTickCount() * portTICK_PERIOD_MS) + UTURN_COOLDOWN_MS;
  uturnActive = false;
  middleBoundOverCount = 0;
}

static FlightState checkTransitionUturn(void) {

  // UTURN -> DANCE: peer dangerously close (safety override)
  if (shouldEnterDance()) {
    DEBUG_PRINT("[%.2f] Uturn -> DANCE: peer in danger zone (dist=%lu <= %u)\n",
                (double)getTimestamp(), (unsigned long)getPeerDistance(), avoidMinLandMm);
    return STATE_DANCE;
  }
  
  // Check if 180° turn complete
  if (uturnActive) {
    float curYaw = logGetFloat(idYaw);
    if (isfinite(curYaw)) {
      float yawDiff = fabsf(normalizeAngle(curYaw - uturnTargetYaw));
      
      if (yawDiff <= UTURN_YAW_TOLERANCE) {
        // 180° turn complete - transition to fly-back phase
        uturnActive = false;
        DEBUG_PRINT("[%.2f] UTURN -> UTURN_FLY: 180 complete, now flying back (d0=%lu)\n",
                    (double)getTimestamp(), (unsigned long)ctx.d0);
        return STATE_UTURN_FLY;
      }
    } else {
      // Yaw unavailable, abort tracking and go to fly-back anyway
      uturnActive = false;
      DEBUG_PRINT("[%.2f] UTURN -> UTURN_FLY: yaw unavailable, blind fly-back (d0=%lu)\n",
                  (double)getTimestamp(), (unsigned long)ctx.d0);
      return STATE_UTURN_FLY;
    }
  }
  
  // If uturnActive became false somehow, go to fly-back
  if (!uturnActive) {
    DEBUG_PRINT("[%.2f] UTURN -> UTURN_FLY: turn tracking ended (d0=%lu)\n",
                (double)getTimestamp(), (unsigned long)ctx.d0);
    return STATE_UTURN_FLY;
  }
  
  return STATE_UTURN;  // Still turning
}

static void executeUturn(void) {
  // Stop and turn in place at UTURN_YAW_RATE_DPS (zero forward speed)
  sendHover(0.0f, 0.0f, targetHeightM, UTURN_YAW_RATE_DPS);
}

// --- UTURN_FLY state (flying straight back after 180° turn) ---
static void onEnterUturnFly(void) {
  middleBoundOverCount = 0;  // Reset confirmation counter for potential re-entry
  DEBUG_PRINT("[%.2f] Enter UTURN_FLY (drone %u): flying straight, hoping to reach inner bound\n",
              (double)getTimestamp(), droneId);
}

static void onExitUturnFly(void) {
  // Cooldown is time-based from onEnterUturn, no need to reset here
  DEBUG_PRINT("[%.2f] Exit UTURN_FLY (drone %u)\n", (double)getTimestamp(), droneId);
}

static FlightState checkTransitionUturnFly(void) {
  // UTURN_FLY -> AVOID: peer too close (safety override)
  if (shouldEnterAvoid()) {
    DEBUG_PRINT("[%.2f] UTURN_FLY -> AVOID: peer too close\n", (double)getTimestamp());
    return STATE_AVOID;
  }
  
  // UTURN_FLY -> STRAIGHT: reached inner bound (success!)
  if (ctx.d0 > 0 && ctx.d0 <= innerBoundMm) {
    DEBUG_PRINT("[%.2f] UTURN_FLY -> STRAIGHT: made it back! (d0=%lu <= %u)\n",
                (double)getTimestamp(), (unsigned long)ctx.d0, innerBoundMm);
    return STATE_STRAIGHT;
  }
  
  // UTURN_FLY -> UTURN: hit middle bound again (cooldown handled by shouldEnterUturn)
  if (shouldEnterUturn()) {
    DEBUG_PRINT("[%.2f] UTURN_FLY -> UTURN: middle bound exceeded again (d0=%lu >= %u)\n",
                (double)getTimestamp(), (unsigned long)ctx.d0, getMiddleBoundMm());
    return STATE_UTURN;
  }
  
  // If we hit outer bound, emergency land will be triggered by checkAndMaybeEmergencyLand()
  // No transition to TURN or RECOVER - just keep flying straight
  
  return STATE_UTURN_FLY;  // Keep flying straight
}

static void executeUturnFly(void) {
  // Fly straight forward at normal speed
  sendHover(fwdSpeedMps, 0.0f, targetHeightM, 0.0f);
}

// ============================================================================
// State machine dispatcher functions
// ============================================================================
static void onEnterState(FlightState state) {
  switch (state) {
    case STATE_STRAIGHT:  onEnterStraight();   break;
    case STATE_TURN:      onEnterTurn();       break;
    case STATE_AVOID:     onEnterAvoid();      break;
    case STATE_RECOVER:   onEnterRecover();    break;
    case STATE_DANCE:     onEnterDance();      break;
    case STATE_UTURN:     onEnterUturn();      break;
    case STATE_UTURN_FLY: onEnterUturnFly();   break;
    case STATE_OBSTACLE:  onEnterObstacle();   break;
  }
}

static void onExitState(FlightState state) {
  switch (state) {
    case STATE_STRAIGHT:  onExitStraight();   break;
    case STATE_TURN:      onExitTurn();       break;
    case STATE_AVOID:     onExitAvoid();      break;
    case STATE_RECOVER:   onExitRecover();    break;
    case STATE_DANCE:     onExitDance();      break;
    case STATE_UTURN:     onExitUturn();      break;
    case STATE_UTURN_FLY: onExitUturnFly();   break;
    case STATE_OBSTACLE:  onExitObstacle();   break;
  }
}

static FlightState checkTransition(FlightState state) {
  switch (state) {
    case STATE_STRAIGHT:  return checkTransitionStraight();
    case STATE_TURN:      return checkTransitionTurn();
    case STATE_AVOID:     return checkTransitionAvoid();
    case STATE_RECOVER:   return checkTransitionRecover();
    case STATE_DANCE:     return checkTransitionDance();
    case STATE_UTURN:     return checkTransitionUturn();
    case STATE_UTURN_FLY: return checkTransitionUturnFly();
    case STATE_OBSTACLE:  return checkTransitionObstacle();
  }
  return state;
}

static void executeState(FlightState state) {
  switch (state) {
    case STATE_STRAIGHT:  executeStraight();   break;
    case STATE_TURN:      executeTurn();       break;
    case STATE_AVOID:     executeAvoid();      break;
    case STATE_RECOVER:   executeRecover();    break;
    case STATE_DANCE:     executeDance();      break;
    case STATE_UTURN:     executeUturn();      break;
    case STATE_UTURN_FLY: executeUturnFly();   break;
    case STATE_OBSTACLE:  executeObstacle();   break;
  }
}

// ============================================================================
// Main flight sequence
// ============================================================================
static void runSequence(void) {
  // Reset all state
  seqAbort = false;
  innerOverCount = innerUnderCount = 0;
  approachCount = departCount = 0;
  rejoinCount = 0;
  danceRole = DANCE_ROLE_NONE;
  danceCriticalPeer = 0;
  middleBoundOverCount = 0;
  uturnCooldownEndTime = 0;   // Reset cooldown for new sequence
  uturnActive = false;
  obstEnterCount = 0;
  obstComplete = false;
  obstPhase = 0;
  
  // Reset state context
  memset(&ctx, 0, sizeof(ctx));
  ctx.rotationDirection = 1;
  
  // Reset derivative buffers
  d0BufferReset();
  peerDistBufferReset();
  
  // Initialize state machine
  currentState = STATE_STRAIGHT;
  prevState = STATE_STRAIGHT;
  onEnterState(currentState);

  uint32_t startTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

  const char* yawDir = (droneId == 1) ? "CW" : "CCW";
  DEBUG_PRINT("[%.2f] Drone %u: TURN@%u, UTURN@%u(one-shot), AVOID peer<=%u (%s), DANCE peer<=%u, abort@%u\n",
              (double)getTimestamp(), droneId, innerBoundMm, getMiddleBoundMm(), 
              peerCloseMm, yawDir, avoidMinLandMm, dist0AbortMm);

  // Kill check before takeoff (drone 2+ only)
  if (checkKillAndDisarm()) return;

  // Takeoff
  rampToHeight(targetHeightM, RAMP_TIME_MS);

  const uint32_t dtMs = 10;

  while (!seqAbort) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if ((now - startTime) > demoTimeMs) {
      break;
    }

    // Kill check (drone 2+ only)
    if (checkKillAndDisarm()) break;

    // Emergency checks (outer bound) - but NOT during DANCE state for drone 1
    // (drone 1 is landed and d0 reading might be invalid/stale)
    if (currentState != STATE_DANCE || droneId != 1) {
      if (checkAndMaybeEmergencyLand()) break;
    }

    // Update context with current sensor readings
    ctx.d0 = logGetUint(idDistance0);
    d0BufferAdd(ctx.d0, now);
    ctx.d0Deriv = d0BufferGetDerivative();
    
    // Update peer distance and derivative (for AVOID state)
    ctx.peerDist = getPeerDistance();
    peerDistBufferAdd(ctx.peerDist, now);
    ctx.peerDistDeriv = peerDistBufferGetDerivative();

    // Clear arc cooldown once we re-enter the inner circle
    if (ctx.arcCooldown && ctx.d0 > 0 && ctx.d0 <= innerBoundMm) {
      ctx.arcCooldown = false;
      DEBUG_PRINT("[%.2f] ARC cooldown cleared by inner re-entry (d0=%lu)\n", (double)getTimestamp(), (unsigned long)ctx.d0);
    }

    // ========================================================================
    // State machine: check transitions and execute current state
    // ========================================================================
    FlightState nextState = checkTransition(currentState);
    
    if (nextState != currentState) {
      onExitState(currentState);
      prevState = currentState;
      currentState = nextState;
      onEnterState(currentState);
    }

    if (seqAbort) break;

    executeState(currentState);

    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }

  landToZero();
  if (seqAbort) {
    DEBUG_PRINT("[%.2f] Emergency landing\n", (double)getTimestamp());
  } else {
    DEBUG_PRINT("[%.2f] Finished demo in approx %u routines\n", (double)getTimestamp(), ctx.routines);
  }
  commanderRelaxPriority();
}

// ============================================================================
// Velocity Controller Gains Configuration
// ============================================================================
static void setVelocityControllerGains(void) {
  // We set the gains to ensure they are consistent, even if a flapper has different gains stored in memory.
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
// App entry point
// ============================================================================
void appMain(void) {
  // Get drone ID from radio address (last nibble, like lpsTwrTag does)
  droneId = (uint8_t)(configblockGetRadioAddress() & 0xF);
  DEBUG_PRINT("[%.2f] Flapper Swarm App started, droneId=%u (from radio address)\n", (double)getTimestamp(), droneId);

  
  // Resolve log IDs based on droneId
  // Common IDs for all drones
  while (!logVarIdIsValid(idDistance0) || !logVarIdIsValid(idZ) || !logVarIdIsValid(idYaw)) {
    ensureLogId(&idDistance0, "ranging", "distance0");
    ensureLogId(&idZ,         "stateEstimate", "z");
    ensureLogId(&idYaw,       "stateEstimate", "yaw");
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  
  // Debug: optic flow IDs (optional, don't block if unavailable)
  ensureLogId(&idMotionDeltaX, "motion", "deltaX");
  ensureLogId(&idMotionDeltaY, "motion", "deltaY");
  ensureLogId(&idZrange,       "range", "zrange");

  // Obstacle: VL53L8CX column averages (optional, require zranger3 deck)
  ensureLogId(&idObstColL,  "range8", "colL");
  ensureLogId(&idObstColML, "range8", "colML");
  ensureLogId(&idObstColMR, "range8", "colMR");
  ensureLogId(&idObstColR,  "range8", "colR");

  // Drone-specific IDs
  // Trigger/kill source:
  //   - Drone 1: RC via cppm.aux0 (active low), no kill
  //   - Drone 2+: UWB via ranging.aux1 (trigger) + ranging.aux2 (kill)
  // Peer logs: each drone resolves distance/height for the OTHER two drones
  // so the avoidance logic can see both peers (used by sandwich-lands rule).
  if (droneId == 1) {
    while (!logVarIdIsValid(idCppmAux0) ||
           !logVarIdIsValid(idDistance2) || !logVarIdIsValid(idHeight2) ||
           !logVarIdIsValid(idDistance3) || !logVarIdIsValid(idHeight3)) {
      ensureLogId(&idCppmAux0,   "cppm",    "aux0");
      ensureLogId(&idDistance2,  "ranging", "distance2");
      ensureLogId(&idHeight2,    "ranging", "height2");
      ensureLogId(&idDistance3,  "ranging", "distance3");
      ensureLogId(&idHeight3,    "ranging", "height3");
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    DEBUG_PRINT("[%.2f] Drone 1: RC trigger (cppm.aux0<%d), avoid peers 2/3, CW yaw\n",
      (double)getTimestamp(), AUX_RC_ACTIVE_THRESH);
  } else if (droneId == 2) {
    while (!logVarIdIsValid(idRangingAux1) || !logVarIdIsValid(idRangingAux2) ||
           !logVarIdIsValid(idDistance1) || !logVarIdIsValid(idHeight1) ||
           !logVarIdIsValid(idDistance3) || !logVarIdIsValid(idHeight3)) {
      ensureLogId(&idRangingAux1, "ranging", "aux1");
      ensureLogId(&idRangingAux2, "ranging", "aux2");
      ensureLogId(&idDistance1,   "ranging", "distance1");
      ensureLogId(&idHeight1,     "ranging", "height1");
      ensureLogId(&idDistance3,   "ranging", "distance3");
      ensureLogId(&idHeight3,     "ranging", "height3");
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    DEBUG_PRINT("[%.2f] Drone 2: UWB trigger (aux1>%u), kill (aux2), avoid peers 1/3, CCW yaw\n",
      (double)getTimestamp(), AUX_UWB_ACTIVE_THRESHOLD);
  } else {
    // Drone 3 (no forward 4x4 ToF: OBSTACLE state is auto-disabled because the
    // range8 log group isn't registered — see shouldEnterObstacle()).
    while (!logVarIdIsValid(idRangingAux1) || !logVarIdIsValid(idRangingAux2) ||
           !logVarIdIsValid(idDistance1) || !logVarIdIsValid(idHeight1) ||
           !logVarIdIsValid(idDistance2) || !logVarIdIsValid(idHeight2)) {
      ensureLogId(&idRangingAux1, "ranging", "aux1");
      ensureLogId(&idRangingAux2, "ranging", "aux2");
      ensureLogId(&idDistance1,   "ranging", "distance1");
      ensureLogId(&idHeight1,     "ranging", "height1");
      ensureLogId(&idDistance2,   "ranging", "distance2");
      ensureLogId(&idHeight2,     "ranging", "height2");
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    const char *fwdToF = logVarIdIsValid(idObstColML) ? "with forward ToF" : "no forward ToF";
    DEBUG_PRINT("[%.2f] Drone 3: UWB trigger (aux1>%u), kill (aux2), avoid peers 1/2, %s\n",
      (double)getTimestamp(), AUX_UWB_ACTIVE_THRESHOLD, fwdToF);
  }
      
  bool wasActive = false;

  if (droneId != 0) {
    while (1) {
      const bool active = isTriggerActive();
  
      // Emergency always active, even when idle
      if (checkAndMaybeEmergencyLand()) {
        landToZero();
      }
      
      // On rising edge of trigger, run the sequence only if the middle beacon is on.
      if ((logGetUint(idDistance0) > 0) && active && !wasActive) {
        // Set velocity controller gains to ensure consistent behavior
        setVelocityControllerGains();
        runSequence();
      }
      
      wasActive = active;
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}

// ============================================================================
// Parameter definitions (can be changed from client)
// ============================================================================
PARAM_GROUP_START(swarm)
  PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, targetHeight, &targetHeightM)
  PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, hoverMs, &takeoffHoverMs)
  PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, fwdSpeed, &fwdSpeedMps)
  PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, dist0Abort, &dist0AbortMm)
  // Middle ring (UTURN trigger) = dist0Abort - middleOffset. Set offset = 0
  // to effectively disable the UTURN one-shot.
  PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, middleOffset, &middleBoundOffsetMm)
  PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, innerBound, &innerBoundMm)
  PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, innerHyst, &innerBoundHysteresisMm)
  PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, turnYawRate, &turnYawRateDps)
  PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, peerClose, &peerCloseMm)
  PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, abortConfirm, &abortConfirmCount)
  PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, turnExitCfm, &turnExitConfirmCount)
  PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, avoidEnter, &avoidEnterConfirmCount)
  PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, avoidExit, &avoidExitConfirmCount)
  PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, avoidMinLand, &avoidMinLandMm)
  PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, avoidSpeed, &avoidSpeedFactor)
  PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, avoidYawRate, &avoidYawRateMagnitude)
  PARAM_ADD(PARAM_UINT32 | PARAM_PERSISTENT, demoTime, &demoTimeMs)
  PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, obstTrigMm, &obstTrigMm)
  PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, obstBackMs, &obstBackDurMs)
  PARAM_ADD(PARAM_FLOAT  | PARAM_PERSISTENT, obstBackVx, &obstBackVx)
  PARAM_ADD(PARAM_FLOAT  | PARAM_PERSISTENT, obstRotDeg, &obstRotDeg)
  PARAM_ADD(PARAM_FLOAT  | PARAM_PERSISTENT, obstRotRate, &obstRotRate)
  PARAM_ADD(PARAM_UINT8  | PARAM_PERSISTENT, obstEnterCnt, &obstEnterConfirmCount)
PARAM_GROUP_STOP(swarm)

// ============================================================================
// Log definitions (for monitoring state)
// State values: 0=STRAIGHT, 1=TURN, 2=AVOID, 3=RECOVER, 4=DANCE, 5=UTURN, 6=UTURN_FLY, 7=OBSTACLE
// ============================================================================
LOG_GROUP_START(swarm)
  LOG_ADD(LOG_UINT8, state, &currentState)
LOG_GROUP_STOP(swarm)
