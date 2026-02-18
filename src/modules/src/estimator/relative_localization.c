/**
 * Relative Localization EKF
 * 
 * Estimates relative position of peers in the swarm using UWB ranging
 * and shared velocity/gyro data. Based on the EKF formulation from:
 * "Relative Localization for Multi-Robot Systems"
 * 
 * Convention (from perspective of THIS drone, called "self"):
 *   - peer[j] state = [x_j, y_j, z_j, psi_j] = position and heading of peer j
 *     expressed in SELF's body frame
 *   - x: forward, y: left, z: up
 *   - psi: relative heading (peer's heading - self's heading)
 * 
 * Measurements:
 *   - Direct: d_ij = ||p_j - p_i|| (UWB range from self to peer j)
 *   - Indirect: d_jk = ||p_k - p_j|| (UWB range between peers j and k, heard via packet)
 * 
 * Inputs (shared via UWB):
 *   - v_j = [vx_j, vy_j, vz_j] = peer j's velocity in peer j's body frame
 *   - r_j = gyro_z of peer j (yaw rate)
 *   - h_j = height of peer j
 */

#include "estimator/relative_localization.h"
#include "debug.h"

#define DEBUG_MODULE "RELLOC"

#include <string.h>
#include <stdint.h>
#include <math.h>
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "param.h"
#include "log.h"

#include "lpsTwrTag.h"
#include "swarm_info.h"

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

// ============ CONFIGURATION ============

// Maximum number of peers (matches lpstwrtag.c)
#define MAX_PEERS 3  // Changed from LOCODECK_NR_OF_TWR_ANCHORS (was 9)

// State dimension per peer: [x, y, z, psi]
#define STATE_DIM 4
#define STATE_X   0
#define STATE_Y   1
#define STATE_Z   2
#define STATE_PSI 3  // Relative heading

// ============ EKF STATE STRUCTURE ============

typedef struct {
  // State vector: [x, y, z, psi] of this peer in self's body frame
  float S[STATE_DIM];
  
  // Covariance matrix (4x4)
  float P[STATE_DIM][STATE_DIM];
  
  // Tracking
  bool initialized;
  uint32_t lastUpdateTick;
  uint32_t lastPredictTick;
  
  // Last measurements for logging
  uint16_t lastDirectRange_mm;
  
} peerEkfState_t;

// ============ MODULE STATE ============

static bool moduleInitialized = false;

// EKF state for each peer
static peerEkfState_t peers[MAX_PEERS];

// Self state (cached from swarm_info)
static float selfVx, selfVy, selfVz;  // m/s in self's body frame
static float selfGz;                   // rad/s yaw rate
static float selfHeight;               // m

// Configuration (tunable via params)
static uint8_t numPeers = 2;           // Number of active peers to track
static uint8_t useIndirectMeas = 0;    // 0 = direct only, 1 = direct + indirect

// Process noise standard deviations
static float sigmaVxy = 0.2f;          // Velocity noise x/y (m/s)
static float sigmaVz = 0.15f;          // Velocity noise z (m/s)
static float sigmaYawRate = 0.1f;      // Yaw rate noise (rad/s)

// Measurement noise standard deviations
static float sigmaRangeDirect = 0.1f;  // Direct UWB range noise (m)
static float sigmaRangeIndirect = 0.15f; // Indirect UWB range noise (m)

// Initial covariance
static float initCovXY = 1.0f;         // Initial x,y uncertainty (m^2)
static float initCovZ = 0.5f;          // Initial z uncertainty (m^2)
static float initCovPsi = 0.5f;        // Initial heading uncertainty (rad^2)

// Default initial state (used when peer first seen)
static float defaultInitX = 1.0f;
static float defaultInitY = 0.0f;
static float defaultInitZ = 0.0f;
static float defaultInitPsi = 0.0f;

// Connection tracking
static uint8_t connectedMask = 0;
static uint8_t numConnected = 0;
#define PEER_TIMEOUT_TICKS 500  // ~500ms at 1kHz tick

// Logging variables
static float logX[MAX_PEERS];
static float logY[MAX_PEERS];
static float logZ[MAX_PEERS];
static float logPsi[MAX_PEERS];
static float logDist[MAX_PEERS];
static float logPxx[MAX_PEERS];
static float logPyy[MAX_PEERS];

// ============ HELPER FUNCTIONS ============

static inline float wrapToPi(float angle) {
  while (angle > M_PI_F) angle -= 2.0f * M_PI_F;
  while (angle < -M_PI_F) angle += 2.0f * M_PI_F;
  return angle;
}

static inline float sqrtSafe(float x) {
  if (x <= 0.0f) return 0.0f;
  float out;
  if (arm_sqrt_f32(x, &out) != ARM_MATH_SUCCESS) {
    return sqrtf(x);
  }
  return out;
}

// ============ EKF CORE FUNCTIONS ============

/**
 * Initialize EKF state for a peer
 */
static void initPeer(uint8_t peerId, float x, float y, float z, float psi) {
  if (peerId >= MAX_PEERS) return;
  
  peerEkfState_t *p = &peers[peerId];
  
  p->S[STATE_X] = x;
  p->S[STATE_Y] = y;
  p->S[STATE_Z] = z;
  p->S[STATE_PSI] = wrapToPi(psi);
  
  // Diagonal covariance
  memset(p->P, 0, sizeof(p->P));
  p->P[STATE_X][STATE_X] = initCovXY;
  p->P[STATE_Y][STATE_Y] = initCovXY;
  p->P[STATE_Z][STATE_Z] = initCovZ;
  p->P[STATE_PSI][STATE_PSI] = initCovPsi;
  
  p->initialized = true;
  p->lastUpdateTick = xTaskGetTickCount();
  p->lastPredictTick = xTaskGetTickCount();
  p->lastDirectRange_mm = 0;
  
  DEBUG_PRINT("Peer %d initialized: (%.2f, %.2f, %.2f, %.2f)\n",
              peerId, (double)x, (double)y, (double)z, (double)psi);
}

/**
 * EKF Prediction Step
 * 
 * State dynamics (from paper):
 *   x_dot = v_j^x * cos(psi) - v_j^y * sin(psi) - v_i^x + r_i * y
 *   y_dot = v_j^x * sin(psi) + v_j^y * cos(psi) - v_i^y - r_i * x
 *   z_dot = v_j^z - v_i^z
 *   psi_dot = r_j - r_i
 * 
 * Where:
 *   - v_j = peer's velocity in peer's body frame
 *   - v_i = self's velocity in self's body frame  
 *   - r_i, r_j = yaw rates
 *   - psi = relative heading (peer - self)
 */
static void ekfPredict(uint8_t peerId, 
                       float peerVx, float peerVy, float peerVz, float peerGz,
                       float dt) {
  if (peerId >= MAX_PEERS || !peers[peerId].initialized) return;
  if (dt <= 0.0f || dt > 1.0f) return;  // Sanity check
  
  peerEkfState_t *p = &peers[peerId];
  
  // Current state
  float x = p->S[STATE_X];
  float y = p->S[STATE_Y];
  float z = p->S[STATE_Z];
  float psi = p->S[STATE_PSI];
  
  float cpsi = arm_cos_f32(psi);
  float spsi = arm_sin_f32(psi);
  
  // State prediction (Eq. from paper)
  float x_dot = peerVx * cpsi - peerVy * spsi - selfVx + selfGz * y;
  float y_dot = peerVx * spsi + peerVy * cpsi - selfVy - selfGz * x;
  float z_dot = peerVz - selfVz;
  float psi_dot = peerGz - selfGz;
  
  p->S[STATE_X] = x + x_dot * dt;
  p->S[STATE_Y] = y + y_dot * dt;
  p->S[STATE_Z] = z + z_dot * dt;
  p->S[STATE_PSI] = wrapToPi(psi + psi_dot * dt);
  
  // Jacobian F = df/dx (state transition matrix)
  // F = I + A*dt where A = df/dx at current state
  float F[STATE_DIM][STATE_DIM];
  
  F[0][0] = 1.0f;
  F[0][1] = selfGz * dt;
  F[0][2] = 0.0f;
  F[0][3] = (-peerVx * spsi - peerVy * cpsi) * dt;
  
  F[1][0] = -selfGz * dt;
  F[1][1] = 1.0f;
  F[1][2] = 0.0f;
  F[1][3] = (peerVx * cpsi - peerVy * spsi) * dt;
  
  F[2][0] = 0.0f;
  F[2][1] = 0.0f;
  F[2][2] = 1.0f;
  F[2][3] = 0.0f;
  
  F[3][0] = 0.0f;
  F[3][1] = 0.0f;
  F[3][2] = 0.0f;
  F[3][3] = 1.0f;
  
  // Process noise Q (diagonal)
  float Q[STATE_DIM][STATE_DIM] = {{0}};
  float dt2 = dt * dt;
  Q[0][0] = sigmaVxy * sigmaVxy * dt2;
  Q[1][1] = sigmaVxy * sigmaVxy * dt2;
  Q[2][2] = sigmaVz * sigmaVz * dt2;
  Q[3][3] = sigmaYawRate * sigmaYawRate * dt2;
  
  // Covariance prediction: P = F * P * F^T + Q
  float FP[STATE_DIM][STATE_DIM];
  float FPFT[STATE_DIM][STATE_DIM];
  
  // FP = F * P
  for (int i = 0; i < STATE_DIM; i++) {
    for (int j = 0; j < STATE_DIM; j++) {
      FP[i][j] = 0.0f;
      for (int k = 0; k < STATE_DIM; k++) {
        FP[i][j] += F[i][k] * p->P[k][j];
      }
    }
  }
  
  // FPFT = FP * F^T
  for (int i = 0; i < STATE_DIM; i++) {
    for (int j = 0; j < STATE_DIM; j++) {
      FPFT[i][j] = 0.0f;
      for (int k = 0; k < STATE_DIM; k++) {
        FPFT[i][j] += FP[i][k] * F[j][k];  // F^T[k][j] = F[j][k]
      }
    }
  }
  
  // P = FPFT + Q
  for (int i = 0; i < STATE_DIM; i++) {
    for (int j = 0; j < STATE_DIM; j++) {
      p->P[i][j] = FPFT[i][j] + Q[i][j];
    }
  }
  
  p->lastPredictTick = xTaskGetTickCount();
}

/**
 * EKF Update - Direct Range Measurement
 * 
 * Measurement model: h(x) = sqrt(x^2 + y^2 + z^2)
 * This is the range from self to peer j.
 */
static void ekfUpdateDirect(uint8_t peerId, uint16_t rangeMm) {
  if (peerId >= MAX_PEERS || !peers[peerId].initialized) return;
  if (rangeMm == 0) return;
  
  peerEkfState_t *p = &peers[peerId];
  
  float range = (float)rangeMm / 1000.0f;  // Convert to meters
  
  // Predicted measurement
  float x = p->S[STATE_X];
  float y = p->S[STATE_Y];
  float z = p->S[STATE_Z];
  float predRange = sqrtSafe(x*x + y*y + z*z);
  
  if (predRange < 0.01f) predRange = 0.01f;
  
  // Measurement Jacobian H = dh/dx = [x/d, y/d, z/d, 0]
  float H[STATE_DIM];
  H[STATE_X] = x / predRange;
  H[STATE_Y] = y / predRange;
  H[STATE_Z] = z / predRange;
  H[STATE_PSI] = 0.0f;
  
  // Innovation
  float y_innov = range - predRange;
  
  // PHT = P * H^T (since H is 1xN, PHT is Nx1)
  float PHT[STATE_DIM];
  for (int i = 0; i < STATE_DIM; i++) {
    PHT[i] = 0.0f;
    for (int j = 0; j < STATE_DIM; j++) {
      PHT[i] += p->P[i][j] * H[j];
    }
  }
  
  // S = H * P * H^T + R (scalar)
  float S = sigmaRangeDirect * sigmaRangeDirect;
  for (int i = 0; i < STATE_DIM; i++) {
    S += H[i] * PHT[i];
  }
  
  if (S < 1e-6f) S = 1e-6f;
  
  // Kalman gain K = PHT / S
  float K[STATE_DIM];
  for (int i = 0; i < STATE_DIM; i++) {
    K[i] = PHT[i] / S;
  }
  
  // State update: x = x + K * y
  for (int i = 0; i < STATE_DIM; i++) {
    p->S[i] += K[i] * y_innov;
  }
  p->S[STATE_PSI] = wrapToPi(p->S[STATE_PSI]);
  
  // Covariance update: P = (I - K*H) * P
  // Using standard form (could use Joseph form for better numerics)
  float KH[STATE_DIM][STATE_DIM];
  for (int i = 0; i < STATE_DIM; i++) {
    for (int j = 0; j < STATE_DIM; j++) {
      KH[i][j] = K[i] * H[j];
    }
  }
  
  float P_new[STATE_DIM][STATE_DIM];
  for (int i = 0; i < STATE_DIM; i++) {
    for (int j = 0; j < STATE_DIM; j++) {
      P_new[i][j] = 0.0f;
      for (int k = 0; k < STATE_DIM; k++) {
        float I_KH_ik = (i == k ? 1.0f : 0.0f) - KH[i][k];
        P_new[i][j] += I_KH_ik * p->P[k][j];
      }
    }
  }
  memcpy(p->P, P_new, sizeof(p->P));
  
  // Ensure positive definiteness
  for (int i = 0; i < STATE_DIM; i++) {
    if (p->P[i][i] < 1e-6f) p->P[i][i] = 1e-6f;
  }
  
  p->lastDirectRange_mm = rangeMm;
  p->lastUpdateTick = xTaskGetTickCount();
}

/**
 * EKF Update - Indirect Range Measurement
 * 
 * Uses the range between two other peers (j and k) to improve estimates.
 * Measurement model: h(x_j, x_k) = ||p_k - p_j||
 * 
 * This constrains the relative positions of peers j and k.
 */
static void ekfUpdateIndirect(uint8_t peerJ, uint8_t peerK, uint16_t rangeMm) {
  if (!useIndirectMeas) return;
  if (peerJ >= MAX_PEERS || peerK >= MAX_PEERS) return;
  if (!peers[peerJ].initialized || !peers[peerK].initialized) return;
  if (rangeMm == 0) return;
  if (peerJ == peerK) return;
  
  float range = (float)rangeMm / 1000.0f;
  
  // Relative position between peers j and k (in self's frame)
  float dx = peers[peerK].S[STATE_X] - peers[peerJ].S[STATE_X];
  float dy = peers[peerK].S[STATE_Y] - peers[peerJ].S[STATE_Y];
  float dz = peers[peerK].S[STATE_Z] - peers[peerJ].S[STATE_Z];
  float predRange = sqrtSafe(dx*dx + dy*dy + dz*dz);
  
  if (predRange < 0.01f) predRange = 0.01f;
  
  // Innovation
  float y_innov = range - predRange;
  
  // For indirect measurements, we use a simplified update:
  // Distribute the correction proportionally based on covariance
  float R = sigmaRangeIndirect * sigmaRangeIndirect;
  
  // Direction unit vector
  float nx = dx / predRange;
  float ny = dy / predRange;
  float nz = dz / predRange;
  
  // Compute gain based on relative uncertainties
  float varJ = peers[peerJ].P[STATE_X][STATE_X] + peers[peerJ].P[STATE_Y][STATE_Y] + peers[peerJ].P[STATE_Z][STATE_Z];
  float varK = peers[peerK].P[STATE_X][STATE_X] + peers[peerK].P[STATE_Y][STATE_Y] + peers[peerK].P[STATE_Z][STATE_Z];
  float totalVar = varJ + varK + R;
  
  if (totalVar < 1e-6f) totalVar = 1e-6f;
  
  // Correction factors (peers with higher uncertainty get more correction)
  float alphaJ = varJ / totalVar;
  float alphaK = varK / totalVar;
  
  // Update peer J (move toward peer K if range is longer than predicted)
  peers[peerJ].S[STATE_X] -= alphaJ * y_innov * nx;
  peers[peerJ].S[STATE_Y] -= alphaJ * y_innov * ny;
  peers[peerJ].S[STATE_Z] -= alphaJ * y_innov * nz;
  
  // Update peer K (move away from peer J if range is longer than predicted)
  peers[peerK].S[STATE_X] += alphaK * y_innov * nx;
  peers[peerK].S[STATE_Y] += alphaK * y_innov * ny;
  peers[peerK].S[STATE_Z] += alphaK * y_innov * nz;
  
  // Reduce covariance (simplified)
  float covReduction = 0.9f;
  for (int i = 0; i < 3; i++) {  // Only position states
    peers[peerJ].P[i][i] *= covReduction;
    peers[peerK].P[i][i] *= covReduction;
  }
}

// ============ PUBLIC API ============

/**
 * Initialize the relative localization module
 */
void relativeLocalizationInit(void) {
  if (moduleInitialized) return;
  
  for (int i = 0; i < MAX_PEERS; i++) {
    memset(&peers[i], 0, sizeof(peerEkfState_t));
    peers[i].initialized = false;
  }
  
  connectedMask = 0;
  numConnected = 0;
  
  moduleInitialized = true;
  DEBUG_PRINT("Relative localization module initialized (max %d peers)\n", MAX_PEERS);
}

/**
 * Reset EKF state for a specific peer
 */
void relativeLocalizationResetPeer(uint8_t peerId) {
  if (peerId >= MAX_PEERS) return;
  memset(&peers[peerId], 0, sizeof(peerEkfState_t));
  peers[peerId].initialized = false;
}

/**
 * Reset all EKF states
 */
void relativeLocalizationReset(void) {
  for (int i = 0; i < MAX_PEERS; i++) {
    relativeLocalizationResetPeer(i);
  }
  connectedMask = 0;
  numConnected = 0;
}

/**
 * Main update function - call this periodically (e.g., 100Hz from your app)
 * 
 * @param dt Time step in seconds since last call
 * @param useIndirect If true, also process indirect measurements
 */
void relativeLocalizationUpdate(float dt, bool useIndirect) {
  if (!moduleInitialized) {
    relativeLocalizationInit();
  }
  
  // Get self state from swarm_info
  float selfX_unused, selfY_unused;
  swarmInfoGet(&selfX_unused, &selfY_unused, &selfGz, &selfHeight, &selfVx, &selfVy, &selfVz);
  
  // Convert gyro from deg/s to rad/s
  selfGz = selfGz * M_PI_F / 180.0f;
  
  // Update connection tracking
  uint32_t currentTick = xTaskGetTickCount();
  connectedMask = 0;
  numConnected = 0;
  
  // Process each peer
  for (uint8_t j = 0; j < numPeers && j < MAX_PEERS; j++) {
    uint16_t range;
    float peerX, peerY, peerGz, peerH, peerVx, peerVy, peerVz;
    
    // Try to get new data from this peer
    if (twrGetSwarmInfo(j, &range, &peerX, &peerY, &peerGz, &peerH, &peerVx, &peerVy, &peerVz)) {
      
      // Initialize if first contact
      if (!peers[j].initialized) {
        // Use range for initial distance estimate
        float initDist = (range > 100) ? ((float)range / 1000.0f) : defaultInitX;
        initPeer(j, initDist, defaultInitY, peerH - selfHeight, defaultInitPsi);
      }
      
      // Convert gyro to rad/s
      float peerGzRad = peerGz * M_PI_F / 180.0f;
      
      // EKF prediction step
      ekfPredict(j, peerVx, peerVy, peerVz, peerGzRad, dt);
      
      // EKF update with direct range measurement
      ekfUpdateDirect(j, range);
      
      // Mark as connected
      connectedMask |= (1 << j);
      numConnected++;
      
    } else if (peers[j].initialized) {
      // No new data - coast with zero peer velocity
      ekfPredict(j, 0.0f, 0.0f, 0.0f, 0.0f, dt);
      
      // Check if still connected
      if ((currentTick - peers[j].lastUpdateTick) < PEER_TIMEOUT_TICKS) {
        connectedMask |= (1 << j);
        numConnected++;
      }
    }
    
    // Update logging variables
    if (j < MAX_PEERS) {
      logX[j] = peers[j].S[STATE_X];
      logY[j] = peers[j].S[STATE_Y];
      logZ[j] = peers[j].S[STATE_Z];
      logPsi[j] = peers[j].S[STATE_PSI];
      logDist[j] = (float)peers[j].lastDirectRange_mm / 1000.0f;
      logPxx[j] = peers[j].P[STATE_X][STATE_X];
      logPyy[j] = peers[j].P[STATE_Y][STATE_Y];
    }
  }
  
  // Process indirect measurements if enabled
  if (useIndirect || useIndirectMeas) {
    // Get indirect distances from lpsTwrTag
    for (uint8_t j = 0; j < numPeers && j < MAX_PEERS; j++) {
      for (uint8_t k = j + 1; k < numPeers && k < MAX_PEERS; k++) {
        uint16_t indirectRange = twrGetIndirectDist(j, k);
        if (indirectRange > 0) {
          ekfUpdateIndirect(j, k, indirectRange);
        }
      }
    }
  }
}

/**
 * Get relative state of a peer
 * 
 * @param peerId Peer index
 * @param state Output array [x, y, z, psi] - position in self's body frame, relative heading
 * @return true if peer is initialized and connected
 */
bool relativeLocalizationGetState(uint8_t peerId, float *state) {
  if (peerId >= MAX_PEERS || state == NULL) return false;
  if (!peers[peerId].initialized) return false;
  
  state[0] = peers[peerId].S[STATE_X];
  state[1] = peers[peerId].S[STATE_Y];
  state[2] = peers[peerId].S[STATE_Z];
  state[3] = peers[peerId].S[STATE_PSI];
  
  return (connectedMask & (1 << peerId)) != 0;
}

/**
 * Get covariance diagonal for a peer
 */
bool relativeLocalizationGetCovariance(uint8_t peerId, float *cov) {
  if (peerId >= MAX_PEERS || cov == NULL) return false;
  if (!peers[peerId].initialized) return false;
  
  cov[0] = peers[peerId].P[STATE_X][STATE_X];
  cov[1] = peers[peerId].P[STATE_Y][STATE_Y];
  cov[2] = peers[peerId].P[STATE_Z][STATE_Z];
  cov[3] = peers[peerId].P[STATE_PSI][STATE_PSI];
  
  return true;
}

/**
 * Check if any peer is connected
 */
bool relativeLocalizationIsConnected(void) {
  return numConnected > 0;
}

/**
 * Get bitmask of connected peers
 */
uint8_t relativeLocalizationGetConnectedMask(void) {
  return connectedMask;
}

/**
 * Get number of connected peers
 */
uint8_t relativeLocalizationGetNumConnected(void) {
  return numConnected;
}

/**
 * Set number of peers to track
 */
void relativeLocalizationSetNumPeers(uint8_t n) {
  if (n > MAX_PEERS) n = MAX_PEERS;
  numPeers = n;
}

/**
 * Enable/disable indirect measurements
 */
void relativeLocalizationSetUseIndirect(bool enable) {
  useIndirectMeas = enable ? 1 : 0;
}

// ============ LOG VARIABLES ============

LOG_GROUP_START(relLoc)
// Peer 0 state
LOG_ADD(LOG_FLOAT, x0, &logX[0])
LOG_ADD(LOG_FLOAT, y0, &logY[0])
LOG_ADD(LOG_FLOAT, z0, &logZ[0])
LOG_ADD(LOG_FLOAT, psi0, &logPsi[0])
LOG_ADD(LOG_FLOAT, d0, &logDist[0])
LOG_ADD(LOG_FLOAT, Px0, &logPxx[0])
LOG_ADD(LOG_FLOAT, Py0, &logPyy[0])
// Peer 1 state
LOG_ADD(LOG_FLOAT, x1, &logX[1])
LOG_ADD(LOG_FLOAT, y1, &logY[1])
LOG_ADD(LOG_FLOAT, z1, &logZ[1])
LOG_ADD(LOG_FLOAT, psi1, &logPsi[1])
LOG_ADD(LOG_FLOAT, d1, &logDist[1])
LOG_ADD(LOG_FLOAT, Px1, &logPxx[1])
LOG_ADD(LOG_FLOAT, Py1, &logPyy[1])
// Peer 2 state
LOG_ADD(LOG_FLOAT, x2, &logX[2])
LOG_ADD(LOG_FLOAT, y2, &logY[2])
LOG_ADD(LOG_FLOAT, z2, &logZ[2])
LOG_ADD(LOG_FLOAT, psi2, &logPsi[2])
LOG_ADD(LOG_FLOAT, d2, &logDist[2])
LOG_ADD(LOG_FLOAT, Px2, &logPxx[2])
LOG_ADD(LOG_FLOAT, Py2, &logPyy[2])
// Connection status
LOG_ADD(LOG_UINT8, connMask, &connectedMask)
LOG_ADD(LOG_UINT8, numConn, &numConnected)
LOG_GROUP_STOP(relLoc)

// Self state for debugging
LOG_GROUP_START(relSelf)
LOG_ADD(LOG_FLOAT, vx, &selfVx)
LOG_ADD(LOG_FLOAT, vy, &selfVy)
LOG_ADD(LOG_FLOAT, vz, &selfVz)
LOG_ADD(LOG_FLOAT, gz, &selfGz)
LOG_ADD(LOG_FLOAT, h, &selfHeight)
LOG_GROUP_STOP(relSelf)

// ============ PARAMETERS ============

PARAM_GROUP_START(relLoc)
// Number of peers to track
PARAM_ADD(PARAM_UINT8, numPeers, &numPeers)
// Use indirect measurements (0=off, 1=on)
PARAM_ADD(PARAM_UINT8, useInd, &useIndirectMeas)
// Process noise
PARAM_ADD(PARAM_FLOAT, sigVxy, &sigmaVxy)
PARAM_ADD(PARAM_FLOAT, sigVz, &sigmaVz)
PARAM_ADD(PARAM_FLOAT, sigYaw, &sigmaYawRate)
// Measurement noise
PARAM_ADD(PARAM_FLOAT, sigRngD, &sigmaRangeDirect)
PARAM_ADD(PARAM_FLOAT, sigRngI, &sigmaRangeIndirect)
// Initial covariance
PARAM_ADD(PARAM_FLOAT, initPxy, &initCovXY)
PARAM_ADD(PARAM_FLOAT, initPz, &initCovZ)
PARAM_ADD(PARAM_FLOAT, initPpsi, &initCovPsi)
// Default initial state
PARAM_ADD(PARAM_FLOAT, defX, &defaultInitX)
PARAM_ADD(PARAM_FLOAT, defY, &defaultInitY)
PARAM_ADD(PARAM_FLOAT, defZ, &defaultInitZ)
PARAM_ADD(PARAM_FLOAT, defPsi, &defaultInitPsi)
PARAM_GROUP_STOP(relLoc)