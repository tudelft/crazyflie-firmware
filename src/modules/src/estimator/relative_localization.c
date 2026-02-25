/**
 * Relative Localization EKF (Augmented, fCRL-style) with static scratch buffers
 *
 * Patch goal:
 *   Avoid large stack allocations that can corrupt memory and break radio link.
 *   All large temporary matrices are file-scope static buffers.
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

#define MAX_PEERS 3

#define STATE_DIM 4
#define STATE_X   0
#define STATE_Y   1
#define STATE_Z   2
#define STATE_PSI 3

#define TOTAL_STATE_DIM (MAX_PEERS * STATE_DIM)

#define IDX(j, s) ((int)(j) * STATE_DIM + (int)(s))

#define PEER_TIMEOUT_TICKS 500

// ============ MODULE STATE ============

static bool moduleInitialized = false;

// Augmented state and covariance
static float X[TOTAL_STATE_DIM];
static float P[TOTAL_STATE_DIM][TOTAL_STATE_DIM];

// Per peer tracking
static bool peerInitialized[MAX_PEERS];
static uint32_t lastUpdateTick[MAX_PEERS];
static uint32_t lastPredictTick[MAX_PEERS];
static uint16_t lastDirectRange_mm[MAX_PEERS];

// Self state
static float selfVx, selfVy, selfVz;
static float selfGz;       // rad/s
static float selfHeight;   // m

// Configuration
static uint8_t selfId = 0xFF;   // Set via relativeLocalizationSetSelfId(); 0xFF = unset
static uint8_t numPeers = 3;    // Total swarm size (including self); loop skips selfId
static uint8_t useIndirectMeas = 0;

// Noise
static float sigmaVxy = 0.2f;
static float sigmaVz = 0.15f;
static float sigmaYawRate = 0.1f;

static float sigmaRangeDirect = 0.1f;
static float sigmaRangeIndirect = 0.15f;

// Initial covariance
static float initCovXY = 1.0f;
static float initCovZ = 0.5f;
static float initCovPsi = 0.5f;

// Default initial state
static float defaultInitX = 0.0f;
static float defaultInitY = 0.0f;
static float defaultInitZ = 0.0f;
static float defaultInitPsi = 0.0f;

// Connection tracking
static uint8_t connectedMask = 0;
static uint8_t numConnected = 0;

// Logging variables
static float logX[MAX_PEERS];
static float logY[MAX_PEERS];
static float logZ[MAX_PEERS];
static float logPsi[MAX_PEERS];
static float logDist[MAX_PEERS];
static float logPxx[MAX_PEERS];
static float logPyy[MAX_PEERS];

// Debug logging - raw EKF inputs and innovation
static uint16_t dbgRawRange_mm[MAX_PEERS];   // direct UWB range in mm
static float    dbgPeerVx[MAX_PEERS];         // peer's self-reported vx
static float    dbgPeerVy[MAX_PEERS];
static float    dbgPeerGz[MAX_PEERS];         // peer's self-reported gyro z (rad/s)
static float    dbgPeerH[MAX_PEERS];          // peer's self-reported height
static uint8_t  dbgPeerValid[MAX_PEERS];      // 1 if twrGetSwarmInfo succeeded
static float    dbgPredRange[MAX_PEERS];      // EKF predicted range
static float    dbgInnov[MAX_PEERS];          // innovation = measured - predicted
static uint16_t dbgIndRange01_mm;             // indirect range peer 0<->1
static uint16_t dbgIndRange02_mm;
static uint16_t dbgIndRange12_mm;

// ============ STATIC SCRATCH BUFFERS (PATCH) ============
// These used to be stack locals, now they are static to prevent stack overflow.

static float F[TOTAL_STATE_DIM][TOTAL_STATE_DIM];
static float Q[TOTAL_STATE_DIM][TOTAL_STATE_DIM];
static float FP[TOTAL_STATE_DIM][TOTAL_STATE_DIM];
static float FPFT[TOTAL_STATE_DIM][TOTAL_STATE_DIM];

static float Hvec[TOTAL_STATE_DIM];
static float PHT[TOTAL_STATE_DIM];
static float Kvec[TOTAL_STATE_DIM];

static float I_KH[TOTAL_STATE_DIM][TOTAL_STATE_DIM];
static float tmpMat[TOTAL_STATE_DIM][TOTAL_STATE_DIM];
static float newP[TOTAL_STATE_DIM][TOTAL_STATE_DIM];

// ============ HELPERS ============

static inline float wrapToPi(float a) {
  while (a > M_PI_F)  a -= 2.0f * M_PI_F;
  while (a < -M_PI_F) a += 2.0f * M_PI_F;
  return a;
}

static inline float sqrtSafe(float x) {
  if (x <= 0.0f) return 0.0f;
  float out;
  if (arm_sqrt_f32(x, &out) != ARM_MATH_SUCCESS) {
    return sqrtf(x);
  }
  return out;
}

static inline void enforceSymmetry(void) {
  for (int i = 0; i < TOTAL_STATE_DIM; i++) {
    for (int j = i + 1; j < TOTAL_STATE_DIM; j++) {
      float a = 0.5f * (P[i][j] + P[j][i]);
      P[i][j] = a;
      P[j][i] = a;
    }
  }
}

static inline void clampDiagonal(void) {
  for (int i = 0; i < TOTAL_STATE_DIM; i++) {
    if (P[i][i] < 1e-9f) P[i][i] = 1e-9f;
  }
}

static void resetPeerBlock(uint8_t peerId) {
  if (peerId >= MAX_PEERS) return;

  // Zero state block
  for (int s = 0; s < STATE_DIM; s++) {
    X[IDX(peerId, s)] = 0.0f;
  }

  // Zero covariance rows and cols
  for (int i = 0; i < TOTAL_STATE_DIM; i++) {
    for (int s = 0; s < STATE_DIM; s++) {
      int gi = IDX(peerId, s);
      P[gi][i] = 0.0f;
      P[i][gi] = 0.0f;
    }
  }

  peerInitialized[peerId] = false;
  lastUpdateTick[peerId] = 0;
  lastPredictTick[peerId] = 0;
  lastDirectRange_mm[peerId] = 0;
}

static void initPeer(uint8_t peerId, float x, float y, float z, float psi) {
  if (peerId >= MAX_PEERS) return;

  resetPeerBlock(peerId);

  X[IDX(peerId, STATE_X)]   = x;
  X[IDX(peerId, STATE_Y)]   = y;
  X[IDX(peerId, STATE_Z)]   = z;
  X[IDX(peerId, STATE_PSI)] = wrapToPi(psi);

  P[IDX(peerId, STATE_X)][IDX(peerId, STATE_X)]     = initCovXY;
  P[IDX(peerId, STATE_Y)][IDX(peerId, STATE_Y)]     = initCovXY;
  P[IDX(peerId, STATE_Z)][IDX(peerId, STATE_Z)]     = initCovZ;
  P[IDX(peerId, STATE_PSI)][IDX(peerId, STATE_PSI)] = initCovPsi;

  peerInitialized[peerId] = true;
  uint32_t now = xTaskGetTickCount();
  lastUpdateTick[peerId] = now;
  lastPredictTick[peerId] = now;

  DEBUG_PRINT("Peer %d initialized: (%.2f, %.2f, %.2f, %.2f)\n",
              peerId, (double)x, (double)y, (double)z, (double)psi);
}

// ============ EKF CORE ============

static void ekfPredictAll(const float peerVx[MAX_PEERS],
                          const float peerVy[MAX_PEERS],
                          const float peerVz[MAX_PEERS],
                          const float peerGz[MAX_PEERS],
                          float dt) {
  if (dt <= 0.0f || dt > 1.0f) return;

  // F = I, Q = 0
  memset(F, 0, sizeof(F));
  memset(Q, 0, sizeof(Q));
  for (int i = 0; i < TOTAL_STATE_DIM; i++) {
    F[i][i] = 1.0f;
  }

  float dt2 = dt * dt;

  for (uint8_t j = 0; j < numPeers && j < MAX_PEERS; j++) {
    if (j == selfId) continue;  // Never predict for self
    if (!peerInitialized[j]) continue;

    float x   = X[IDX(j, STATE_X)];
    float y   = X[IDX(j, STATE_Y)];
    float z   = X[IDX(j, STATE_Z)];
    float psi = X[IDX(j, STATE_PSI)];

    float cpsi = arm_cos_f32(psi);
    float spsi = arm_sin_f32(psi);

    float x_dot   = peerVx[j] * cpsi - peerVy[j] * spsi - selfVx + selfGz * y;
    float y_dot   = peerVx[j] * spsi + peerVy[j] * cpsi - selfVy - selfGz * x;
    float z_dot   = peerVz[j] - selfVz;
    float psi_dot = peerGz[j] - selfGz;

    X[IDX(j, STATE_X)]   = x + x_dot * dt;
    X[IDX(j, STATE_Y)]   = y + y_dot * dt;
    X[IDX(j, STATE_Z)]   = z + z_dot * dt;
    X[IDX(j, STATE_PSI)] = wrapToPi(psi + psi_dot * dt);

    int gx = IDX(j, STATE_X);
    int gy = IDX(j, STATE_Y);
    int gz = IDX(j, STATE_Z);
    int gp = IDX(j, STATE_PSI);

    // F block for peer j
    F[gx][gx] = 1.0f;
    F[gx][gy] = selfGz * dt;
    F[gx][gz] = 0.0f;
    F[gx][gp] = (-peerVx[j] * spsi - peerVy[j] * cpsi) * dt;

    F[gy][gx] = -selfGz * dt;
    F[gy][gy] = 1.0f;
    F[gy][gz] = 0.0f;
    F[gy][gp] = (peerVx[j] * cpsi - peerVy[j] * spsi) * dt;

    F[gz][gz] = 1.0f;
    F[gp][gp] = 1.0f;

    // Q block (simple diagonal)
    Q[gx][gx] = sigmaVxy * sigmaVxy * dt2;
    Q[gy][gy] = sigmaVxy * sigmaVxy * dt2;
    Q[gz][gz] = sigmaVz  * sigmaVz  * dt2;
    Q[gp][gp] = sigmaYawRate * sigmaYawRate * dt2;

    lastPredictTick[j] = xTaskGetTickCount();
  }

  // FP = F P
  memset(FP, 0, sizeof(FP));
  for (int i = 0; i < TOTAL_STATE_DIM; i++) {
    for (int j = 0; j < TOTAL_STATE_DIM; j++) {
      float acc = 0.0f;
      for (int k = 0; k < TOTAL_STATE_DIM; k++) {
        acc += F[i][k] * P[k][j];
      }
      FP[i][j] = acc;
    }
  }

  // FPFT = FP F^T + Q
  memset(FPFT, 0, sizeof(FPFT));
  for (int i = 0; i < TOTAL_STATE_DIM; i++) {
    for (int j = 0; j < TOTAL_STATE_DIM; j++) {
      float acc = 0.0f;
      for (int k = 0; k < TOTAL_STATE_DIM; k++) {
        acc += FP[i][k] * F[j][k];
      }
      FPFT[i][j] = acc + Q[i][j];
    }
  }

  memcpy(P, FPFT, sizeof(P));
  enforceSymmetry();
  clampDiagonal();
}

static void ekfUpdateScalar(const float *H, float R, float innov) {
  // PHT = P H^T
  for (int i = 0; i < TOTAL_STATE_DIM; i++) {
    float acc = 0.0f;
    for (int j = 0; j < TOTAL_STATE_DIM; j++) {
      acc += P[i][j] * H[j];
    }
    PHT[i] = acc;
  }

  // S = H P H^T + R
  float S = R;
  for (int i = 0; i < TOTAL_STATE_DIM; i++) {
    S += H[i] * PHT[i];
  }
  if (S < 1e-9f) S = 1e-9f;

  // K = PHT / S
  for (int i = 0; i < TOTAL_STATE_DIM; i++) {
    Kvec[i] = PHT[i] / S;
  }

  // State update
  for (int i = 0; i < TOTAL_STATE_DIM; i++) {
    X[i] += Kvec[i] * innov;
  }

  // Wrap psi for all initialized peers
  for (uint8_t j = 0; j < numPeers && j < MAX_PEERS; j++) {
    if (!peerInitialized[j]) continue;
    X[IDX(j, STATE_PSI)] = wrapToPi(X[IDX(j, STATE_PSI)]);
  }

  // Joseph covariance update:
  // P = (I - K H) P (I - K H)^T + K R K^T
  for (int i = 0; i < TOTAL_STATE_DIM; i++) {
    for (int j = 0; j < TOTAL_STATE_DIM; j++) {
      I_KH[i][j] = (i == j ? 1.0f : 0.0f) - Kvec[i] * H[j];
    }
  }

  memset(tmpMat, 0, sizeof(tmpMat));
  for (int i = 0; i < TOTAL_STATE_DIM; i++) {
    for (int j = 0; j < TOTAL_STATE_DIM; j++) {
      float acc = 0.0f;
      for (int k = 0; k < TOTAL_STATE_DIM; k++) {
        acc += I_KH[i][k] * P[k][j];
      }
      tmpMat[i][j] = acc;
    }
  }

  memset(newP, 0, sizeof(newP));
  for (int i = 0; i < TOTAL_STATE_DIM; i++) {
    for (int j = 0; j < TOTAL_STATE_DIM; j++) {
      float acc = 0.0f;
      for (int k = 0; k < TOTAL_STATE_DIM; k++) {
        acc += tmpMat[i][k] * I_KH[j][k];
      }
      newP[i][j] = acc + Kvec[i] * R * Kvec[j];
    }
  }

  memcpy(P, newP, sizeof(P));
  enforceSymmetry();
  clampDiagonal();
}

static void ekfUpdateDirect(uint8_t peerId, uint16_t rangeMm) {
  if (peerId >= MAX_PEERS) return;
  if (!peerInitialized[peerId]) return;
  if (rangeMm == 0) return;

  float range = (float)rangeMm / 1000.0f;

  float x = X[IDX(peerId, STATE_X)];
  float y = X[IDX(peerId, STATE_Y)];
  float z = X[IDX(peerId, STATE_Z)];

  float pred = sqrtSafe(x*x + y*y + z*z);
  if (pred < 0.01f) pred = 0.01f;

  memset(Hvec, 0, sizeof(Hvec));
  Hvec[IDX(peerId, STATE_X)] = x / pred;
  Hvec[IDX(peerId, STATE_Y)] = y / pred;
  Hvec[IDX(peerId, STATE_Z)] = z / pred;

  float innov = range - pred;
  float R = sigmaRangeDirect * sigmaRangeDirect;

  ekfUpdateScalar(Hvec, R, innov);

  lastDirectRange_mm[peerId] = rangeMm;
  lastUpdateTick[peerId] = xTaskGetTickCount();
}

static void ekfUpdateIndirect(uint8_t peerJ, uint8_t peerK, uint16_t rangeMm) {
  if (peerJ >= MAX_PEERS || peerK >= MAX_PEERS) return;
  if (peerJ == peerK) return;
  if (!peerInitialized[peerJ] || !peerInitialized[peerK]) return;
  if (rangeMm == 0) return;

  float range = (float)rangeMm / 1000.0f;

  float xj = X[IDX(peerJ, STATE_X)];
  float yj = X[IDX(peerJ, STATE_Y)];
  float zj = X[IDX(peerJ, STATE_Z)];

  float xk = X[IDX(peerK, STATE_X)];
  float yk = X[IDX(peerK, STATE_Y)];
  float zk = X[IDX(peerK, STATE_Z)];

  float dx = xk - xj;
  float dy = yk - yj;
  float dz = zk - zj;

  float pred = sqrtSafe(dx*dx + dy*dy + dz*dz);
  if (pred < 0.01f) pred = 0.01f;

  float nx = dx / pred;
  float ny = dy / pred;
  float nz = dz / pred;

  memset(Hvec, 0, sizeof(Hvec));

  // dh/dp_j = -n
  Hvec[IDX(peerJ, STATE_X)] = -nx;
  Hvec[IDX(peerJ, STATE_Y)] = -ny;
  Hvec[IDX(peerJ, STATE_Z)] = -nz;

  // dh/dp_k = +n
  Hvec[IDX(peerK, STATE_X)] = +nx;
  Hvec[IDX(peerK, STATE_Y)] = +ny;
  Hvec[IDX(peerK, STATE_Z)] = +nz;

  float innov = range - pred;
  float R = sigmaRangeIndirect * sigmaRangeIndirect;

  ekfUpdateScalar(Hvec, R, innov);
}

// ============ PUBLIC API ============

void relativeLocalizationInit(void) {
  if (moduleInitialized) return;

  memset(X, 0, sizeof(X));
  memset(P, 0, sizeof(P));

  for (int i = 0; i < MAX_PEERS; i++) {
    peerInitialized[i] = false;
    lastUpdateTick[i] = 0;
    lastPredictTick[i] = 0;
    lastDirectRange_mm[i] = 0;
  }

  connectedMask = 0;
  numConnected = 0;

  moduleInitialized = true;
  DEBUG_PRINT("Relative localization module initialized (max %d peers)\n", MAX_PEERS);
}

void relativeLocalizationResetPeer(uint8_t peerId) {
  if (!moduleInitialized) relativeLocalizationInit();
  resetPeerBlock(peerId);
}

void relativeLocalizationReset(void) {
  if (!moduleInitialized) relativeLocalizationInit();
  for (int i = 0; i < MAX_PEERS; i++) {
    resetPeerBlock(i);
  }
  connectedMask = 0;
  numConnected = 0;
}

void relativeLocalizationUpdate(float dt, bool useIndirect) {
  if (!moduleInitialized) relativeLocalizationInit();

  // Self state
  float selfX_unused, selfY_unused;
  swarmInfoGet(&selfX_unused, &selfY_unused, &selfGz, &selfHeight, &selfVx, &selfVy, &selfVz);
  selfGz = selfGz * M_PI_F / 180.0f;

  bool doIndirect = (useIndirect || (useIndirectMeas != 0));

  // Gather peer inputs
  float peerVx[MAX_PEERS] = {0};
  float peerVy[MAX_PEERS] = {0};
  float peerVz[MAX_PEERS] = {0};
  float peerGz[MAX_PEERS] = {0};

  uint32_t now = xTaskGetTickCount();
  connectedMask = 0;
  numConnected = 0;

  // Read peer packets (skip selfId — a drone has no UWB data for itself)
  for (uint8_t j = 0; j < numPeers && j < MAX_PEERS; j++) {
    if (j == selfId) continue;  // Skip self — state.refresh[selfId] is never set
    uint16_t range = 0;
    float peerX, peerY, peerGzDeg, peerH, vjx, vjy, vjz;

    if (twrGetSwarmInfo(j, &range, &peerX, &peerY, &peerGzDeg, &peerH, &vjx, &vjy, &vjz)) {

      // Debug: log raw inputs
      dbgRawRange_mm[j] = range;
      dbgPeerVx[j]      = vjx;
      dbgPeerVy[j]      = vjy;
      dbgPeerGz[j]      = peerGzDeg * M_PI_F / 180.0f;
      dbgPeerH[j]       = peerH;
      dbgPeerValid[j]   = 1;

      if (!peerInitialized[j]) {
        float initDist = (range > 100) ? ((float)range / 1000.0f) : defaultInitX;
        float initZ = peerH - selfHeight;
        initPeer(j, initDist, defaultInitY, initZ, defaultInitPsi);
      }

      peerVx[j] = vjx;
      peerVy[j] = vjy;
      peerVz[j] = vjz;
      peerGz[j] = peerGzDeg * M_PI_F / 180.0f;

      connectedMask |= (1 << j);
      numConnected++;

      lastUpdateTick[j] = now;
      lastDirectRange_mm[j] = range;

    } else {
      dbgPeerValid[j] = 0;  // No fresh UWB data for this peer
      if (peerInitialized[j] && (now - lastUpdateTick[j]) < PEER_TIMEOUT_TICKS) {
        connectedMask |= (1 << j);
        numConnected++;
      }
    }
  }

  // Predict
  ekfPredictAll(peerVx, peerVy, peerVz, peerGz, dt);

  // Direct updates
  for (uint8_t j = 0; j < numPeers && j < MAX_PEERS; j++) {
    if (j == selfId) continue;  // Skip self
    if (!peerInitialized[j]) continue;
    uint16_t r = lastDirectRange_mm[j];
    if (r > 0) {
      // Debug: compute innovation before update
      float x = X[IDX(j, STATE_X)];
      float y = X[IDX(j, STATE_Y)];
      float z = X[IDX(j, STATE_Z)];
      float pred = sqrtSafe(x*x + y*y + z*z);
      if (pred < 0.01f) pred = 0.01f;
      dbgPredRange[j] = pred;
      dbgInnov[j]     = ((float)r / 1000.0f) - pred;

      ekfUpdateDirect(j, r);
    }
  }

  // Indirect updates (only between two OTHER peers, not self)
  if (doIndirect) {
    for (uint8_t j = 0; j < numPeers && j < MAX_PEERS; j++) {
      if (j == selfId) continue;  // Skip self
      for (uint8_t k = j + 1; k < numPeers && k < MAX_PEERS; k++) {
        if (k == selfId) continue;  // Skip self
        uint16_t rjk = twrGetIndirectDist(j, k);
        // Debug: log key indirect ranges
        if      (j == 0 && k == 1) dbgIndRange01_mm = rjk;
        else if (j == 0 && k == 2) dbgIndRange02_mm = rjk;
        else if (j == 1 && k == 2) dbgIndRange12_mm = rjk;
        if (rjk > 0) {
          ekfUpdateIndirect(j, k, rjk);
        }
      }
    }
  }

  // Logging
  for (uint8_t j = 0; j < MAX_PEERS; j++) {
    if (j < numPeers && peerInitialized[j]) {
      logX[j] = X[IDX(j, STATE_X)];
      logY[j] = X[IDX(j, STATE_Y)];
      logZ[j] = X[IDX(j, STATE_Z)];
      logPsi[j] = X[IDX(j, STATE_PSI)];
      logDist[j] = (float)lastDirectRange_mm[j] / 1000.0f;
      logPxx[j] = P[IDX(j, STATE_X)][IDX(j, STATE_X)];
      logPyy[j] = P[IDX(j, STATE_Y)][IDX(j, STATE_Y)];
    } else {
      logX[j] = 0.0f;
      logY[j] = 0.0f;
      logZ[j] = 0.0f;
      logPsi[j] = 0.0f;
      logDist[j] = 0.0f;
      logPxx[j] = 0.0f;
      logPyy[j] = 0.0f;
    }
  }
}

bool relativeLocalizationGetState(uint8_t peerId, float *state) {
  if (peerId >= MAX_PEERS || state == NULL) return false;
  if (!peerInitialized[peerId]) return false;

  state[0] = X[IDX(peerId, STATE_X)];
  state[1] = X[IDX(peerId, STATE_Y)];
  state[2] = X[IDX(peerId, STATE_Z)];
  state[3] = X[IDX(peerId, STATE_PSI)];

  return (connectedMask & (1 << peerId)) != 0;
}

bool relativeLocalizationGetCovariance(uint8_t peerId, float *cov) {
  if (peerId >= MAX_PEERS || cov == NULL) return false;
  if (!peerInitialized[peerId]) return false;

  cov[0] = P[IDX(peerId, STATE_X)][IDX(peerId, STATE_X)];
  cov[1] = P[IDX(peerId, STATE_Y)][IDX(peerId, STATE_Y)];
  cov[2] = P[IDX(peerId, STATE_Z)][IDX(peerId, STATE_Z)];
  cov[3] = P[IDX(peerId, STATE_PSI)][IDX(peerId, STATE_PSI)];

  return true;
}

bool relativeLocalizationIsConnected(void) {
  return numConnected > 0;
}

uint8_t relativeLocalizationGetConnectedMask(void) {
  return connectedMask;
}

uint8_t relativeLocalizationGetNumConnected(void) {
  return numConnected;
}

void relativeLocalizationSetSelfId(uint8_t id) {
  if (id < MAX_PEERS) {
    selfId = id;
    // Make sure self-slot is never initialized
    if (peerInitialized[id]) {
      resetPeerBlock(id);
    }
  }
}

void relativeLocalizationSetNumPeers(uint8_t n) {
  if (n > MAX_PEERS) n = MAX_PEERS;
  numPeers = n;
}

void relativeLocalizationSetUseIndirect(bool enable) {
  useIndirectMeas = enable ? 1 : 0;
}

// ============ LOG VARIABLES ============

LOG_GROUP_START(relLoc)
// Peer 0
LOG_ADD(LOG_FLOAT, x0, &logX[0])
LOG_ADD(LOG_FLOAT, y0, &logY[0])
LOG_ADD(LOG_FLOAT, z0, &logZ[0])
LOG_ADD(LOG_FLOAT, psi0, &logPsi[0])
LOG_ADD(LOG_FLOAT, d0, &logDist[0])
LOG_ADD(LOG_FLOAT, Px0, &logPxx[0])
LOG_ADD(LOG_FLOAT, Py0, &logPyy[0])
// Peer 1
LOG_ADD(LOG_FLOAT, x1, &logX[1])
LOG_ADD(LOG_FLOAT, y1, &logY[1])
LOG_ADD(LOG_FLOAT, z1, &logZ[1])
LOG_ADD(LOG_FLOAT, psi1, &logPsi[1])
LOG_ADD(LOG_FLOAT, d1, &logDist[1])
LOG_ADD(LOG_FLOAT, Px1, &logPxx[1])
LOG_ADD(LOG_FLOAT, Py1, &logPyy[1])
// Peer 2
LOG_ADD(LOG_FLOAT, x2, &logX[2])
LOG_ADD(LOG_FLOAT, y2, &logY[2])
LOG_ADD(LOG_FLOAT, z2, &logZ[2])
LOG_ADD(LOG_FLOAT, psi2, &logPsi[2])
LOG_ADD(LOG_FLOAT, d2, &logDist[2])
LOG_ADD(LOG_FLOAT, Px2, &logPxx[2])
LOG_ADD(LOG_FLOAT, Py2, &logPyy[2])
// Connection
LOG_ADD(LOG_UINT8, connMask, &connectedMask)
LOG_ADD(LOG_UINT8, numConn, &numConnected)
LOG_GROUP_STOP(relLoc)

// Self state
LOG_GROUP_START(relSelf)
LOG_ADD(LOG_FLOAT, vx, &selfVx)
LOG_ADD(LOG_FLOAT, vy, &selfVy)
LOG_ADD(LOG_FLOAT, vz, &selfVz)
LOG_ADD(LOG_FLOAT, gz, &selfGz)
LOG_ADD(LOG_FLOAT, h, &selfHeight)
LOG_GROUP_STOP(relSelf)

// Debug: raw EKF inputs and innovation for diagnosing EKF issues
LOG_GROUP_START(relDbg)
// Is UWB data arriving for each peer?
LOG_ADD(LOG_UINT8,  valid0,   &dbgPeerValid[0])
LOG_ADD(LOG_UINT8,  valid1,   &dbgPeerValid[1])
LOG_ADD(LOG_UINT8,  valid2,   &dbgPeerValid[2])
// Raw UWB ranges (mm) - what comes out of twrGetSwarmInfo
LOG_ADD(LOG_UINT16, rng0mm,   &dbgRawRange_mm[0])
LOG_ADD(LOG_UINT16, rng1mm,   &dbgRawRange_mm[1])
LOG_ADD(LOG_UINT16, rng2mm,   &dbgRawRange_mm[2])
// Indirect ranges (mm) between peer pairs
LOG_ADD(LOG_UINT16, iRng01,   &dbgIndRange01_mm)
LOG_ADD(LOG_UINT16, iRng02,   &dbgIndRange02_mm)
LOG_ADD(LOG_UINT16, iRng12,   &dbgIndRange12_mm)
// EKF predicted range (m) - should converge to match raw range
LOG_ADD(LOG_FLOAT,  pred0,    &dbgPredRange[0])
LOG_ADD(LOG_FLOAT,  pred1,    &dbgPredRange[1])
LOG_ADD(LOG_FLOAT,  pred2,    &dbgPredRange[2])
// Innovation = measured - predicted (m), should trend toward 0
LOG_ADD(LOG_FLOAT,  innov0,   &dbgInnov[0])
LOG_ADD(LOG_FLOAT,  innov1,   &dbgInnov[1])
LOG_ADD(LOG_FLOAT,  innov2,   &dbgInnov[2])
// Peer velocities as the EKF sees them
LOG_ADD(LOG_FLOAT,  pvx0,     &dbgPeerVx[0])
LOG_ADD(LOG_FLOAT,  pvy0,     &dbgPeerVy[0])
LOG_ADD(LOG_FLOAT,  pGz0,     &dbgPeerGz[0])
LOG_ADD(LOG_FLOAT,  pH0,      &dbgPeerH[0])
LOG_ADD(LOG_FLOAT,  pvx1,     &dbgPeerVx[1])
LOG_ADD(LOG_FLOAT,  pvy1,     &dbgPeerVy[1])
LOG_ADD(LOG_FLOAT,  pGz1,     &dbgPeerGz[1])
LOG_ADD(LOG_FLOAT,  pH1,      &dbgPeerH[1])
LOG_ADD(LOG_FLOAT,  pvx2,     &dbgPeerVx[2])
LOG_ADD(LOG_FLOAT,  pvy2,     &dbgPeerVy[2])
LOG_ADD(LOG_FLOAT,  pGz2,     &dbgPeerGz[2])
LOG_ADD(LOG_FLOAT,  pH2,      &dbgPeerH[2])
LOG_GROUP_STOP(relDbg)

// ============ PARAMETERS ============

PARAM_GROUP_START(relLoc)
PARAM_ADD(PARAM_UINT8, numPeers, &numPeers)
PARAM_ADD(PARAM_UINT8, useInd, &useIndirectMeas)

PARAM_ADD(PARAM_FLOAT, sigVxy, &sigmaVxy)
PARAM_ADD(PARAM_FLOAT, sigVz, &sigmaVz)
PARAM_ADD(PARAM_FLOAT, sigYaw, &sigmaYawRate)

PARAM_ADD(PARAM_FLOAT, sigRngD, &sigmaRangeDirect)
PARAM_ADD(PARAM_FLOAT, sigRngI, &sigmaRangeIndirect)

PARAM_ADD(PARAM_FLOAT, initPxy, &initCovXY)
PARAM_ADD(PARAM_FLOAT, initPz, &initCovZ)
PARAM_ADD(PARAM_FLOAT, initPpsi, &initCovPsi)

PARAM_ADD(PARAM_FLOAT, defX, &defaultInitX)
PARAM_ADD(PARAM_FLOAT, defY, &defaultInitY)
PARAM_ADD(PARAM_FLOAT, defZ, &defaultInitZ)
PARAM_ADD(PARAM_FLOAT, defPsi, &defaultInitPsi)
PARAM_GROUP_STOP(relLoc)
