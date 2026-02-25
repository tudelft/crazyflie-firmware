/**
 * Relative Localization Demo App
 *
 * This app demonstrates the relative localization EKF using UWB data
 * from the loco deck (lpsTwrTag.c). It estimates the relative position
 * of other drones in the swarm from this drone's perspective.
 *
 * The EKF runs at 100Hz and can use:
 *   - Direct measurements: UWB range from self to each peer
 *   - Indirect measurements: UWB range between other peers (optional)
 *
 * Usage:
 *   1. Flash this app to all drones in the swarm
 *   2. Set swarm.size parameter to number of drones
 *   3. Set relApp.numPeers to number of peers to track (max 3 right now)
 *   4. Optionally enable indirect measurements: relApp.useInd = 1
 *   5. Monitor relLoc.x0, y0, z0, etc. in cfclient
 *
 * Build: make -j$(nproc) APP=app_relative_localization
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "RELLOCAPP"
#include "debug.h"

#include "log.h"
#include "param.h"
#include "configblock.h"
#include "estimator/relative_localization.h"
#include "lpsTwrTag.h"

// ============================================================================
// Configuration
// ============================================================================

#define EKF_UPDATE_RATE_HZ     100
#define EKF_UPDATE_PERIOD_MS   (1000 / EKF_UPDATE_RATE_HZ)
#define EKF_DT                (1.0f / EKF_UPDATE_RATE_HZ)

// Keep this aligned with relative_localization.c MAX_PEERS
#define MAX_PEERS_TO_TRACK 3

// ============================================================================
// App State
// ============================================================================

static uint8_t selfId = 0;
static uint8_t numPeersToTrack = 3;  // Total swarm size (including self); EKF skips selfId
static uint8_t useIndirectMeasurements = 0;
static uint8_t appEnabled = 1;

// Computed relative states for logging
static float relX[MAX_PEERS_TO_TRACK];
static float relY[MAX_PEERS_TO_TRACK];
static float relZ[MAX_PEERS_TO_TRACK];
static float relPsi[MAX_PEERS_TO_TRACK];
static float relDist[MAX_PEERS_TO_TRACK];  // Euclidean distance to peer

// Status
static uint8_t ekfConnected = 0;
static uint8_t ekfNumConnected = 0;
static uint32_t ekfUpdateCount = 0;

// ============================================================================
// Timestamp helper
// ============================================================================
static inline float getTimestampSec(void) {
  return (float)(xTaskGetTickCount() * portTICK_PERIOD_MS) / 1000.0f;
}

// ============================================================================
// Main App Entry Point
// ============================================================================

void appMain(void) {
  // Get self ID from radio address (same as lpsTwrTag does)
  selfId = (uint8_t)(configblockGetRadioAddress() & 0xF);

  DEBUG_PRINT("Relative Localization App started\n");
  DEBUG_PRINT("Self ID: %u\n", selfId);
  DEBUG_PRINT("EKF update rate: %d Hz\n", EKF_UPDATE_RATE_HZ);

  // Wait for system to stabilize
  vTaskDelay(pdMS_TO_TICKS(2000));

  // Initialize the relative localization EKF
  relativeLocalizationInit();

  // Tell EKF our own ID so it skips the self-slot
  // (a drone never has UWB data about itself)
  relativeLocalizationSetSelfId(selfId);

  // Clamp configuration to EKF capacity
  if (numPeersToTrack > MAX_PEERS_TO_TRACK) {
    numPeersToTrack = MAX_PEERS_TO_TRACK;
  }

  // Configure EKF
  relativeLocalizationSetNumPeers(numPeersToTrack);
  relativeLocalizationSetUseIndirect(useIndirectMeasurements != 0);

  DEBUG_PRINT("EKF initialized, tracking %u peers, indirect=%u\n",
              numPeersToTrack, useIndirectMeasurements);

  // Initialize state arrays
  memset(relX, 0, sizeof(relX));
  memset(relY, 0, sizeof(relY));
  memset(relZ, 0, sizeof(relZ));
  memset(relPsi, 0, sizeof(relPsi));
  memset(relDist, 0, sizeof(relDist));

  TickType_t lastWakeTime = xTaskGetTickCount();

  // Main loop
  while (1) {
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(EKF_UPDATE_PERIOD_MS));

    if (!appEnabled) {
      continue;
    }

    // Clamp live params to EKF capacity
    if (numPeersToTrack > MAX_PEERS_TO_TRACK) {
      numPeersToTrack = MAX_PEERS_TO_TRACK;
    }

    // Update configuration if changed via params
    relativeLocalizationSetNumPeers(numPeersToTrack);
    relativeLocalizationSetUseIndirect(useIndirectMeasurements != 0);

    // Run one EKF update cycle
    // Note: relative_localization.c now correctly treats this flag as a per-call toggle too
    relativeLocalizationUpdate(EKF_DT, useIndirectMeasurements != 0);

    ekfUpdateCount++;

    // Update status
    ekfConnected = relativeLocalizationIsConnected() ? 1 : 0;
    ekfNumConnected = relativeLocalizationGetNumConnected();

    // Read estimated relative states for each peer
    for (uint8_t j = 0; j < numPeersToTrack && j < MAX_PEERS_TO_TRACK; j++) {
      float state[4];
      if (relativeLocalizationGetState(j, state)) {
        relX[j] = state[0];
        relY[j] = state[1];
        relZ[j] = state[2];
        relPsi[j] = state[3];

        // Compute Euclidean distance
        relDist[j] = sqrtf(state[0]*state[0] + state[1]*state[1] + state[2]*state[2]);
      }
    }

    // Periodic debug output (every 5 seconds)
    static uint32_t lastDebugTick = 0;
    uint32_t now = xTaskGetTickCount();
    if (now - lastDebugTick > pdMS_TO_TICKS(5000)) {
      lastDebugTick = now;

      DEBUG_PRINT("[%.1f] EKF status: connected=%u, numPeers=%u, updates=%lu\n",
                  (double)getTimestampSec(), ekfConnected, ekfNumConnected,
                  (unsigned long)ekfUpdateCount);

      for (uint8_t j = 0; j < numPeersToTrack && j < MAX_PEERS_TO_TRACK; j++) {
        if (relativeLocalizationGetConnectedMask() & (1 << j)) {
          DEBUG_PRINT("  Peer %u: (%.2f, %.2f, %.2f) psi=%.1f deg, dist=%.2f m\n",
                      j, (double)relX[j], (double)relY[j], (double)relZ[j],
                      (double)(relPsi[j] * 180.0f / 3.14159f), (double)relDist[j]);
        }
      }
    }
  }
}

// ============================================================================
// Log Variables
// ============================================================================

LOG_GROUP_START(relApp)
// Status
LOG_ADD(LOG_UINT8, enabled, &appEnabled)
LOG_ADD(LOG_UINT8, connected, &ekfConnected)
LOG_ADD(LOG_UINT8, numConn, &ekfNumConnected)
LOG_ADD(LOG_UINT32, updates, &ekfUpdateCount)
// Peer 0
LOG_ADD(LOG_FLOAT, x0, &relX[0])
LOG_ADD(LOG_FLOAT, y0, &relY[0])
LOG_ADD(LOG_FLOAT, z0, &relZ[0])
LOG_ADD(LOG_FLOAT, psi0, &relPsi[0])
LOG_ADD(LOG_FLOAT, dist0, &relDist[0])
// Peer 1
LOG_ADD(LOG_FLOAT, x1, &relX[1])
LOG_ADD(LOG_FLOAT, y1, &relY[1])
LOG_ADD(LOG_FLOAT, z1, &relZ[1])
LOG_ADD(LOG_FLOAT, psi1, &relPsi[1])
LOG_ADD(LOG_FLOAT, dist1, &relDist[1])
// Peer 2
LOG_ADD(LOG_FLOAT, x2, &relX[2])
LOG_ADD(LOG_FLOAT, y2, &relY[2])
LOG_ADD(LOG_FLOAT, z2, &relZ[2])
LOG_ADD(LOG_FLOAT, psi2, &relPsi[2])
LOG_ADD(LOG_FLOAT, dist2, &relDist[2])
LOG_GROUP_STOP(relApp)

// ============================================================================
// Parameters
// ============================================================================

PARAM_GROUP_START(relApp)
PARAM_ADD(PARAM_UINT8, enabled, &appEnabled)
// Total swarm size including self (max 3); EKF skips selfId
PARAM_ADD(PARAM_UINT8, numPeers, &numPeersToTrack)
// Use indirect measurements (0=direct only, 1=direct+indirect)
PARAM_ADD(PARAM_UINT8, useInd, &useIndirectMeasurements)
PARAM_GROUP_STOP(relApp)
