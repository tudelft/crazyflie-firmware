/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* uwb_twr_anchor.c: Uwb two way ranging anchor implementation */


#include <string.h>
#include "lpsTwrTag.h"
#include "log.h"
#include "param.h"
#include "physicalConstants.h"
#include "FreeRTOS.h"
#include "task.h"
#include "configblock.h"
#include "estimator.h"
#include "swarm_info.h"

#include "debug.h"

// Forward declaration
static uint8_t buildLocalAuxMask(void);

// Local CPPM AUX var IDs (0xFFFF = invalid)
static logVarId_t idAux0 = (logVarId_t)0xFFFF;
static logVarId_t idAux1 = (logVarId_t)0xFFFF;
static logVarId_t idAux2 = (logVarId_t)0xFFFF;
static logVarId_t idAux3 = (logVarId_t)0xFFFF;

// Build local AUX mask from cppm.aux0..aux3 (low-active < 1400 => bit set)
static uint8_t buildLocalAuxMask(void)
{
  if (!logVarIdIsValid(idAux0)) {
    idAux0 = logGetVarId("cppm", "aux0");
    idAux1 = logGetVarId("cppm", "aux1");
    idAux2 = logGetVarId("cppm", "aux2");
    idAux3 = logGetVarId("cppm", "aux3");
  }
  uint8_t mask = 0;
  if (logVarIdIsValid(idAux0)) { uint16_t v = logGetUint(idAux0); if (v > 0 && v < 1400) mask |= 1u << 0; }
  if (logVarIdIsValid(idAux1)) { uint16_t v = logGetUint(idAux1); if (v > 0 && v < 1400) mask |= 1u << 1; }
  if (logVarIdIsValid(idAux2)) { uint16_t v = logGetUint(idAux2); if (v > 0 && v < 1400) mask |= 1u << 2; }
  if (logVarIdIsValid(idAux3)) { uint16_t v = logGetUint(idAux3); if (v > 0 && v < 1400) mask |= 1u << 3; }
  return mask;
}

#define basicAddr 0xbccf851300000000
static uint8_t selfID;
static locoAddress_t selfAddress;

// Swarm size runtime control
#define MAX_SWARM_SIZE (LOCODECK_NR_OF_TWR_ANCHORS + 1)

// Swarm size: total number of crazyflies in the swarm (including self)
static uint8_t swarmSize = 3;

// Only this drone ID is allowed to publish AUX (default: 1)
static uint8_t auxPublisherId = 1;

static inline uint8_t effectiveSwarmSize(void) {
  uint8_t n = swarmSize;
  if (n < 1) n = 1;
  if (n > MAX_SWARM_SIZE) n = MAX_SWARM_SIZE;
  return n;
}

// Config
static lpsTwrAlgoOptions_t defaultOptions = {
   .tagAddress = 0xbccf000000000008,
   .antennaDelay = LOCODECK_ANTENNA_DELAY,
   .rangingFailedThreshold = 6,
   .combinedAnchorPositionOk = false,
};

static lpsTwrAlgoOptions_t* options = &defaultOptions;

typedef struct
{
  uint16_t distance[MAX_SWARM_SIZE];
  float x[MAX_SWARM_SIZE];
  float y[MAX_SWARM_SIZE];
  float gz[MAX_SWARM_SIZE];
  float h[MAX_SWARM_SIZE];
  float vx[MAX_SWARM_SIZE];
  float vy[MAX_SWARM_SIZE];
  bool refresh[MAX_SWARM_SIZE];
  bool keep_flying;
  int failedRanging[LOCODECK_NR_OF_TWR_ANCHORS];
  uint8_t auxMask[MAX_SWARM_SIZE];
  uint8_t aux[4];
  // Indirect distance matrix: indirectDist[i][j] = drone i's distance to drone j
  uint16_t indirectDist[MAX_SWARM_SIZE][MAX_SWARM_SIZE];
} swarmInfo_t;
static swarmInfo_t state;

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

static packet_t txPacket;
static bool rangingOk;

// Communication logic between each UWB
static bool current_mode_trans;
static uint8_t current_receiveID;

#if (MAX_SWARM_SIZE > 2)
static bool checkTurn;
static uint32_t checkTurnTick = 0;
#endif

// Timeout and peer tracking for robust communication
static uint32_t lastSuccessfulRanging[MAX_SWARM_SIZE];
static uint32_t consecutiveTimeouts = 0;
#define MAX_CONSECUTIVE_TIMEOUTS 3
#define PEER_TIMEOUT_MS 5000  // Consider peer inactive after 5 seconds

// ID-dependent timeout for starting communication when no messages are overheard
static uint32_t lastOverheardMessage = 0;  // Track when we last heard ANY message
static bool hasOverheardAnyMessage = false;  // Track if we've ever heard a message
static uint32_t lastTimeoutStart = 0;  // Track when we last started due to timeout
#define TIMEOUT_BASE_MS 1000  // Base timeout period 
#define MAX_STARTUP_TIMEOUT_MS 10000  // Maximum time to wait before forcing startup
#define MIN_TIMEOUT_START_INTERVAL_MS 500  // Minimum interval between timeout-based starts

// Median filter for distance ranging (size=3)
typedef struct
{
  uint16_t distance_history[3];
  uint8_t index_inserting;
} median_data_t;
static median_data_t median_data[MAX_SWARM_SIZE];

// Ranging rate debug tracking
static uint32_t lastRateTick = 0;
static uint32_t successfulRangeCount = 0;

static uint16_t median_filter_3(uint16_t *data)
{
  uint16_t middle;
  if ((data[0] <= data[1]) && (data[0] <= data[2]))
  {
    middle = (data[1] <= data[2]) ? data[1] : data[2];
  }
  else if ((data[1] <= data[0]) && (data[1] <= data[2]))
  {
    middle = (data[0] <= data[2]) ? data[0] : data[2];
  }
  else
  {
    middle = (data[0] <= data[1]) ? data[0] : data[1];
  }
  return middle;
}
#define ABS(a) ((a) > 0 ? (a) : -(a))

// Helper function to select a random peer from the swarm (excluding self)
static uint8_t selectRandomPeer(void)
{
  uint32_t tick = xTaskGetTickCount();
  uint8_t numPeers = swarmSize - 1;

  if (numPeers == 0) {
    return selfID; // No peers available
  }

  // Simple random selection based on tick count
  uint8_t peerIndex = tick % numPeers;
  uint8_t selectedPeer = peerIndex;

  // Skip self ID
  if (selectedPeer >= selfID) {
    selectedPeer++;
  }

  return selectedPeer;
}

// Check if this node should start communication due to timeout
static bool shouldStartDueToTimeout(void)
{
  uint32_t currentTick = xTaskGetTickCount();
  
  // Prevent too frequent timeout-based starts
  if (currentTick < lastTimeoutStart + MIN_TIMEOUT_START_INTERVAL_MS) {
    return false;
  }
  
  uint32_t timeSinceLastMessage = currentTick - lastOverheardMessage;
  uint32_t idBasedTimeout = (selfID + 1) * TIMEOUT_BASE_MS;
  
  return (timeSinceLastMessage > idBasedTimeout);
}

static uint8_t selectNextPeer(void)
{
  return selectRandomPeer();
}

static void txcallback(dwDevice_t *dev)
{
  dwTime_t departure;
  dwGetTransmitTimestamp(dev, &departure);
  departure.full += (options->antennaDelay / 2);

  if (current_mode_trans)
  {
    switch (txPacket.payload[0]) {
    case LPS_TWR_POLL:
      poll_tx = departure;
      break;
    case LPS_TWR_FINAL:
      final_tx = departure;
      break;
    case LPS_TWR_REPORT + 1:
      current_mode_trans = false;
      dwIdle(dev);
      dwSetReceiveWaitTimeout(dev, 10000);
      dwNewReceive(dev);
      dwSetDefaults(dev);
      dwStartReceive(dev);

      // Plan next peer randomly (robust vs. failures)
      current_receiveID = selectNextPeer();

      checkTurn = true;
      checkTurnTick = xTaskGetTickCount();
      break;
    }
  }
  else
  {
    switch (txPacket.payload[0])
    {
    case LPS_TWR_ANSWER:
      answer_tx = departure;
      break;
    case LPS_TWR_REPORT:
      break;
    }
  }
}

static void rxcallback(dwDevice_t *dev) {
  dwTime_t arival = { .full=0 };
  int dataLength = dwGetDataLength(dev);

  if (dataLength == 0) return;

  packet_t rxPacket;
  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  // Track that we've overheard a message (any message, not just for us)
  uint32_t currentTick = xTaskGetTickCount();
  lastOverheardMessage = currentTick;
  hasOverheardAnyMessage = true;

  uint8_t sourceId = (uint8_t)(rxPacket.sourceAddress & 0xFF);
  lastSuccessfulRanging[sourceId] = xTaskGetTickCount();
  if (rxPacket.destAddress != selfAddress) {
    if (!current_mode_trans)
    {
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
    }
    return;
  }

  txPacket.destAddress = rxPacket.sourceAddress;
  txPacket.sourceAddress = rxPacket.destAddress;

  if (current_mode_trans)
  {
    switch (rxPacket.payload[LPS_TWR_TYPE])
    {
    case LPS_TWR_ANSWER:

      txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_FINAL;
      txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];

      dwGetReceiveTimestamp(dev, &arival);
      arival.full -= (options->antennaDelay / 2);
      answer_rx = arival;

      dwNewTransmit(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);

      break;
    case LPS_TWR_REPORT:
    {
      lpsTwrTagReportPayload_t *report = (lpsTwrTagReportPayload_t *)(rxPacket.payload+2);
      double tround1, treply1, treply2, tround2, tprop_ctn, tprop;

      memcpy(&poll_rx, &report->pollRx, 5);
      memcpy(&answer_tx, &report->answerTx, 5);
      memcpy(&final_rx, &report->finalRx, 5);

      tround1 = answer_rx.low32 - poll_tx.low32;
      treply1 = answer_tx.low32 - poll_rx.low32;
      tround2 = final_rx.low32 - answer_tx.low32;
      treply2 = final_tx.low32 - answer_rx.low32;

      tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);

      tprop = tprop_ctn / LOCODECK_TS_FREQ;
      uint16_t calcDist = (uint16_t)(1000 * SPEED_OF_LIGHT * tprop);
      uint8_t n = effectiveSwarmSize();
      if (calcDist != 0 && current_receiveID < n)
      {
        uint16_t medianDist = median_filter_3(median_data[current_receiveID].distance_history);
        if (ABS(medianDist - calcDist) > 500)
          state.distance[current_receiveID] = medianDist;
        else
          state.distance[current_receiveID] = calcDist;
        median_data[current_receiveID].index_inserting++;
        if (median_data[current_receiveID].index_inserting == 3)
          median_data[current_receiveID].index_inserting = 0;
        median_data[current_receiveID].distance_history[median_data[current_receiveID].index_inserting] = calcDist;
        rangingOk = true;

        // Update last successful ranging timestamp
        lastSuccessfulRanging[current_receiveID] = xTaskGetTickCount();
        consecutiveTimeouts = 0; // Reset timeout counter on successful ranging

        state.x[current_receiveID] = report->selfX;
        state.y[current_receiveID] = report->selfY;
        state.gz[current_receiveID] = report->selfGz;
        state.h[current_receiveID] = report->selfh;
        state.vx[current_receiveID] = report->selfVx;
        state.vy[current_receiveID] = report->selfVy;
        if (current_receiveID == 0)
          state.keep_flying = report->keep_flying;
        // Store peer AUX mask
        state.auxMask[current_receiveID] = report->auxMask;
        // Only accept AUX from the designated publisher
        if (current_receiveID == auxPublisherId) {
          for (int ch = 0; ch < 4; ch++) {
            state.aux[ch] = (report->auxMask >> ch) & 0x1;
          }
        }
        state.refresh[current_receiveID] = true;

        // Store indirect distances from this peer
        for (int j = 0; j < MAX_SWARM_SIZE; j++) {
          state.indirectDist[current_receiveID][j] = report->distToPeers[j];
        }

        if (isAnchor == 0)
        {
          distanceMeasurement_t dist;
          dist.distance = (float)state.distance[current_receiveID] / 1000.0f;
          dist.x = state.x[current_receiveID];
          dist.y = state.y[current_receiveID];
          dist.z = state.h[current_receiveID];
          dist.anchorId = current_receiveID;
          dist.stdDev = 0.3; // Make this depend on type of other crazyflie
          // DEBUG_PRINT("Tag got distance to Anchor %u: %.2f m\n", current_receiveID, (double)dist.distance);
          // estimatorEnqueueDistance(&dist);
        }
      }

      lpsTwrTagReportPayload_t *report2 = (lpsTwrTagReportPayload_t *)(txPacket.payload + 2);
      txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT + 1;
      txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
      report2->reciprocalDistance = calcDist;

      // first load data into local variable to prevent memory misalignment warning
      float selfX2 = report2->selfX;
      float selfY2 = report2->selfY;
      float selfGz2 = report2->selfGz;
      float selfh2 = report2->selfh;
      float selfVx2 = report2->selfVx;
      float selfVy2 = report2->selfVy;

      // Fetch latest self state to include in the report
      swarmInfoGet(&selfX2, &selfY2, &selfGz2, &selfh2, &selfVx2, &selfVy2);

      report2->selfX = selfX2;
      report2->selfY = selfY2;
      report2->selfGz = selfGz2;
      report2->selfh = selfh2;
      report2->selfVx = selfVx2;
      report2->selfVy = selfVy2;
      
      report2->keep_flying = state.keep_flying;
      uint8_t localMask2 = (selfID == auxPublisherId) ? buildLocalAuxMask() : 0;
      report2->auxMask = localMask2;
      if (selfID == auxPublisherId) {
        for (int ch = 0; ch < 4; ch++) {
          state.aux[ch] = (localMask2 >> ch) & 0x1;
        }
      }

      // Include our distances to all peers (indirect distances)
      for (int j = 0; j < MAX_SWARM_SIZE; j++) {
        report2->distToPeers[j] = state.distance[j];
      }

      dwNewTransmit(dev);
      dwSetData(dev, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2 + sizeof(lpsTwrTagReportPayload_t));
      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);
      break;
    }
    }
  }
  else
  {
    switch (rxPacket.payload[LPS_TWR_TYPE])
    {
      case LPS_TWR_POLL:
      {
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_ANSWER;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        dwGetReceiveTimestamp(dev, &arival);
        arival.full -= (options->antennaDelay / 2);
        poll_rx = arival;
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
      case LPS_TWR_FINAL:
      {
        lpsTwrTagReportPayload_t *report = (lpsTwrTagReportPayload_t *)(txPacket.payload + 2);
        dwGetReceiveTimestamp(dev, &arival);
        arival.full -= (options->antennaDelay / 2);
        final_rx = arival;
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        memcpy(&report->pollRx, &poll_rx, 5);
        memcpy(&report->answerTx, &answer_tx, 5);
        memcpy(&report->finalRx, &final_rx, 5);
        
        // first load data into local variable to prevent memory misalignment warning
        float selfX= report->selfX;
        float selfY = report->selfY;
        float selfGz = report->selfGz;
        float selfh = report->selfh;
        float selfVx = report->selfVx;
        float selfVy = report->selfVy;

        // Fetch latest self state to include in the report
        swarmInfoGet(&selfX, &selfY, &selfGz, &selfh, &selfVx, &selfVy);
        report->selfX = selfX;
        report->selfY = selfY;
        report->selfGz = selfGz;
        report->selfh = selfh;
        report->selfVx = selfVx;
        report->selfVy = selfVy;

        report->keep_flying = state.keep_flying;
        report->auxMask = buildLocalAuxMask();

        // Include our distances to all peers (indirect distances)
        for (int j = 0; j < MAX_SWARM_SIZE; j++) {
          report->distToPeers[j] = state.distance[j];
        }

        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2 + sizeof(lpsTwrTagReportPayload_t));
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
      case (LPS_TWR_REPORT + 1):
      {
        lpsTwrTagReportPayload_t *report2 = (lpsTwrTagReportPayload_t *)(rxPacket.payload + 2);
        uint8_t rangingID = (uint8_t)(rxPacket.sourceAddress & 0xFF);
        uint8_t n = effectiveSwarmSize();
        if ((report2->reciprocalDistance) != 0 && rangingID < n) // IS THE SECOND IF STATEMENT REALLY NECESSARY?
        {
          // received distance has large noise
          uint16_t calcDist = report2->reciprocalDistance;
          uint16_t medianDist = median_filter_3(median_data[rangingID].distance_history);
          if (ABS(medianDist - calcDist) > 500)
            state.distance[rangingID] = medianDist;
          else
            state.distance[rangingID] = calcDist;
          median_data[rangingID].index_inserting++;
          if (median_data[rangingID].index_inserting == 3)
            median_data[rangingID].index_inserting = 0;
          median_data[rangingID].distance_history[median_data[rangingID].index_inserting] = calcDist;
          state.x[rangingID] = report2->selfX;
          state.y[rangingID] = report2->selfY;
          state.gz[rangingID] = report2->selfGz;
          state.h[rangingID] = report2->selfh;
          state.vx[rangingID] = report2->selfVx;
          state.vy[rangingID] = report2->selfVy;
          if (rangingID == 0)
            state.keep_flying = report2->keep_flying;
          // Store peer AUX mask
          state.auxMask[rangingID] = report2->auxMask;
          // Only accept AUX from the designated publisher
          if (rangingID == auxPublisherId) {
            for (int ch = 0; ch < 4; ch++) {
              state.aux[ch] = (report2->auxMask >> ch) & 0x1;
            }
          }
          state.refresh[rangingID] = true;

          // Store indirect distances from this peer
          for (int j = 0; j < MAX_SWARM_SIZE; j++) {
            state.indirectDist[rangingID][j] = report2->distToPeers[j];
          }

          if (isAnchor == 0)
          {
            distanceMeasurement_t dist;
            dist.distance = (float)state.distance[rangingID] / 1000.0f;
            dist.x = state.x[rangingID];
            dist.y = state.y[rangingID];
            dist.z = state.h[rangingID];
            dist.anchorId = rangingID;
            dist.stdDev = 0.25; // Make this depend on type of other crazyflie
            // DEBUG_PRINT("Tag got distance to Anchor %u: %.2f m\n", rangingID, (double)dist.distance);
            // estimatorEnqueueDistance(&dist);
          }
        }
        // Update last successful ranging timestamp
        lastSuccessfulRanging[rangingID] = xTaskGetTickCount();
        consecutiveTimeouts = 0; // Reset timeout counter on successful ranging
        rangingOk = true;

        // Count successful ranging for rate debug
        // noteSuccessfulRange();
#if (MAX_SWARM_SIZE > 2)
        current_mode_trans = true;
        dwIdle(dev);
        dwSetReceiveWaitTimeout(dev, 1000);

        // Select a random peer to communicate with
        current_receiveID = selectNextPeer();

        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
        txPacket.payload[LPS_TWR_SEQ] = 0;
        txPacket.sourceAddress = selfAddress;
        txPacket.destAddress = basicAddr + current_receiveID;
        dwNewTransmit(dev);
        dwSetDefaults(dev);
        dwSetData(dev, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
#endif
      break;
    }
    }
  }
}

static uint32_t twrTagOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  // TODO: Nothing is done with failedRanging. This should be calculated
  // if (event != eventReceiveTimeout) {
    // DEBUG_PRINT("TWR Tag event: %d\n", event);
  // }
  switch(event) {
    case eventPacketReceived:
      rxcallback(dev);
#if (MAX_SWARM_SIZE > 2)
      checkTurn = false;
#endif
      break;
    case eventPacketSent:
      txcallback(dev);
      break;
    case eventReceiveFailed:
    // Likely collision/CRC/SFD error. Don't spend seconds retrying on same peer.
      dwIdle(dev);
      if (current_mode_trans) {
        current_receiveID = selectNextPeer();
        consecutiveTimeouts = 0;

        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
        txPacket.payload[LPS_TWR_SEQ] = 0;
        txPacket.sourceAddress = selfAddress;
        txPacket.destAddress = basicAddr + current_receiveID;

        dwNewTransmit(dev);
        dwSetDefaults(dev);
        dwSetData(dev, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
      } else {
        dwNewReceive(dev);
        dwSetDefaults(dev);
        dwStartReceive(dev);
      }
      break;
    case eventTimeout:  // Comes back to timeout after each ranging attempt
    case eventReceiveTimeout:
      if (current_mode_trans)
      {
        consecutiveTimeouts++;

        // If we've had too many timeouts with current peer, try a different one
        if (consecutiveTimeouts >= MAX_CONSECUTIVE_TIMEOUTS) {
          // DEBUG_PRINT("TWR Tag ID %d: switching peer due to timeouts when sending to %d\n", selfID, current_receiveID);
          current_receiveID = selectNextPeer();
          consecutiveTimeouts = 0;
        }

        dwIdle(dev);
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
        txPacket.payload[LPS_TWR_SEQ] = 0;
        txPacket.sourceAddress = selfAddress;
        txPacket.destAddress = basicAddr + current_receiveID;
        dwNewTransmit(dev);
        dwSetDefaults(dev);
        dwSetData(dev, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
      }
      else
      {
#if (MAX_SWARM_SIZE > 2)
        // Check if we should start communication due to ID-dependent timeout
        bool shouldStartTimeout = shouldStartDueToTimeout();
        
        if (xTaskGetTickCount() > checkTurnTick + 20) // > 20ms
        {
          if (checkTurn == true || shouldStartTimeout)
          {
            // If starting due to timeout, log it for debugging and record the time
            if (shouldStartTimeout && !checkTurn) {
              DEBUG_PRINT("TWR Tag ID %d: starting communication due to timeout\n", selfID);
              lastTimeoutStart = xTaskGetTickCount();
            }
            
            current_mode_trans = true;
            dwIdle(dev);
            dwSetReceiveWaitTimeout(dev, 1000);
            txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
            txPacket.payload[LPS_TWR_SEQ] = 0;
            txPacket.sourceAddress = selfAddress;
            txPacket.destAddress = basicAddr + current_receiveID;
            dwNewTransmit(dev);
            dwSetDefaults(dev);
            dwSetData(dev, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2);
            dwWaitForResponse(dev, true);
            dwStartTransmit(dev);
            checkTurn = false;
            break;
          }
        }
#endif
        dwNewReceive(dev);
        dwSetDefaults(dev);
        dwStartReceive(dev);
      }
      break;
    default:
      configASSERT(false);
  }

  return MAX_TIMEOUT;
}

static void twrTagInit(dwDevice_t *dev)
{
  // Initialize the packet in the TX buffer
  memset(&txPacket, 0, sizeof(txPacket));
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

  memset(&poll_tx, 0, sizeof(poll_tx));
  memset(&poll_rx, 0, sizeof(poll_rx));
  memset(&answer_tx, 0, sizeof(answer_tx));
  memset(&answer_rx, 0, sizeof(answer_rx));
  memset(&final_tx, 0, sizeof(final_tx));
  memset(&final_rx, 0, sizeof(final_rx));

  selfID = (uint8_t)(configblockGetRadioAddress() & 0xF);
  selfAddress = basicAddr + selfID;

  // Initialize peer activity tracking
  uint32_t currentTick = xTaskGetTickCount();
  for (int i = 0; i < (LOCODECK_NR_OF_TWR_ANCHORS + 1); i++)
  {
    lastSuccessfulRanging[i] = currentTick;
  }
  consecutiveTimeouts = 0;

  // Initialize timeout-based startup mechanism
  lastOverheardMessage = currentTick;
  hasOverheardAnyMessage = false;
  lastTimeoutStart = 0;

  // Init ranging rate tracking
  lastRateTick = xTaskGetTickCount();
  successfulRangeCount = 0;


  // Communication logic between each UWB
  // All nodes start in receive mode and use timeout-based startup
  // This prevents immediate collisions and allows for distributed startup
  current_receiveID = selectRandomPeer();
  current_mode_trans = false;
  dwSetReceiveWaitTimeout(dev, TWR_RECEIVE_TIMEOUT);

  for (int i = 0; i < MAX_SWARM_SIZE; i++)
  {
    median_data[i].index_inserting = 0;
    state.refresh[i] = false;
    // Initialize indirect distances to 0
    for (int j = 0; j < MAX_SWARM_SIZE; j++) {
      state.indirectDist[i][j] = 0;
    }
  }

  state.keep_flying = false;

  DEBUG_PRINT("twrtag initialized with ID: %d\n", selfID);
#if (MAX_SWARM_SIZE > 2)
  checkTurn = false;
#endif
  rangingOk = false;
}

static bool isRangingOk()
{
  return rangingOk;
}

void uwbTwrTagSetOptions(lpsTwrAlgoOptions_t* newOptions) {
  options = newOptions;
}

float lpsTwrTagGetDistance(const uint8_t anchorId) {
  return state.distance[anchorId];
}

static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {
  if (anchorId < LOCODECK_NR_OF_TWR_ANCHORS) {
    *position = options->anchorPosition[anchorId];
    return true;
  }
  return false;
}

static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  for (int i = 0; i < LOCODECK_NR_OF_TWR_ANCHORS; i++) {
    unorderedAnchorList[i] = i;
  }
  return LOCODECK_NR_OF_TWR_ANCHORS;
}

static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  uint8_t count = 0;
  for (int i = 0; i < LOCODECK_NR_OF_TWR_ANCHORS; i++) {
    if (state.failedRanging[i] < options->rangingFailedThreshold) {
      unorderedAnchorList[count] = i;
      count++;
    }
  }
  return count;
}

bool twrGetSwarmInfo(int robNum, uint16_t *range, float *x, float *y, float *gyroZ, float *height, float *vx, float *vy)
{
  // DEBUG_PRINT("twrGetSwarmInfo called for robot %d\n", robNum);
  uint8_t n = effectiveSwarmSize();
  if ((robNum < 0) || ((uint8_t)robNum >= n)) {
    return false;
  }
  if (state.refresh[robNum] == true)
  {
    state.refresh[robNum] = false;
    *range = state.distance[robNum];
    *x = state.x[robNum];
    *y = state.y[robNum];
    *gyroZ = state.gz[robNum];
    *height = state.h[robNum];
    *vx = state.vx[robNum];
    *vy = state.vy[robNum];
    return (true);
  }
  else
  {
    return (false);
  }
}

bool command_share(int RobIDfromControl, bool keep_flying)
{
  if (RobIDfromControl == 0)
  {
    state.keep_flying = keep_flying;
    return keep_flying;
  }
  else
  {
    return state.keep_flying;
  }
}

uwbAlgorithm_t uwbTwrTagAlgorithm = {
  .init = twrTagInit,
  .onEvent = twrTagOnEvent,
  .isRangingOk = isRangingOk,
  .getAnchorPosition = getAnchorPosition,
  .getAnchorIdList = getAnchorIdList,
  .getActiveAnchorIdList = getActiveAnchorIdList,
};

LOG_GROUP_START(ranging)
// Direct distances (my distance to peer j)
LOG_ADD(LOG_UINT16, distance0, &state.distance[0])
LOG_ADD(LOG_UINT16, distance1, &state.distance[1])
LOG_ADD(LOG_UINT16, distance2, &state.distance[2])
LOG_ADD(LOG_UINT16, distance3, &state.distance[3])
LOG_ADD(LOG_UINT16, distance4, &state.distance[4])
// Peer heights (meters)
LOG_ADD(LOG_FLOAT, height0, &state.h[0])
LOG_ADD(LOG_FLOAT, height1, &state.h[1])
LOG_ADD(LOG_FLOAT, height2, &state.h[2])
LOG_ADD(LOG_FLOAT, height3, &state.h[3])
LOG_ADD(LOG_FLOAT, height4, &state.h[4])
// Peer yaw rate (rad/s)
LOG_ADD(LOG_FLOAT, yawR0, &state.gz[0])
LOG_ADD(LOG_FLOAT, yawR1, &state.gz[1])
LOG_ADD(LOG_FLOAT, yawR2, &state.gz[2])
LOG_ADD(LOG_FLOAT, yawR3, &state.gz[3])
LOG_ADD(LOG_FLOAT, yawR4, &state.gz[4])
// Peer velocity X (m/s)
LOG_ADD(LOG_FLOAT, vx0, &state.vx[0])
LOG_ADD(LOG_FLOAT, vx1, &state.vx[1])
LOG_ADD(LOG_FLOAT, vx2, &state.vx[2])
LOG_ADD(LOG_FLOAT, vx3, &state.vx[3])
LOG_ADD(LOG_FLOAT, vx4, &state.vx[4])
// Peer velocity Y (m/s)
LOG_ADD(LOG_FLOAT, vy0, &state.vy[0])
LOG_ADD(LOG_FLOAT, vy1, &state.vy[1])
LOG_ADD(LOG_FLOAT, vy2, &state.vy[2])
LOG_ADD(LOG_FLOAT, vy3, &state.vy[3])
LOG_ADD(LOG_FLOAT, vy4, &state.vy[4])
// Indirect distances: drone i's distance to drone j (inDij)
// Drone 0's distances to others
LOG_ADD(LOG_UINT16, inD00, &state.indirectDist[0][0])
LOG_ADD(LOG_UINT16, inD01, &state.indirectDist[0][1])
LOG_ADD(LOG_UINT16, inD02, &state.indirectDist[0][2])
LOG_ADD(LOG_UINT16, inD03, &state.indirectDist[0][3])
LOG_ADD(LOG_UINT16, inD04, &state.indirectDist[0][4])
// Drone 1's distances to others
LOG_ADD(LOG_UINT16, inD10, &state.indirectDist[1][0])
LOG_ADD(LOG_UINT16, inD11, &state.indirectDist[1][1])
LOG_ADD(LOG_UINT16, inD12, &state.indirectDist[1][2])
LOG_ADD(LOG_UINT16, inD13, &state.indirectDist[1][3])
LOG_ADD(LOG_UINT16, inD14, &state.indirectDist[1][4])
// Drone 2's distances to others
LOG_ADD(LOG_UINT16, inD20, &state.indirectDist[2][0])
LOG_ADD(LOG_UINT16, inD21, &state.indirectDist[2][1])
LOG_ADD(LOG_UINT16, inD22, &state.indirectDist[2][2])
LOG_ADD(LOG_UINT16, inD23, &state.indirectDist[2][3])
LOG_ADD(LOG_UINT16, inD24, &state.indirectDist[2][4])
// Drone 3's distances to others
LOG_ADD(LOG_UINT16, inD30, &state.indirectDist[3][0])
LOG_ADD(LOG_UINT16, inD31, &state.indirectDist[3][1])
LOG_ADD(LOG_UINT16, inD32, &state.indirectDist[3][2])
LOG_ADD(LOG_UINT16, inD33, &state.indirectDist[3][3])
LOG_ADD(LOG_UINT16, inD34, &state.indirectDist[3][4])
// Drone 4's distances to others
LOG_ADD(LOG_UINT16, inD40, &state.indirectDist[4][0])
LOG_ADD(LOG_UINT16, inD41, &state.indirectDist[4][1])
LOG_ADD(LOG_UINT16, inD42, &state.indirectDist[4][2])
LOG_ADD(LOG_UINT16, inD43, &state.indirectDist[4][3])
LOG_ADD(LOG_UINT16, inD44, &state.indirectDist[4][4])
// Shared AUX channels
LOG_ADD(LOG_UINT8,  aux0, &state.aux[0])
LOG_ADD(LOG_UINT8,  aux1, &state.aux[1])
LOG_ADD(LOG_UINT8,  aux2, &state.aux[2])
LOG_ADD(LOG_UINT8,  aux3, &state.aux[3])
LOG_GROUP_STOP(ranging)

PARAM_GROUP_START(swarm)
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, size, &swarmSize)
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, auxPub, &auxPublisherId)
PARAM_GROUP_STOP(swarm)