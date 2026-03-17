#include <math.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "autoconf.h"
#include "debug.h"
#include "system.h"
#include "timers.h"
#include "olsr.h"
#include "routing.h"
#include "static_mem.h"

#ifndef OLSR_DEBUG_ENABLE
#undef DEBUG_PRINT
#define DEBUG_PRINT
#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static TaskHandle_t olsrRxTaskHandle;
static QueueHandle_t rxQueue;
static SemaphoreHandle_t olsrSetsMutex; /* Mutex for mprSet & mprSelectorSet & tcSet */
static Routing_Table_t *routingTable;
static Neighbor_Set_t *neighborSet;
static MPR_Set_t mprSet;
static MPR_Selector_Set_t mprSelectorSet;
static TimerHandle_t mprSelectorSetEvictionTimer;
NO_DMA_CCM_SAFE_ZERO_INIT Topology_Set_t topologySet;
static TimerHandle_t topologySetEvictionTimer;
static TimerHandle_t olsrTcTimer;
static uint16_t olsrTcMsgSeqNumber = 0;
static uint16_t olsrTcANSN = 0; /* Advertised Neighbor Sequence Number */
static uint16_t lastReceivedTcSeqNumbers[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = 0};
static uint16_t olsrPacketSeqNumber = 0;

static bool olsrIsDupTc(UWB_Address_t originAddress, uint16_t seqNumber) {
  ASSERT(originAddress <= NEIGHBOR_ADDRESS_MAX);
  if (seqNumber < lastReceivedTcSeqNumbers[originAddress]
      && (lastReceivedTcSeqNumbers[originAddress] - seqNumber) * OLSR_TC_INTERVAL > 2 * OLSR_TOPOLOGY_SET_HOLD_TIME) {
    DEBUG_PRINT("olsrIsDupTc: origin %u may have restarted, reset seq to %u.\n", originAddress, seqNumber);
    lastReceivedTcSeqNumbers[originAddress] = seqNumber;
    return false;
  }
  return seqNumber <= lastReceivedTcSeqNumbers[originAddress];
}

static void olsrUpdateDupTc(UWB_Address_t originAddress, uint16_t seqNumber) {
  ASSERT(originAddress <= NEIGHBOR_ADDRESS_MAX);
  lastReceivedTcSeqNumbers[originAddress] = MAX(lastReceivedTcSeqNumbers[originAddress], seqNumber);
}

static uint16_t getNextPacketSeqNumber() {
  return olsrPacketSeqNumber++;
}

static uint16_t getNextTcSeqNumber() {
  return olsrTcMsgSeqNumber++;
}

static void computeMPR() {
  /* 1. Clear previous computed mpr set. */
  mprSetClear(&mprSet);
  Neighbor_Bit_Set_t coverSet;
  neighborBitSetInit(&coverSet);

  /* 2. Add all symmetric one-hop neighbors that provide reachability to symmetric two-hop neighbors that are not yet covered. */
  for (UWB_Address_t twoHopNeighbor = 0; twoHopNeighbor <= NEIGHBOR_ADDRESS_MAX; twoHopNeighbor++) {
    if (coverSet.size == neighborSet->twoHop.size) {
      break;
    }
    if (!neighborSetHasTwoHop(neighborSet, twoHopNeighbor)) {
      continue;
    }
    if (!neighborBitSetHas(&coverSet, twoHopNeighbor) && neighborSet->twoHopReachSets[twoHopNeighbor].size == 1) {
      UWB_Address_t onlyOneHopNeighbor = (UWB_Address_t) log2(neighborSet->twoHopReachSets[twoHopNeighbor].bits);
      DEBUG_PRINT("computeMPR: onlyOneHopNeighbor to %u = %u.\n", twoHopNeighbor, onlyOneHopNeighbor);
      mprSetAdd(&mprSet, onlyOneHopNeighbor);
      neighborBitSetAdd(&coverSet, twoHopNeighbor);
    }
  }
  uint8_t remainUncoveredCount = neighborSet->twoHop.size - coverSet.size;
  /* 3. Add all symmetric one-hop neighbors that covers most uncovered two-hop neighbors. */
  for (UWB_Address_t round = 0; round <= remainUncoveredCount; round++) {
    if (coverSet.size == neighborSet->twoHop.size) {
      break;
    }
    /* 3.1 Collect reach counts of one-hop neighbors according to the number of uncovered two-hop neighbors. */
    uint8_t reachCount[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = 0};
    for (UWB_Address_t twoHopNeighbor = 0; twoHopNeighbor <= NEIGHBOR_ADDRESS_MAX; twoHopNeighbor++) {
      if (!neighborSetHasTwoHop(neighborSet, twoHopNeighbor)) {
        continue;
      }
      if (!neighborBitSetHas(&coverSet, twoHopNeighbor)) {
        for (UWB_Address_t oneHopNeighbor = 0; oneHopNeighbor <= NEIGHBOR_ADDRESS_MAX; oneHopNeighbor++) {
          if (!neighborSetHasOneHop(neighborSet, oneHopNeighbor)) {
            continue;
          }
          if (neighborSetHasRelation(neighborSet, oneHopNeighbor, twoHopNeighbor)) {
            reachCount[oneHopNeighbor]++;
          }
        }
      }
    }
    /* 3.2 Find the one-hop neighbor that covers most uncovered two-hop neighbors in this round. */
    UWB_Address_t mostOneHopNeighbor = 0;
    uint8_t mostCount = 0;
    for (UWB_Address_t oneHopNeighbor = 0; oneHopNeighbor <= NEIGHBOR_ADDRESS_MAX; oneHopNeighbor++) {
      if (!neighborSetHasOneHop(neighborSet, oneHopNeighbor)) {
        continue;
      }
      if (reachCount[oneHopNeighbor] > mostCount) {
        mostOneHopNeighbor = oneHopNeighbor;
        mostCount = reachCount[oneHopNeighbor];
      }
    }
    /* 3.3 Add this one-hop neighbor to mpr set and then update cover set. */
    mprSetAdd(&mprSet, mostOneHopNeighbor);
    for (UWB_Address_t twoHopNeighbor = 0; twoHopNeighbor <= NEIGHBOR_ADDRESS_MAX; twoHopNeighbor++) {
      if (!neighborSetHasTwoHop(neighborSet, twoHopNeighbor)) {
        continue;
      }
      if (!neighborBitSetHas(&coverSet, twoHopNeighbor)
          && neighborSetHasRelation(neighborSet, mostOneHopNeighbor, twoHopNeighbor)) {
        neighborBitSetAdd(&coverSet, twoHopNeighbor);
      }
    }
  }

  /* 4. For each currently selected mpr neighbor N, remove N from mpr set if all two-hop neighbors still covered. */
  // TODO: optimize

  if (coverSet.size == neighborSet->twoHop.size && coverSet.bits == neighborSet->twoHop.bits) {
    DEBUG_PRINT("computeMPR: covered all %u two-hop neighbors.\n", neighborSet->twoHop.size);
    printNeighborBitSet(&coverSet);
  } else {
    DEBUG_PRINT("computeMPR: cannot covered all %u two-hop neighbors, now covers %u.\n",
                neighborSet->twoHop.size,
                coverSet.size);
    printNeighborBitSet(&coverSet);
  }
}

static void routingTableClear(Neighbor_Bit_Set_t *destAddresses) {
  int count = 0;
  for (int i = 0; i < routingTable->size; i++) {
    if (routingTable->entries[i].type == ROUTE_OLSR) {
      routingTable->entries[i] = emptyRouteEntry();
      count++;
    }
  }
  routingTableSort(routingTable);
  routingTable->size -= count;
}

static void computeRoutingTable() {
  /* Populate all known node currently in this network */
  Neighbor_Bit_Set_t allKnownNodes;
  neighborBitSetInit(&allKnownNodes);
  /* Add one-hop and two-hop neighbors to allKnownNodes */
  for (UWB_Address_t neighbor = 0; neighbor <= NEIGHBOR_ADDRESS_MAX; neighbor++) {
    if (uwbGetAddress() != neighbor && neighborSetHas(neighborSet, neighbor)
        && !neighborBitSetHas(&allKnownNodes, neighbor)) {
      neighborBitSetAdd(&allKnownNodes, neighbor);
    }
  }
  /* Add other known nodes in topology set to allKnownNodes */
  for (UWB_Address_t mprSelector = 0; mprSelector <= NEIGHBOR_ADDRESS_MAX; mprSelector++) {
    for (UWB_Address_t mpr = 0; mpr <= NEIGHBOR_ADDRESS_MAX; mpr++) {
      if (topologySetHas(&topologySet, mprSelector, mpr)) {
        if (uwbGetAddress() != mprSelector && !neighborBitSetHas(&allKnownNodes, mprSelector)) {
          neighborBitSetAdd(&allKnownNodes, mprSelector);
        }
        if (uwbGetAddress() != mpr && !neighborBitSetHas(&allKnownNodes, mpr)) {
          neighborBitSetAdd(&allKnownNodes, mpr);
        }
      }
    }
  }

  /* Remove all related route entries */
  routingTableClear(&allKnownNodes);

  DEBUG_PRINT("%u has %u known neighbors = ", uwbGetAddress(), allKnownNodes.size);
  for (int neighborAddress = 0; neighborAddress <= NEIGHBOR_ADDRESS_MAX; neighborAddress++) {
    if (neighborBitSetHas(&allKnownNodes, neighborAddress)) {
      DEBUG_PRINT("%u ", neighborAddress);
    }
  }
  DEBUG_PRINT("\n");

  UWB_Address_t prevHopOf[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = UWB_DEST_EMPTY};
  bool visited[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = false};
  #ifdef OLSR_ROUTING_COMPUTATION_USE_HOP
  uint8_t curHop[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = 255};
  /* Init hop for one-hop neighbors */
  for (UWB_Address_t oneHopNeighbor = 0; oneHopNeighbor <= NEIGHBOR_ADDRESS_MAX; oneHopNeighbor++) {
    if (neighborSetHasOneHop(neighborSet, oneHopNeighbor)) {
      curHop[oneHopNeighbor] = 1;
      prevHopOf[oneHopNeighbor] = uwbGetAddress();
    }
  }
  /* Perform dijkstra's algorithm to find the shortest path to each dest node in allKnownNodes */
  for (UWB_Address_t round = 1; round <= allKnownNodes.size; round++) {
    UWB_Address_t minNode = UWB_DEST_EMPTY;
    uint8_t minHop = 255;
    /* Find the best path in current round */
    for (UWB_Address_t curNode = 0; curNode <= NEIGHBOR_ADDRESS_MAX; curNode++) {
      if (neighborBitSetHas(&allKnownNodes, curNode) && !visited[curNode]) {
        if (curHop[curNode] < minHop) {
          minNode = curNode;
          minHop = curHop[curNode];
        }
      }
    }
    if (minNode == UWB_DEST_EMPTY) {
      break;
    }
    visited[minNode] = true;
    /* Update hop for each path with lastAddress (MPR) = minNode */
    for (UWB_Address_t mprSelector = 0; mprSelector <= NEIGHBOR_ADDRESS_MAX; mprSelector++) {
      if (topologySetHas(&topologySet, mprSelector, minNode)) {
        if (curHop[minNode] + 1 < curHop[mprSelector]) {
          curHop[mprSelector] = curHop[minNode] + 1;
          prevHopOf[mprSelector] = minNode;
        }
      }
    }
  }
  #else
  float curWeight[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = 0};
  /* Init path weight for one-hop neighbors */
  for (UWB_Address_t oneHopNeighbor = 0; oneHopNeighbor <= NEIGHBOR_ADDRESS_MAX; oneHopNeighbor++) {
    if (neighborSetHasOneHop(neighborSet, oneHopNeighbor)) {
      // TODO: add link state as weight to calculate routing table.
      curWeight[oneHopNeighbor] = 1;
      prevHopOf[oneHopNeighbor] = uwbGetAddress();
    }
  }
  /* Perform dijkstra's algorithm to find the largest weight to each dest node in allKnownNodes */
  for (UWB_Address_t round = 1; round <= allKnownNodes.size; round++) {
    UWB_Address_t maxNode = UWB_DEST_EMPTY;
    float maxWeight = 0;
    /* Find the best path in current round */
    for (UWB_Address_t curNode = 0; curNode <= NEIGHBOR_ADDRESS_MAX; curNode++) {
      if (neighborBitSetHas(&allKnownNodes, curNode) && !visited[curNode]) {
        if (curWeight[curNode] > maxWeight) {
          maxNode = curNode;
          maxWeight = curWeight[curNode];
        }
      }
    }
    if (maxNode == UWB_DEST_EMPTY) {
      break;
    }
    visited[maxNode] = true;
    /* Update weight for each path with lastAddress (MPR) = maxNode */
    for (UWB_Address_t mprSelector = 0; mprSelector <= NEIGHBOR_ADDRESS_MAX; mprSelector++) {
      if (topologySetHas(&topologySet, mprSelector, maxNode)) {
        // TODO: add link state as weight to calculate routing table.
        if (curWeight[mprSelector] < curWeight[maxNode] + 1) {
          curWeight[mprSelector] = curWeight[maxNode] + 1;
          prevHopOf[mprSelector] = maxNode;
        }
      }
    }
  }
  #endif
  for (UWB_Address_t node = 0; node <= NEIGHBOR_ADDRESS_MAX; node++) {
    if (neighborBitSetHas(&allKnownNodes, node)) {
      uint8_t hopCount = 1;
      UWB_Address_t cur = node;
      DEBUG_PRINT("%u", cur);
      // TODO: check
      while (cur != UWB_DEST_EMPTY && prevHopOf[cur] != UWB_DEST_EMPTY && prevHopOf[cur] != uwbGetAddress()) {
        DEBUG_PRINT("<-%u", prevHopOf[cur]);
        cur = prevHopOf[cur];
        hopCount++;
      }
      DEBUG_PRINT("<-%u \n", prevHopOf[cur]);
      // TODO: add route entry
      if (prevHopOf[cur] != UWB_DEST_EMPTY) {
        Route_Entry_t route = {
            .type = ROUTE_OLSR,
            .valid = true,
            .destAddress = node,
            .nextHop = cur,
            .hopCount = hopCount,
            .expirationTime = 0,
            .destSeqNumber = 0,
            .validDestSeqFlag = false,
            .precursors = 0
        };
        routingTableAddEntry(routingTable, route);
      }
    }
  }
  printRoutingTable(routingTable);
}

static void olsrSendTc() {
  UWB_Packet_t packet = {
      .header.type = UWB_OLSR_MESSAGE,
      .header.srcAddress = uwbGetAddress(),
      .header.destAddress = UWB_DEST_ANY,
      .header.length = sizeof(UWB_Packet_Header_t)
  };
  OLSR_Packet_t *olsrPacket = (OLSR_Packet_t *) &packet.payload;
  OLSR_TC_Message_t *tcMsg = (OLSR_TC_Message_t *) &olsrPacket->payload;
  uint8_t mprSelectorToSend = mprSelectorSet.mprSelectors.size;
  uint8_t round = (uint8_t) ceil((double) mprSelectorToSend / OLSR_TC_MAX_BODY_UNIT);
  UWB_Address_t curMPRSelector = 0;
  for (uint8_t r = 1; r <= round; r++) {
    tcMsg->header.type = OLSR_TC_MESSAGE;
    tcMsg->header.srcAddress = uwbGetAddress();
    tcMsg->header.msgSequence = getNextTcSeqNumber();
    tcMsg->header.ttl = 255;
    tcMsg->header.hopCount = 0;
    tcMsg->ANSN = olsrTcANSN;

    uint8_t mprSelectorToSendThisRound = MIN(mprSelectorToSend, OLSR_TC_MAX_BODY_UNIT);
    uint8_t mprSelectorSendCount = 0;
    for (UWB_Address_t cur = curMPRSelector; cur <= NEIGHBOR_ADDRESS_MAX; cur++) {
      if (mprSelectorSendCount == mprSelectorToSendThisRound) {
        curMPRSelector = cur;
        break;
      }
      if (mprSelectorSetHas(&mprSelectorSet, cur)) {
        tcMsg->bodyUnits[mprSelectorSendCount].mprSelector = cur;
        mprSelectorSendCount++;
      }
    }

    tcMsg->header.msgLength =
        sizeof(OLSR_Message_Header_t) + sizeof(tcMsg->ANSN) + mprSelectorToSendThisRound * sizeof(OLSR_TC_Body_Unit_t);
    olsrPacket->header.seqNumber = getNextPacketSeqNumber();
    olsrPacket->header.length = sizeof(OLSR_Packet_Header_t) + tcMsg->header.msgLength;
    packet.header.length = sizeof(UWB_Packet_Header_t) + olsrPacket->header.length;
    DEBUG_PRINT("olsrSendTc: %u send %u mpr selector in tc = ",
                uwbGetAddress(),
                mprSelectorToSendThisRound
    );
    for (uint8_t i = 0; i < mprSelectorToSendThisRound; i++) {
      DEBUG_PRINT("%u ", tcMsg->bodyUnits[i].mprSelector);
    }
    DEBUG_PRINT(", seq = %u, ANSN = %u at round %u.\n",
                tcMsg->header.msgSequence,
                tcMsg->ANSN,
                r);
    uwbSendPacketBlock(&packet);
  }

}

static void olsrProcessTC(UWB_Address_t neighborAddress, OLSR_TC_Message_t *tcMsg) {
  /* 1. If the sender (not originator) of this message is not in the symmetric 1-hop neighborhood of this node, discard this tc message. */
  if (!neighborSetHasOneHop(neighborSet, neighborAddress)) {
    DEBUG_PRINT("olsrProcessTC: %u discard tc from non-one-hop neighbor %u.\n", uwbGetAddress(), neighborAddress);
    return;
  }
  uint16_t originAddress = tcMsg->header.srcAddress;
  uint16_t tcSeqNumber = tcMsg->header.msgSequence;
  if (olsrIsDupTc(originAddress, tcSeqNumber)) {
    DEBUG_PRINT(
        "olsrProcessTC: %u received duplicate tc message from neighbor %u, origin = %u, seq = %u, ttl = %u, hop = %u, ignore.\n",
        uwbGetAddress(),
        neighborAddress,
        originAddress,
        tcSeqNumber,
        tcMsg->header.ttl,
        tcMsg->header.hopCount);
    return;
  }
  olsrUpdateDupTc(originAddress, tcSeqNumber);
  DEBUG_PRINT("olsrProcessTC: %u received tc message from neighbor %u, origin = %u, seq = %u, ttl = %u, hop = %u.\n",
              uwbGetAddress(),
              neighborAddress,
              originAddress,
              tcSeqNumber,
              tcMsg->header.ttl,
              tcMsg->header.hopCount);
  /* 2. If there exist some tuple in the topology set where mpr == originator and tuple.ANSN > tcMsg.ANSN, discard this tc message. */
  for (UWB_Address_t mprSelector = 0; mprSelector <= NEIGHBOR_ADDRESS_MAX; mprSelector++) {
    if (topologySetHas(&topologySet, mprSelector, originAddress)
        && topologySet.items[mprSelector][originAddress].seqNumber > tcMsg->ANSN) {
      DEBUG_PRINT("olsrProcessTC: tuple ansn = %u > tcMsg.ANSN = %u, discard.\n",
                  topologySet.items[mprSelector][originAddress].seqNumber,
                  tcMsg->ANSN
      );
      return;
    }
  }
  bool topologyChanged = false;
  /* 3. Remove all tuples in the topology set where mpr == originator and tuple.ANSN < tcMsg.ANSN. */
  for (UWB_Address_t mprSelector = 0; mprSelector <= NEIGHBOR_ADDRESS_MAX; mprSelector++) {
    if (topologySetHas(&topologySet, mprSelector, originAddress)
        && topologySet.items[mprSelector][originAddress].seqNumber < tcMsg->ANSN) {
      DEBUG_PRINT("olsrProcessTC: discard tuple (mprSelector = %u, mpr = %u) with ansn = %u < tcMsg.ANSN = %u.\n",
                  mprSelector,
                  originAddress,
                  topologySet.items[mprSelector][originAddress].seqNumber,
                  tcMsg->ANSN
      );
      topologySetRemove(&topologySet, mprSelector, originAddress);
      topologyChanged = true;
    }
  }

  uint8_t bodyUnitCount =
      (tcMsg->header.msgLength - sizeof(OLSR_Message_Header_t) - sizeof(tcMsg->ANSN)) / sizeof(OLSR_TC_Body_Unit_t);
  DEBUG_PRINT("olsrProcessTC: mpr selector of %u = ", originAddress);
  for (uint8_t i = 0; i < bodyUnitCount; i++) {
    UWB_Address_t mprSelector = tcMsg->bodyUnits[i].mprSelector;
    UWB_Address_t mpr = originAddress;
    if (!topologySetHas(&topologySet, mprSelector, mpr)) {
      topologyChanged = true;
      topologySetAdd(&topologySet, mprSelector, mpr, tcMsg->ANSN);
    } else {
      topologySetUpdateExpirationTime(&topologySet, mprSelector, mpr);
    }
    DEBUG_PRINT("%u ", mprSelector);
  }
  DEBUG_PRINT("\n");

  if (topologyChanged) {
    DEBUG_PRINT("olsrProcessTC: compute routing table.\n");
    printTopologySet(&topologySet);
    computeRoutingTable();
  }

  tcMsg->header.hopCount++;
  tcMsg->header.ttl--;
  /* Forward this tc tcMsg if TTL > 0 if am the MPR of this one-hop neighbor */
  if (tcMsg->header.ttl > 0 && mprSelectorSetHas(&mprSelectorSet, neighborAddress)) {
    DEBUG_PRINT("olsrProcessTC: %u forward tc message from mpr selector %u, origin = %u, ttl = %u, hopCount = %u.\n",
                uwbGetAddress(),
                neighborAddress,
                tcMsg->header.srcAddress,
                tcMsg->header.ttl,
                tcMsg->header.hopCount);
    // TODO: check
    UWB_Packet_t packet = {
        .header.type = UWB_OLSR_MESSAGE,
        .header.srcAddress = uwbGetAddress(),
        .header.destAddress = UWB_DEST_ANY,
        .header.length = sizeof(UWB_Packet_Header_t) + tcMsg->header.msgLength
    };
    OLSR_Packet_t *olsrPacket = (OLSR_Packet_t *) &packet.payload;
    memcpy(&olsrPacket->payload, tcMsg, tcMsg->header.msgLength);
    olsrPacket->header.seqNumber = getNextPacketSeqNumber();
    olsrPacket->header.length = sizeof(OLSR_Packet_Header_t) + tcMsg->header.msgLength;
    packet.header.length = sizeof(UWB_Packet_Header_t) + olsrPacket->header.length;
    uwbSendPacketBlock(&packet);
  }
}

static void olsrTcTimerCallback(TimerHandle_t timer) {
  xSemaphoreTake(olsrSetsMutex, portMAX_DELAY);
  xSemaphoreTake(neighborSet->mu, portMAX_DELAY);
//  printNeighborSet(neighborSet);
//  printMPRSet(&mprSet);
//  printMPRSelectorSet(&mprSelectorSet);
//  printTopologySet(&topologySet);
//  printRoutingTable(routingTable);
  if (mprSelectorSet.mprSelectors.size > 0) {
    olsrSendTc();
  }
  xSemaphoreGive(neighborSet->mu);
  xSemaphoreGive(olsrSetsMutex);
}

void olsrNeighborTopologyChangeHook(UWB_Address_t neighborAddress) {
  xSemaphoreTake(olsrSetsMutex, portMAX_DELAY);
  xSemaphoreTake(routingTable->mu, portMAX_DELAY);
  computeMPR();
  DEBUG_PRINT("olsrNeighborTopologyChangeHook: compute routing table.\n");
  computeRoutingTable();
  olsrTcANSN++;
  lastReceivedTcSeqNumbers[neighborAddress] = 0;
  xSemaphoreGive(routingTable->mu);
  xSemaphoreGive(olsrSetsMutex);
}

MPR_Set_t *getGlobalMPRSet() {
  return &mprSet;
}

void mprSetInit(MPR_Set_t *set) {
  neighborBitSetInit(set);
}

void mprSetAdd(MPR_Set_t *set, UWB_Address_t neighborAddress) {
  neighborBitSetAdd(set, neighborAddress);
}

void mprSetRemove(MPR_Set_t *set, UWB_Address_t neighborAddress) {
  neighborBitSetRemove(set, neighborAddress);
}

bool mprSetHas(MPR_Set_t *set, UWB_Address_t neighborAddress) {
  return neighborBitSetHas(set, neighborAddress);
}

void mprSetClear(MPR_Set_t *set) {
  neighborBitSetClear(set);
}

MPR_Selector_Set_t *getGlobalMPRSelectorSet() {
  return &mprSelectorSet;
}

void mprSelectorSetInit(MPR_Selector_Set_t *set) {
  neighborBitSetInit(&set->mprSelectors);
  for (UWB_Address_t neighborAddress = 0; neighborAddress <= NEIGHBOR_ADDRESS_MAX; neighborAddress++) {
    set->expirationTime[neighborAddress] = 0;
  }
}

void mprSelectorSetAdd(MPR_Selector_Set_t *set, UWB_Address_t neighborAddress) {
  if (!neighborBitSetHas(&set->mprSelectors, neighborAddress)) {
    neighborBitSetAdd(&set->mprSelectors, neighborAddress);
    set->expirationTime[neighborAddress] = xTaskGetTickCount() + M2T(OLSR_MPR_SELECTOR_SET_HOLD_TIME);
  }
}

void mprSelectorSetRemove(MPR_Selector_Set_t *set, UWB_Address_t neighborAddress) {
  if (neighborBitSetHas(&set->mprSelectors, neighborAddress)) {
    neighborBitSetRemove(&set->mprSelectors, neighborAddress);
    set->expirationTime[neighborAddress] = 0;
  }
}

bool mprSelectorSetHas(MPR_Selector_Set_t *set, UWB_Address_t neighborAddress) {
  return neighborBitSetHas(&set->mprSelectors, neighborAddress);
}

void mprSelectorSetUpdateExpirationTime(MPR_Selector_Set_t *set, UWB_Address_t neighborAddress) {
  ASSERT(neighborAddress <= NEIGHBOR_ADDRESS_MAX);
  set->expirationTime[neighborAddress] = xTaskGetTickCount() + M2T(OLSR_MPR_SELECTOR_SET_HOLD_TIME);
}

int mprSelectorSetClearExpire(MPR_Selector_Set_t *set) {
  Time_t curTime = xTaskGetTickCount();
  int evictionCount = 0;
  for (UWB_Address_t neighborAddress = 0; neighborAddress <= NEIGHBOR_ADDRESS_MAX; neighborAddress++) {
    if (mprSelectorSetHas(set, neighborAddress) && set->expirationTime[neighborAddress] <= curTime) {
      evictionCount++;
      mprSelectorSetRemove(set, neighborAddress);
      DEBUG_PRINT("mprSelectorSetClearExpire: mpr selector %u expire at %lu.\n", neighborAddress, curTime);
    }
  }
  return evictionCount;
}

static void mprSelectorSetClearExpireTimerCallback(TimerHandle_t timer) {
  xSemaphoreTake(olsrSetsMutex, portMAX_DELAY);

  Time_t curTime = xTaskGetTickCount();
  DEBUG_PRINT("mprSelectorSetClearExpireTimerCallback: Trigger expiration timer at %lu.\n", curTime);

  int evictionCount = mprSelectorSetClearExpire(&mprSelectorSet);
  if (evictionCount > 0) {
    DEBUG_PRINT("mprSelectorSetClearExpireTimerCallback: Evict total %d mpr selectors.\n", evictionCount);
  } else {
    DEBUG_PRINT("mprSelectorSetClearExpireTimerCallback: Evict none.\n");
  }

  xSemaphoreGive(olsrSetsMutex);
}

void topologySetInit(Topology_Set_t *set) {
  set->size = 0;
  for (UWB_Address_t mprSelector = 0; mprSelector <= NEIGHBOR_ADDRESS_MAX; mprSelector++) {
    for (UWB_Address_t mpr = 0; mpr <= NEIGHBOR_ADDRESS_MAX; mpr++) {
      set->items[mprSelector][mpr].destAddress = UWB_DEST_EMPTY;
      set->items[mprSelector][mpr].lastAddress = UWB_DEST_EMPTY;
      set->items[mprSelector][mpr].seqNumber = 0;
      set->items[mprSelector][mpr].expirationTime = 0;
    }
  }
}

void topologySetAdd(Topology_Set_t *set, UWB_Address_t mprSelector, UWB_Address_t mpr, uint16_t seqNumber) {
  ASSERT(mprSelector <= NEIGHBOR_ADDRESS_MAX);
  ASSERT(mpr <= NEIGHBOR_ADDRESS_MAX);
  if (!topologySetHas(set, mprSelector, mpr)) {
    set->items[mprSelector][mpr].destAddress = mprSelector;
    set->items[mprSelector][mpr].lastAddress = mpr;
    set->items[mprSelector][mpr].seqNumber = seqNumber;
    set->items[mprSelector][mpr].expirationTime = xTaskGetTickCount() + M2T(OLSR_TOPOLOGY_SET_HOLD_TIME);
    set->size++;
  }
}

void topologySetRemove(Topology_Set_t *set, UWB_Address_t mprSelector, UWB_Address_t mpr) {
  ASSERT(mprSelector <= NEIGHBOR_ADDRESS_MAX);
  ASSERT(mpr <= NEIGHBOR_ADDRESS_MAX);
  if (topologySetHas(set, mprSelector, mpr)) {
    set->items[mprSelector][mpr].destAddress = UWB_DEST_EMPTY;
    set->items[mprSelector][mpr].lastAddress = UWB_DEST_EMPTY;
    set->items[mprSelector][mpr].seqNumber = 0;
    set->items[mprSelector][mpr].expirationTime = 0;
    set->size--;
  }
}

bool topologySetHas(Topology_Set_t *set, UWB_Address_t mprSelector, UWB_Address_t mpr) {
  ASSERT(mprSelector <= NEIGHBOR_ADDRESS_MAX);
  ASSERT(mpr <= NEIGHBOR_ADDRESS_MAX);
  return set->items[mprSelector][mpr].destAddress != UWB_DEST_EMPTY;
}

void topologySetUpdateExpirationTime(Topology_Set_t *set, UWB_Address_t mprSelector, UWB_Address_t mpr) {
  ASSERT(mprSelector <= NEIGHBOR_ADDRESS_MAX);
  ASSERT(mpr <= NEIGHBOR_ADDRESS_MAX);
  set->items[mprSelector][mpr].expirationTime = xTaskGetTickCount() + M2T(OLSR_TOPOLOGY_SET_HOLD_TIME);
}

int topologySetClearExpire(Topology_Set_t *set) {
  Time_t curTime = xTaskGetTickCount();
  int evictionCount = 0;
  for (UWB_Address_t mprSelector = 0; mprSelector <= NEIGHBOR_ADDRESS_MAX; mprSelector++) {
    for (UWB_Address_t mpr = 0; mpr <= NEIGHBOR_ADDRESS_MAX; mpr++) {
      if (topologySetHas(set, mprSelector, mpr) && set->items[mprSelector][mpr].expirationTime <= curTime) {
        evictionCount++;
        topologySetRemove(set, mprSelector, mpr);
        DEBUG_PRINT("topologySetClearExpire: topology tuple (mprSelector = %u, mpr = %u, ansn = %u) expire at %lu.\n",
                    mprSelector,
                    mpr,
                    set->items[mprSelector][mpr].seqNumber,
                    curTime);
      }
    }
  }
  return evictionCount;
}

static void topologySetClearExpireTimerCallback(TimerHandle_t timer) {
  xSemaphoreTake(olsrSetsMutex, portMAX_DELAY);
  xSemaphoreTake(routingTable->mu, portMAX_DELAY);

  Time_t curTime = xTaskGetTickCount();
  DEBUG_PRINT("topologySetClearExpireTimerCallback: Trigger expiration timer at %lu.\n", curTime);

  int evictionCount = topologySetClearExpire(&topologySet);
  if (evictionCount > 0) {
    DEBUG_PRINT("topologySetClearExpireTimerCallback: Evict total %d topology tuples.\n", evictionCount);
    computeRoutingTable();
  } else {
    DEBUG_PRINT("topologySetClearExpireTimerCallback: Evict none.\n");
  }

  xSemaphoreGive(routingTable->mu);
  xSemaphoreGive(olsrSetsMutex);
}

void printMPRSet(MPR_Set_t *set) {
  DEBUG_PRINT("%u has %u mpr neighbors = ", uwbGetAddress(), set->size);
  for (UWB_Address_t neighborAddress = 0; neighborAddress <= NEIGHBOR_ADDRESS_MAX; neighborAddress++) {
    if (mprSetHas(set, neighborAddress)) {
      DEBUG_PRINT("%u ", neighborAddress);
    }
  }
  DEBUG_PRINT("\n");
}

void printMPRSelectorSet(MPR_Selector_Set_t *set) {
  DEBUG_PRINT("%u has %u mpr selectors = ", uwbGetAddress(), set->mprSelectors.size);
  for (UWB_Address_t neighborAddress = 0; neighborAddress <= NEIGHBOR_ADDRESS_MAX; neighborAddress++) {
    if (mprSelectorSetHas(set, neighborAddress)) {
      DEBUG_PRINT("%u ", neighborAddress);
    }
  }
  DEBUG_PRINT("\n");
}

void printTopologySetTuple(Topology_Tuple_t *tuple) {
  DEBUG_PRINT("dest\t last\t seq\t expire\t \n");
  DEBUG_PRINT("%u\t %u\t %u\t %lu\t \n",
              tuple->destAddress,
              tuple->lastAddress,
              tuple->seqNumber,
              tuple->expirationTime
  );
}

void printTopologySet(Topology_Set_t *set) {
  DEBUG_PRINT("dest\t last\t seq\t expire\t \n");
  for (UWB_Address_t mprSelector = 0; mprSelector <= NEIGHBOR_ADDRESS_MAX; mprSelector++) {
    for (UWB_Address_t mpr = 0; mpr <= NEIGHBOR_ADDRESS_MAX; mpr++) {
      if (topologySetHas(set, mprSelector, mpr)) {
        DEBUG_PRINT("%u\t %u\t %u\t %lu\t \n",
                    set->items[mprSelector][mpr].destAddress,
                    set->items[mprSelector][mpr].lastAddress,
                    set->items[mprSelector][mpr].seqNumber,
                    set->items[mprSelector][mpr].expirationTime
        );
      }
    }
  }
}

void olsrRxCallback(void *parameters) {
//  DEBUG_PRINT("olsrRxCallback\n");
}

void olsrTxCallback(void *parameters) {
//  DEBUG_PRINT("olsrTxCallback\n");
}

static void olsrRxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t rxPacketCache;
  OLSR_Packet_t *olsrPacket = (OLSR_Packet_t *) &rxPacketCache.payload;

  while (true) {
    if (uwbReceivePacketBlock(UWB_OLSR_MESSAGE, &rxPacketCache)) {
      xSemaphoreTake(olsrSetsMutex, portMAX_DELAY);
      xSemaphoreTake(neighborSet->mu, portMAX_DELAY);
      xSemaphoreTake(routingTable->mu, portMAX_DELAY);
      /* Since we do not send multiple messages in a single OLSR packet, this processing approach is OK. */
      OLSR_Message_Header_t *msgHeader = (OLSR_Message_Header_t *) olsrPacket->payload;
      switch (msgHeader->type) {
        case OLSR_HELLO_MESSAGE:DEBUG_PRINT("olsrRxTask: %u received HELLO from %u.\n",
                                            uwbGetAddress(),
                                            msgHeader->srcAddress);
          // Use Ranging instead of HELLO here, see swarm_ranging.h, just ignore.
          break;
        case OLSR_TC_MESSAGE:DEBUG_PRINT("olsrRxTask: %u received TC from %u.\n",
                                         uwbGetAddress(),
                                         msgHeader->srcAddress);
          olsrProcessTC(rxPacketCache.header.srcAddress, (OLSR_TC_Message_t *) &olsrPacket->payload);
          break;
        default:DEBUG_PRINT("olsrRxTask: %u received unknown olsr message type from %u.\n",
                            uwbGetAddress(),
                            msgHeader->srcAddress);
      }
      xSemaphoreGive(routingTable->mu);
      xSemaphoreGive(neighborSet->mu);
      xSemaphoreGive(olsrSetsMutex);
    }
    vTaskDelay(M2T(1));
  }

}

void olsrInit() {
  rxQueue = xQueueCreate(OLSR_RX_QUEUE_SIZE, OLSR_RX_QUEUE_ITEM_SIZE);
  olsrSetsMutex = xSemaphoreCreateMutex();
  routingTable = getGlobalRoutingTable();
  neighborSet = getGlobalNeighborSet();
  neighborSetRegisterTopologyChangeHook(neighborSet, olsrNeighborTopologyChangeHook);
  mprSetInit(&mprSet);
  mprSelectorSetInit(&mprSelectorSet);
  mprSelectorSetEvictionTimer = xTimerCreate("mprSelectorSetEvictionTimer",
                                             M2T(OLSR_MPR_SELECTOR_SET_HOLD_TIME / 2),
                                             pdTRUE,
                                             (void *) 0,
                                             mprSelectorSetClearExpireTimerCallback);
  xTimerStart(mprSelectorSetEvictionTimer, M2T(0));
  topologySetInit(&topologySet);
  topologySetEvictionTimer = xTimerCreate("topologySetEvictionTimer",
                                          M2T(OLSR_TOPOLOGY_SET_HOLD_TIME / 2),
                                          pdTRUE,
                                          (void *) 0,
                                          topologySetClearExpireTimerCallback);
  xTimerStart(topologySetEvictionTimer, M2T(0));
  olsrTcTimer = xTimerCreate("olsrTcTimer",
                             M2T(OLSR_TC_INTERVAL),
                             pdTRUE,
                             (void *) 0,
                             olsrTcTimerCallback);
  xTimerStart(olsrTcTimer, M2T(0));

  UWB_Message_Listener_t listener;
  listener.type = UWB_OLSR_MESSAGE;
  listener.rxQueue = rxQueue;
  listener.rxCb = olsrRxCallback;
  listener.txCb = olsrTxCallback;
  uwbRegisterListener(&listener);

  xTaskCreate(olsrRxTask,
              ADHOC_DECK_OLSR_RX_TASK_NAME,
              UWB_TASK_STACK_SIZE,
              NULL,
              ADHOC_DECK_TASK_PRI,
              &olsrRxTaskHandle);
}