#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "autoconf.h"
#include "debug.h"
#include "system.h"
#include "routing.h"
#include "aodv.h"
#include "timers.h"

#ifndef AODV_DEBUG_ENABLE
  #undef DEBUG_PRINT
  #define DEBUG_PRINT
#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

typedef struct {
  UWB_Address_t origAddress;
  uint32_t requestId;
} RREQ_Buffer_Item_t;

typedef struct {
  uint8_t index;
  RREQ_Buffer_Item_t items[AODV_RREQ_BUFFER_SIZE_MAX];
} RREQ_Buffer_t;

static TaskHandle_t aodvRxTaskHandle = 0;
static QueueHandle_t rxQueue;
static uint32_t aodvSeqNumber = 0;
static uint32_t aodvRequestId = 0;
static RREQ_Buffer_t rreqBuffer;
static Routing_Table_t *routingTable;
static TimerHandle_t aodvHelloTimer;

static void rreqBufferInit(RREQ_Buffer_t *buffer) {
  for (int i = 0; i < AODV_RREQ_BUFFER_SIZE_MAX; i++) {
    buffer->items[i].requestId = 0;
    buffer->items[i].origAddress = UWB_DEST_EMPTY;
  }
}

static void rreqBufferAdd(RREQ_Buffer_t *buffer, UWB_Address_t neighborAddress, uint32_t requestId) {
  RREQ_Buffer_Item_t item = {.origAddress = neighborAddress, .requestId = requestId};
  buffer->items[buffer->index] = item;
  buffer->index = (buffer->index + 1) % AODV_RREQ_BUFFER_SIZE_MAX;
}

static int rreqBufferFind(RREQ_Buffer_t *buffer, UWB_Address_t neighborAddress, uint32_t requestId) {
  for (int i = 0; i < AODV_RREQ_BUFFER_SIZE_MAX; i++) {
    if (neighborAddress == buffer->items[i].origAddress && requestId == buffer->items[i].requestId) {
      return i;
    }
  }
  return -1;
}

static bool rreqBufferIsDuplicate(RREQ_Buffer_t *buffer, UWB_Address_t neighborAddress, uint32_t requestId) {
  for (int i = 0; i < AODV_RREQ_BUFFER_SIZE_MAX; i++) {
    if (neighborAddress == buffer->items[i].origAddress && requestId == buffer->items[i].requestId) {
      return true;
    }
  }
  return false;
}

static uint64_t precursorListAdd(uint64_t precursors, UWB_Address_t address) {
  return precursors | (1ULL << address);
}

static uint64_t precursorListRemove(uint64_t precursors, UWB_Address_t address) {
  return precursors & ~(1ULL << address);
}

static int aodvCompareSeqNumber(uint32_t first, uint32_t second) {
  int32_t firstV = (int32_t) first;
  int32_t secondV = (int32_t) second;
  if (firstV == secondV) {
    return 0;
  }
  if (firstV - secondV > 0) {
    return 1;
  }
  return -1;
}

static void aodvSendHello() {
//  printRoutingTable(routingTable);
  UWB_Packet_t packet = {
      .header.type = UWB_AODV_MESSAGE,
      .header.srcAddress = uwbGetAddress(),
      .header.destAddress = UWB_DEST_ANY,
      .header.length = sizeof(UWB_Packet_Header_t) + sizeof(AODV_RREP_Message_t)
  };
  AODV_RREP_Message_t *rrepHello = (AODV_RREP_Message_t *) &packet.payload;
  rrepHello->type = AODV_RREP;
  rrepHello->hopCount = 0;
  rrepHello->destAddress = uwbGetAddress();
  rrepHello->origAddress = uwbGetAddress();
  rrepHello->destSeqNumber = aodvSeqNumber;
  rrepHello->lifetime = xTaskGetTickCount() + 2 * M2T(AODV_HELLO_INTERVAL);
  uwbSendPacketBlock(&packet);
}

static void aodvProcessHello(AODV_RREP_Message_t *rrep) {
  /*
   * Whenever a node receives a Hello message from a neighbor, the node
   * SHOULD make sure that it has an active route to the neighbor, and
   * create one if necessary.
   */
  Route_Entry_t toNeighbor = routingTableFindEntry(routingTable, rrep->origAddress);
  if (toNeighbor.destAddress == UWB_DEST_EMPTY) {
    toNeighbor.type = ROUTE_AODV,
    toNeighbor.valid = true;
    toNeighbor.destAddress = rrep->origAddress;
    toNeighbor.nextHop = rrep->origAddress;
    toNeighbor.hopCount = 1;
    toNeighbor.expirationTime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME);
    toNeighbor.destSeqNumber = rrep->destSeqNumber;
    toNeighbor.validDestSeqFlag = true;
    routingTableAddEntry(routingTable, toNeighbor);
  } else {
    toNeighbor.type = ROUTE_AODV,
    toNeighbor.valid = true;
    toNeighbor.nextHop = rrep->origAddress;
    toNeighbor.hopCount = 1;
    toNeighbor.expirationTime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME);
    toNeighbor.destSeqNumber = rrep->destSeqNumber;
    toNeighbor.validDestSeqFlag = true;
    routingTableUpdateEntry(routingTable, toNeighbor);
  }
}

static void aodvHelloTimerCallback(TimerHandle_t timer) {
  Time_t curTime = xTaskGetTickCount();
  DEBUG_PRINT("aodvHelloTimerCallback: send hello at %lu.\n", curTime);
  aodvSendHello();
}

static void sendRREPACK(UWB_Address_t neighborAddress) {
  UWB_Packet_t packet = {
      .header.type = UWB_AODV_MESSAGE,
      .header.srcAddress = uwbGetAddress(),
      .header.destAddress = neighborAddress,
      .header.length = sizeof(UWB_Packet_Header_t) + sizeof(AODV_RREP_ACK_Message_t)
  };
  AODV_RREP_ACK_Message_t *rrepAck = (AODV_RREP_ACK_Message_t *) &packet.payload;
  rrepAck->type = AODV_RREP_ACK;
  DEBUG_PRINT("sendRREPACK: %u send rrepAck to neighbor %u.\n",
              uwbGetAddress(),
              neighborAddress
  );
  uwbSendPacketBlock(&packet);
}

// TODO: rate limiter
static void sendRREP(AODV_RREQ_Message_t *rreq, Route_Entry_t toOrigin) {
  /*
   * Destination node MUST increment its own sequence number by one if the sequence number in the
   * RREQ packet is equal to that incremented value. Otherwise, the destination does not change
   * its sequence number before generating the  RREP message.
   */
  if (!rreq->flags.U && rreq->destSeqNumber == aodvSeqNumber + 1) {
    aodvSeqNumber++;
  }
  UWB_Packet_t packet = {
      .header.type = UWB_AODV_MESSAGE,
      .header.srcAddress = uwbGetAddress(),
      .header.destAddress = toOrigin.nextHop,
      .header.length = sizeof(UWB_Packet_Header_t) + sizeof(AODV_RREP_Message_t)
  };
  AODV_RREP_Message_t *rrep = (AODV_RREP_Message_t *) &packet.payload;
  rrep->type = AODV_RREP;
  rrep->hopCount = 0;
  rrep->destAddress = rreq->destAddress;
  rrep->destSeqNumber = aodvSeqNumber;
  rrep->origAddress = toOrigin.destAddress;
  rrep->lifetime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME);
  uwbSendPacketBlock(&packet);
}
// TODO: rate limiter
static void sendRREPByIntermediateNode(Route_Entry_t toDest, Route_Entry_t toOrigin) {
  /* Have not implement gratuitous rrep here. */
  UWB_Packet_t packet = {
      .header.type = UWB_AODV_MESSAGE,
      .header.srcAddress = uwbGetAddress(),
      .header.destAddress = toOrigin.nextHop,
      .header.length = sizeof(UWB_Packet_Header_t) + sizeof(AODV_RREP_Message_t)
  };

  AODV_RREP_Message_t *rrep = (AODV_RREP_Message_t *) &packet.payload;
  rrep->type = AODV_RREP;
  rrep->flags.prefixSize = 0;
  rrep->hopCount = toDest.hopCount;
  rrep->destAddress = toDest.destAddress;
  rrep->destSeqNumber = toDest.destSeqNumber;
  rrep->origAddress = toOrigin.destAddress;
  rrep->lifetime = toDest.expirationTime;

  /* If the node we received a RREQ for is a neighbor we are
   * probably facing a unidirectional link... Better request a RREP-ack
   */
  if (toDest.hopCount == 1) {
    rrep->flags.A = true;
  }

  toDest.precursors = precursorListAdd(toDest.precursors, toOrigin.nextHop);
  toOrigin.precursors = precursorListAdd(toOrigin.precursors, toDest.nextHop);
  routingTableUpdateEntry(routingTable, toDest);
  routingTableUpdateEntry(routingTable, toOrigin);
  uwbSendPacketBlock(&packet);
}

static void aodvProcessRREQ(UWB_Packet_t *packet) {
  AODV_RREQ_Message_t *rreq = (AODV_RREQ_Message_t *) &packet->payload;
  DEBUG_PRINT(
      "aodvProcessRREQ: %u received RREQ from origin = %u, reqId = %lu, src = %u, dest = %u, destSeq = %lu, hop = %u.\n",
      uwbGetAddress(),
      rreq->origAddress,
      rreq->requestId,
      packet->header.srcAddress,
      rreq->destAddress,
      rreq->destSeqNumber,
      rreq->hopCount
  );
  if (rreq->origAddress == uwbGetAddress() || rreqBufferIsDuplicate(&rreqBuffer, rreq->origAddress, rreq->requestId)) {
    DEBUG_PRINT("aodvProcessRREQ: %u discard duplicate rreq origin = %u, reqId = %lu, dest = %u.\n",
                uwbGetAddress(),
                rreq->origAddress,
                rreq->requestId,
                rreq->destAddress
    );
    return;
  } else {
    rreqBufferAdd(&rreqBuffer, rreq->origAddress, rreq->requestId);
  }
  rreq->hopCount++;

  /*  Origin -> Neighbor -> Me -> Dest
   *  When the reverse route is created or updated, the following actions on the route are also
   * carried out:
   *  1. the Originator Sequence Number from the RREQ is compared to the corresponding destination
   * sequence number in the route table entry and copied if greater than the existing value there;
   *  2. the valid sequence number field is set to true;
   *  3. the next hop in the routing table becomes the node from which the RREQ was received
   *  4. the hop count is copied from the Hop Count in the rreq;
   *  5. the Lifetime is updated.
   */

  /* Update route for Me to Origin through Neighbor */
  Route_Entry_t toOrigin = routingTableFindEntry(routingTable, rreq->origAddress);
  if (toOrigin.destAddress == UWB_DEST_EMPTY) {
    toOrigin.type = ROUTE_AODV,
    toOrigin.destAddress = rreq->origAddress;
    toOrigin.valid = true;
    toOrigin.validDestSeqFlag = true;
    toOrigin.destSeqNumber = rreq->origSeqNumber;
    toOrigin.hopCount = rreq->hopCount;
    toOrigin.nextHop = packet->header.srcAddress;
    toOrigin.expirationTime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME);
    routingTableAddEntry(routingTable, toOrigin);
  } else {
    if (toOrigin.validDestSeqFlag) {
      if (aodvCompareSeqNumber(rreq->origSeqNumber, toOrigin.destSeqNumber) > 0) {
        toOrigin.destSeqNumber = rreq->origSeqNumber;
      }
    } else {
      toOrigin.destSeqNumber = rreq->origSeqNumber;
    }
    toOrigin.type = ROUTE_AODV,
    toOrigin.valid = true;
    toOrigin.validDestSeqFlag = true;
    toOrigin.nextHop = packet->header.srcAddress;
    toOrigin.hopCount = rreq->hopCount;
    toOrigin.expirationTime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME);
    routingTableUpdateEntry(routingTable, toOrigin);
  }

  /* Update route for Me to Neighbor */
  Route_Entry_t toNeighbor = routingTableFindEntry(routingTable, packet->header.srcAddress);
  if (toNeighbor.destAddress == UWB_DEST_EMPTY) {
    toNeighbor.type = ROUTE_AODV,
    toNeighbor.valid = true;
    toNeighbor.destAddress = packet->header.srcAddress;
    toNeighbor.validDestSeqFlag = false;
    toNeighbor.destSeqNumber = rreq->origSeqNumber;
    toNeighbor.hopCount = 1;
    toNeighbor.nextHop = packet->header.srcAddress;
    toNeighbor.expirationTime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME);
    routingTableAddEntry(routingTable, toNeighbor);
  } else {
    toNeighbor.type = ROUTE_AODV,
    toNeighbor.valid = true;
    toNeighbor.validDestSeqFlag = false;
    toNeighbor.destAddress = packet->header.srcAddress;
    toNeighbor.destSeqNumber = rreq->origSeqNumber;
    toNeighbor.hopCount = 1;
    toNeighbor.nextHop = packet->header.srcAddress;
    toNeighbor.expirationTime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME);
    routingTableUpdateEntry(routingTable, toNeighbor);
  }

  /* A node generates a RREP if either:
   * (i) it is itself the destination,
   */
  if (rreq->destAddress == uwbGetAddress()) {
    sendRREP(rreq, toOrigin);
    return;
  }
  /*
   * (ii) or it has an active route to the destination, the destination sequence number in the
   * node's existing route table entry for the destination is valid and greater than or equal to
   * the Destination Sequence Number of the RREQ, and the "destination only" flag is NOT set.
   */
  Route_Entry_t toDest = routingTableFindEntry(routingTable, rreq->destAddress);
  if (toDest.destAddress != UWB_DEST_EMPTY) {
    /* Drop RREQ that will make a loop. (i.e. A->D but get A->B->C->B->C->B->C) */
    if (toDest.nextHop == packet->header.srcAddress) {
      DEBUG_PRINT(
          "aodvProcessRREQ: %u drop RREQ from origin = %u, reqId = %lu, src = %u, dest = %u, destSeq = %lu, next hop = %u.\n",
          uwbGetAddress(),
          rreq->origAddress,
          rreq->requestId,
          packet->header.srcAddress,
          rreq->destAddress,
          rreq->destSeqNumber,
          toDest.nextHop);
      return;
    }
    /*
     * The Destination Sequence number for the requested destination is set to the maximum of
     * the corresponding value received in the RREQ message, and the destination sequence value
     * currently maintained by the node for the requested destination. However, the forwarding
     * node MUST NOT modify its maintained value for the destination sequence number, even if
     * the value received in the incoming RREQ is larger than the value currently maintained by
     * the forwarding node.
     */
    if (rreq->flags.U || (toDest.validDestSeqFlag && aodvCompareSeqNumber(toDest.destSeqNumber, rreq->destSeqNumber))) {
      if (!rreq->flags.D && toDest.valid) {
        toOrigin = routingTableFindEntry(routingTable, rreq->origAddress);
        sendRREPByIntermediateNode(toDest, toOrigin);
        return;
      }
      rreq->destSeqNumber = toDest.destSeqNumber;
      rreq->flags.U = false;
    }

    DEBUG_PRINT("aodvProcessRREQ: %u forward RREQ from origin = %u, reqId = %lu, src = %u, dest = %u, destSeq = %lu.\n",
                uwbGetAddress(),
                rreq->origAddress,
                rreq->requestId,
                packet->header.srcAddress,
                rreq->destAddress,
                rreq->destSeqNumber
    );
    /* Forward this RREQ message. */
    packet->header.srcAddress = uwbGetAddress();
    packet->header.destAddress = UWB_DEST_ANY;
    uwbSendPacketBlock(packet);
  }
}

static void aodvProcessRREP(UWB_Packet_t *packet) {
  AODV_RREP_Message_t *rrep = (AODV_RREP_Message_t *) &packet->payload;
  DEBUG_PRINT("aodvProcessRREP: %u received RREP from %u, origin = %u, dest = %u, hop = %u, expire at %lu.\n",
              uwbGetAddress(),
              packet->header.srcAddress,
              rrep->origAddress,
              rrep->destAddress,
              rrep->hopCount,
              rrep->lifetime
  );
  rrep->hopCount++;
  /* This RREP is a hello message */
  if (rrep->origAddress == rrep->destAddress) {
    DEBUG_PRINT("aodvProcessRREP: %u received hello from %u.\n", uwbGetAddress(), packet->header.srcAddress);
    aodvProcessHello(rrep);
    return;
  }
  /* Check whether this RREP is fresh, which needs ntp or other global timer, cannot implement here. */
//  Time_t curTime = xTaskGetTickCount();
//  if (rrep->lifetime < curTime) {
//    DEBUG_PRINT("aodvProcessRREP: %u received stale RREP from %u, origin = %u, dest = %u, expire at %lu.\n",
//                uwbGetAddress(),
//                packet->header.srcAddress,
//                rrep->origAddress,
//                rrep->destAddress,
//                rrep->lifetime
//    );
//    return;
//  }
  /*
   * If the route table entry to the destination is created or updated, then the following actions
   * occur:
   * -  the route is marked as active,
   * -  the destination sequence number is marked as valid,
   * -  the next hop in the route entry is assigned to be the node from which the RREP is
   * received, which is indicated by the source UWB address field in the UWB packet header,
   * -  the hop count is set to the value of the hop count from RREP message + 1
   * -  the expiry time is set to the current time plus the value of the Lifetime in the RREP
   * message,
   * -  and the destination sequence number is the Destination Sequence Number in the RREP
   * message.
   */
  Route_Entry_t newEntry = {
      .type = ROUTE_AODV,
      .destAddress = rrep->destAddress,
      .valid = true,
      .validDestSeqFlag = true,
      .destSeqNumber = rrep->destSeqNumber,
      .hopCount = rrep->hopCount,
      .nextHop = packet->header.srcAddress,
      .expirationTime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME)
  };
  Route_Entry_t toDest = routingTableFindEntry(routingTable, rrep->destAddress);
  if (toDest.destAddress != UWB_DEST_EMPTY) {
    /*
     * The existing entry is updated only in the following circumstances:
     * (i) the sequence number in the routing table is marked as invalid in route table entry.
     */
    if (!toDest.validDestSeqFlag) {
      routingTableUpdateEntry(routingTable, newEntry);
    } else if (aodvCompareSeqNumber(rrep->destSeqNumber, toDest.destSeqNumber)) {
      /* (ii) the Destination Sequence Number in the RREP is greater than the node's copy of the
       * destination sequence number and the known value is valid.
       */
      routingTableUpdateEntry(routingTable, newEntry);
    } else {
      /* (iii) the sequence numbers are the same, but the route is marked as inactive. */
      if (rrep->destSeqNumber == toDest.destSeqNumber && !toDest.valid) {
        routingTableUpdateEntry(routingTable, newEntry);
      } else if (rrep->destSeqNumber == toDest.destSeqNumber && rrep->hopCount < toDest.hopCount) {
        /* (iv) the sequence numbers are the same, and the New Hop Count is smaller than the hop count
         * in route table entry.
         */
        routingTableUpdateEntry(routingTable, newEntry);
      }
    }
  } else {
    /* The forward route for this destination is created if it does not already exist. */
    routingTableAddEntry(routingTable, newEntry);
  }
  /* Acknowledge receipt of the RREP by sending a RREP-ACK message back. */
  if (rrep->flags.A) {
    sendRREPACK(packet->header.srcAddress);
    rrep->flags.A = false;
  }
  Route_Entry_t toOrigin = routingTableFindEntry(routingTable, rrep->origAddress);
  if (toOrigin.destAddress == UWB_DEST_EMPTY) {
    DEBUG_PRINT("aodvProcessRREP: Impossible, just drop the RREP.\n");
    return;
  }
  toOrigin.expirationTime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME);
  routingTableUpdateEntry(routingTable, toOrigin);

  /* Update precursors */
  toDest = routingTableFindEntry(routingTable, rrep->destAddress);
  if (toDest.valid) {
    toDest.precursors = precursorListAdd(toDest.precursors, toOrigin.nextHop);
    routingTableUpdateEntry(routingTable, toDest);

    Route_Entry_t toNextHopToDest = routingTableFindEntry(routingTable, toDest.nextHop);
    if (toNextHopToDest.destAddress == UWB_DEST_EMPTY) {
      DEBUG_PRINT("aodvProcessRREP: Empty toNextHopToDest.\n");
    } else {
      toNextHopToDest.precursors = precursorListAdd(toNextHopToDest.precursors, toOrigin.nextHop);
      routingTableUpdateEntry(routingTable, toNextHopToDest);
    }

    toOrigin.precursors = precursorListAdd(toOrigin.precursors, toDest.nextHop);
    routingTableUpdateEntry(routingTable, toOrigin);

    Route_Entry_t toNextHopToOrigin = routingTableFindEntry(routingTable, toOrigin.nextHop);
    if (toNextHopToOrigin.destAddress == UWB_DEST_EMPTY) {
      DEBUG_PRINT("aodvProcessRREP: Empty toNextHopToOrigin.\n");
    } else {
      toNextHopToOrigin.precursors = precursorListAdd(toNextHopToOrigin.precursors, toDest.nextHop);
      routingTableUpdateEntry(routingTable, toNextHopToOrigin);
    }
  }

  DEBUG_PRINT("aodvProcessRREP: %u forward RREQ from src = %u, origin = %u, dest = %u, destSeq = %lu, hop = %u.\n",
              uwbGetAddress(),
              packet->header.srcAddress,
              rrep->origAddress,
              rrep->destAddress,
              rrep->destSeqNumber,
              rrep->hopCount
  );
  /* Forward this RREP message. */
  packet->header.srcAddress = uwbGetAddress();
  packet->header.destAddress = toOrigin.nextHop;
  uwbSendPacketBlock(packet);
}

static void aodvProcessRERR(UWB_Packet_t *packet) {
  AODV_RERR_Message_t *rerr = (AODV_RERR_Message_t *) &packet->payload;
  DEBUG_PRINT("aodvProcessRERR: %u Received RERR from neighbor %u, err dest count = %u.\n",
              uwbGetAddress(),
              packet->header.srcAddress,
              rerr->destCount
  );

  /* We don't uni-cast RERR to each corresponding precursors here, instead, we just broadcast it to all one-hop
   * neighbors. Each neighbor checks if it is the effected node (mostly the precursor of the packet.srcAddress).
   * If it has the local route to the unreachable.destAddress, delete the effected route entries and lastly forward the
   * RERR, or not effected then just ignore this RERR to prevent broadcast storm. This approach differs from the RFC.
   */
  int dropCount = 0;
  for (int i = 0; i < rerr->destCount; i++) {
    /* Drop all route dest in unreachable list and route entries that next hop set to unreachable. */
    Unreachable_Dest_t unreachable = rerr->unreachableList[i];
    int index = routingTableSearchEntry(routingTable, unreachable.destAddress);
    if (index != -1 && aodvCompareSeqNumber(routingTable->entries[index].destSeqNumber, unreachable.destSeqNumber) <= 0) {
      routingTable->entries[index] = emptyRouteEntry();
      dropCount++;
      for (int j = 0; j < routingTable->size; j++) {
        if (routingTable->entries[j].destAddress != UWB_DEST_EMPTY
            && routingTable->entries[j].nextHop == unreachable.destAddress) {
          routingTable->entries[j] = emptyRouteEntry();
          dropCount++;
        }
      }
    }
  }
  if (dropCount > 0) {
    DEBUG_PRINT("aodvProcessRERR: %u Received RERR from neighbor %u, drop %d routes.\n",
                uwbGetAddress(),
                packet->header.srcAddress,
                dropCount
    );
    routingTableSort(routingTable);
    routingTable->size -= dropCount;
    packet->header.srcAddress = uwbGetAddress();
    packet->header.destAddress = UWB_DEST_ANY;
    uwbSendPacketBlock(packet);
  }
}

static void aodvProcessRREPACK(UWB_Packet_t *packet) {
  AODV_RREP_ACK_Message_t *rrepAck = (AODV_RREP_ACK_Message_t *) &packet->payload;
  Route_Entry_t toNeighbor = routingTableFindEntry(routingTable, packet->header.srcAddress);
  if (toNeighbor.destAddress != UWB_DEST_EMPTY) {
    toNeighbor.valid = true;
    routingTableUpdateEntry(routingTable, toNeighbor);
    DEBUG_PRINT("aodvProcessRREPACK: %u received RREP_ACK from neighbor %u, mark route as valid.\n",
                uwbGetAddress(),
                packet->header.srcAddress);
  }
}

// TODO: rate limiter
void aodvDiscoveryRoute(UWB_Address_t destAddress) {
  DEBUG_PRINT("aodvDiscoveryRoute: Try to discovery route to %u.\n", destAddress);
  UWB_Packet_t packet;
  packet.header.type = UWB_AODV_MESSAGE;
  packet.header.srcAddress = uwbGetAddress();
  packet.header.destAddress = UWB_DEST_ANY;
  packet.header.length = sizeof(UWB_Packet_Header_t) + sizeof(AODV_RREQ_Message_t);

  AODV_RREQ_Message_t *rreqMsg = (AODV_RREQ_Message_t *) &packet.payload;
  rreqMsg->type = AODV_RREQ;
  rreqMsg->hopCount = 0;
  rreqMsg->requestId = aodvRequestId++;
  rreqMsg->origAddress = uwbGetAddress();
  rreqMsg->origSeqNumber = aodvSeqNumber++;
  rreqMsg->destAddress = destAddress;

  Route_Entry_t routeEntry = routingTableFindEntry(routingTable, destAddress);
  /* Find corresponding route entry for destAddress. */
  if (routeEntry.destAddress != UWB_DEST_EMPTY) {
    if (routeEntry.validDestSeqFlag) {
      rreqMsg->destSeqNumber = routeEntry.destSeqNumber;
    } else {
      rreqMsg->destSeqNumber = 0;
      rreqMsg->flags.U = true;
    }
    routeEntry.expirationTime = xTaskGetTickCount() + M2T(AODV_ROUTE_DISCOVERY_TIME);
    routingTableUpdateEntry(routingTable, routeEntry);
  } else {
    rreqMsg->flags.U = true;
    rreqMsg->destSeqNumber = 0;
    /* Add new route entry for destAddress. */
    Route_Entry_t newRouteEntry = emptyRouteEntry();
    newRouteEntry.type = ROUTE_AODV;
    newRouteEntry.destAddress = destAddress;
    newRouteEntry.expirationTime = xTaskGetTickCount() + M2T(AODV_ROUTE_DISCOVERY_TIME);
    routingTableAddEntry(routingTable, newRouteEntry);
  }

  if (AODV_GRATUITOUS_REPLY) {
    rreqMsg->flags.G = true;
  }

  if (AODV_DESTINATION_ONLY) {
    rreqMsg->flags.D = true;
  }

  DEBUG_PRINT("aodvDiscoveryRoute: Send aodv rreq, reqId = %lu, orig = %u, origSeq = %lu, dest = %u, destSeq = %lu.\n",
              rreqMsg->requestId,
              rreqMsg->origAddress,
              rreqMsg->origSeqNumber,
              rreqMsg->destAddress,
              rreqMsg->destSeqNumber
  );

  uwbSendPacketBlock(&packet);
}

void aodvSendRERR(Unreachable_Dest_t *unreachableList, int count) {
  DEBUG_PRINT("aodvDiscoveryRoute: Send RERR about %d invalid routes.\n", count);
  UWB_Packet_t packet;
  packet.header.type = UWB_AODV_MESSAGE;
  packet.header.srcAddress = uwbGetAddress();
  packet.header.destAddress = UWB_DEST_ANY;
  packet.header.length = sizeof(UWB_Packet_Header_t) + sizeof(AODV_RERR_Message_t);
  AODV_RERR_Message_t *rerr = (AODV_RERR_Message_t *) &packet.payload;
  rerr->type = AODV_RERR;
  rerr->destCount = 0;
  int next = 0;
  int round = (count + AODV_RERR_UNREACHABLE_DEST_SIZE_MAX - 1) / AODV_RERR_UNREACHABLE_DEST_SIZE_MAX;
  // (count + AODV_RERR_UNREACHABLE_DEST_SIZE_MAX - 1) / AODV_RERR_UNREACHABLE_DEST_SIZE_MAX = ceil(count / AODV_RERR_UNREACHABLE_DEST_SIZE_MAX)
  for (int k = 0; k < round; k++) {
    int size = MIN(count, AODV_RERR_UNREACHABLE_DEST_SIZE_MAX);
    rerr->destCount = size;
    for (int j = 0; j < size; j++) {
      rerr->unreachableList[j] = unreachableList[next];
      next++;
      count--;
    }
    DEBUG_PRINT("aodvSendRERR: Send RERR msg contains %d unreachable dest.\n", size);
    uwbSendPacketBlock(&packet);
  }
}

void aodvRxCallback(void *parameters) {
//  DEBUG_PRINT("aodvRxCallback\n");
}

void aodvTxCallback(void *parameters) {
//  DEBUG_PRINT("aodvTxCallback\n");
}

static void aodvRxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t rxPacketCache;

  while (true) {
    if (uwbReceivePacketBlock(UWB_AODV_MESSAGE, &rxPacketCache)) {
      xSemaphoreTake(routingTable->mu, portMAX_DELAY);
      DEBUG_PRINT("aodvRxTask: receive aodv message from neighbor %u.\n", rxPacketCache.header.srcAddress);

      /* Update route for Me to Neighbor (reverse route) */
      Route_Entry_t toNeighbor = routingTableFindEntry(routingTable, rxPacketCache.header.srcAddress);
      if (toNeighbor.destAddress != UWB_DEST_EMPTY && toNeighbor.validDestSeqFlag && toNeighbor.hopCount == 1) {
        toNeighbor.expirationTime = MAX(toNeighbor.expirationTime, xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME));
        routingTableUpdateEntry(routingTable, toNeighbor);
      } else {
        toNeighbor.type = ROUTE_AODV,
        toNeighbor.valid = true;
        toNeighbor.destAddress = rxPacketCache.header.srcAddress;
        toNeighbor.nextHop = rxPacketCache.header.srcAddress;
        toNeighbor.destSeqNumber = 0;
        toNeighbor.hopCount = 1;
        toNeighbor.validDestSeqFlag = false;
        toNeighbor.expirationTime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME);
        routingTableAddEntry(routingTable, toNeighbor);
      }

      /* Dispatch message to corresponding handler. */
      uint8_t msgType = rxPacketCache.payload[0];
      switch (msgType) {
        case AODV_RREQ:aodvProcessRREQ(&rxPacketCache);
          break;
        case AODV_RREP:aodvProcessRREP(&rxPacketCache);
          break;
        case AODV_RERR:aodvProcessRERR(&rxPacketCache);
          break;
        case AODV_RREP_ACK:aodvProcessRREPACK(&rxPacketCache);
          break;
        default:DEBUG_PRINT("aodvRxTask: Receive unknown aodv message type %u from %u, discard.\n", msgType,
                            rxPacketCache.header.srcAddress);
      }
      xSemaphoreGive(routingTable->mu);
    }
    vTaskDelay(M2T(1));
  }

}

void aodvRouteExpirationHook(UWB_Address_t *addresses, int count) {
  DEBUG_PRINT("aodvRouteExpirationHook\n");
  Unreachable_Dest_t unreachableList[count];
  for (int i = 0; i < count; i++) {
    Route_Entry_t expired = routingTableFindEntry(routingTable, addresses[i]);
    if (expired.type != ROUTE_AODV) {
      continue;
    }
    unreachableList[i].destAddress = addresses[i];
    unreachableList[i].destSeqNumber = expired.destSeqNumber;
  }
  aodvSendRERR(unreachableList, count);
}

void aodvInit() {
  rxQueue = xQueueCreate(AODV_RX_QUEUE_SIZE, AODV_RX_QUEUE_ITEM_SIZE);
  rreqBufferInit(&rreqBuffer);
  routingTable = getGlobalRoutingTable();
  routingTableRegisterExpirationHook(routingTable, aodvRouteExpirationHook);
  #ifdef AODV_ENABLE_HELLO
  aodvHelloTimer = xTimerCreate("aodvHelloTimer",
                                M2T(AODV_HELLO_INTERVAL),
                                pdTRUE,
                                (void *) 0,
                                aodvHelloTimerCallback);
  xTimerStart(aodvHelloTimer, M2T(0));
  #endif

  UWB_Message_Listener_t listener;
  listener.type = UWB_AODV_MESSAGE;
  listener.rxQueue = rxQueue;
  listener.rxCb = aodvRxCallback;
  listener.txCb = aodvTxCallback;
  uwbRegisterListener(&listener);

  xTaskCreate(aodvRxTask,
              ADHOC_DECK_AODV_RX_TASK_NAME,
              UWB_TASK_STACK_SIZE,
              NULL,
              ADHOC_DECK_TASK_PRI,
              &aodvRxTaskHandle);
}
