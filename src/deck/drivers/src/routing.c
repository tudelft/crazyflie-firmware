#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

#include "autoconf.h"
#include "debug.h"
#include "log.h"
#include "system.h"
#include "routing.h"
#include "olsr.h"
#include "aodv.h"

#ifndef ROUTING_DEBUG_ENABLE
  #undef DEBUG_PRINT
  #define DEBUG_PRINT
#endif

static TaskHandle_t uwbRoutingTxTaskHandle = 0;
static TaskHandle_t uwbRoutingRxTaskHandle = 0;
static QueueHandle_t txQueue;
static QueueHandle_t rxQueue;
static QueueHandle_t txBufferQueue;
static SemaphoreHandle_t txBufferMutex;
static TimerHandle_t txBufferEvictionTimer;
static TimerHandle_t routeEvictionTimer;
static xQueueHandle queues[UWB_DATA_MESSAGE_TYPE_COUNT];
static UWB_Data_Packet_Listener_t listeners[UWB_DATA_MESSAGE_TYPE_COUNT];
static Routing_Table_t routingTable;
static int routingSeqNumber = 1;

static Route_Entry_t EMPTY_ROUTE_ENTRY = {
    .type = ROUTE_RESERVED,
    .destAddress = UWB_DEST_EMPTY,
    .nextHop = UWB_DEST_EMPTY,
    .hopCount = 1,
    .expirationTime = 0,
    .destSeqNumber = 0,
    .validDestSeqFlag = false,
    .precursors = 0,
    // TODO: init metrics
};

static void routingTableSwapRouteEntry(Routing_Table_t *table, int first, int second) {
  Route_Entry_t temp = table->entries[first];
  table->entries[first] = table->entries[second];
  table->entries[second] = temp;
}

static void bufferDataPacket(UWB_Data_Packet_t *packet) {
  Time_t evictTime = xTaskGetTickCount() + M2T(ROUTING_TX_BUFFER_QUEUE_ITEM_HOLD_TIME);
  UWB_Data_Packet_With_Timestamp_t bufferedPacket = {.packet = *packet, .evictTime = evictTime};
  DEBUG_PRINT("bufferDataPacket: Try to buffer data packet src = %u, dest = %u, seq = %lu.\n",
              packet->header.srcAddress,
              packet->header.destAddress,
              packet->header.seqNumber);
  if (xQueueSend(txBufferQueue, &bufferedPacket, M2T(0)) == pdFALSE) {
    UWB_Data_Packet_With_Timestamp_t evictedPacket;
    xQueueReceive(txBufferQueue, &evictedPacket, M2T(0));
    DEBUG_PRINT("bufferDataPacket: Buffer is full, evict oldest one that dest to %u, seq = %lu.\n",
                evictedPacket.packet.header.destAddress,
                evictedPacket.packet.header.seqNumber);
  }
  DEBUG_PRINT("bufferDataPacket: Buffer packet dest to %u, seq = %lu.\n",
              bufferedPacket.packet.header.destAddress,
              bufferedPacket.packet.header.seqNumber);
}

static void evictDataPacketTimerCallback(TimerHandle_t timer) {
  xSemaphoreTake(txBufferMutex, portMAX_DELAY);

  DEBUG_PRINT("evictDataPacketTimerCallback: Trigger eviction timer at %lu.\n", xTaskGetTickCount());
  UWB_Data_Packet_With_Timestamp_t evictedPacket;
  Time_t curTime = xTaskGetTickCount();
  bool evicted = false;
  while (xQueuePeek(txBufferQueue, &evictedPacket, M2T(0)) && evictedPacket.evictTime < curTime) {
    /* Discard this packet */
    xQueueReceive(txBufferQueue, &evictedPacket, M2T(0));
    DEBUG_PRINT("evictDataPacketTimerCallback: Evict dest to %u, seq = %lu.\n",
                evictedPacket.packet.header.destAddress,
                evictedPacket.packet.header.seqNumber);
    evicted = true;
  }
  if (!evicted) {
    DEBUG_PRINT("evictDataPacketTimerCallback: Evict none.\n");
  }

  xSemaphoreGive(txBufferMutex);
}

int routingTableClearExpire(Routing_Table_t *table) {
  Time_t curTime = xTaskGetTickCount();
  int evictionCount = 0;
  for (int p1 = 0, p2 = table->size - 1; p1 <= p2;) {
    if (table->entries[p1].expirationTime != 0 && table->entries[p1].expirationTime <= curTime) {
      DEBUG_PRINT("routingTableClearExpire: Clean route entry for neighbor %u that expire at %lu.\n",
                  table->entries[p1].destAddress,
                  table->entries[p1].expirationTime);
      routingTableSwapRouteEntry(table, p1, p2);
      p2--;
      evictionCount++;
    } else {
      p1++;
    }
  }
  if (evictionCount > 0) {
    UWB_Address_t addresses[evictionCount];
    for (int i = 0; i < evictionCount; i++) {
      addresses[i] = table->entries[table->size - 1 - i].destAddress;
      table->entries[table->size - 1 - i] = EMPTY_ROUTE_ENTRY;
    }
    routeExpirationHooksInvoke(&table->expirationHooks, addresses, evictionCount);
  }
  table->size -= evictionCount;
  /* Keeps routing table in order. */
  routingTableSort(table);

  return evictionCount;
}

static void evictRouteTimerCallback(TimerHandle_t timer) {
  xSemaphoreTake(routingTable.mu, portMAX_DELAY);

  Time_t curTime = xTaskGetTickCount();
  DEBUG_PRINT("evictRouteTimerCallback: Trigger expiration timer at %lu.\n", curTime);

  int evictionCount = routingTableClearExpire(&routingTable);
  if (evictionCount > 0) {
    DEBUG_PRINT("evictRouteTimerCallback: Evict total %d routing table entries.\n", evictionCount);
  } else {
    DEBUG_PRINT("evictRouteTimerCallback: Evict none.\n");
  }

  xSemaphoreGive(routingTable.mu);
}

static void routingTableUpdateExpirationTime(Routing_Table_t *table, UWB_Address_t address) {
  int index = routingTableSearchEntry(table, address);
  if (index != -1) {
    routingTable.entries[index].expirationTime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME);
  }
}

void routingRxCallback(void *parameters) {
  UWB_Packet_t *uwbPacket = (UWB_Packet_t *) parameters;
  UWB_Data_Packet_t *uwbDataPacket = (UWB_Data_Packet_t *) &uwbPacket->payload;
  ASSERT(uwbDataPacket->header.type < UWB_DATA_MESSAGE_TYPE_COUNT);
  if (listeners[uwbDataPacket->header.type].rxCb) {
    listeners[uwbDataPacket->header.type].rxCb(uwbDataPacket);
  }
}

void routingTxCallback(void *parameters) {
  UWB_Packet_t *uwbPacket = (UWB_Packet_t *) parameters;
  UWB_Data_Packet_t *uwbDataPacket = (UWB_Data_Packet_t *) &uwbPacket->payload;
  ASSERT(uwbDataPacket->header.type < UWB_DATA_MESSAGE_TYPE_COUNT);
  if (listeners[uwbDataPacket->header.type].txCb) {
    listeners[uwbDataPacket->header.type].txCb(uwbDataPacket);
  }
}

static void uwbRoutingTxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t uwbTxPacketCache;
  uwbTxPacketCache.header.srcAddress = uwbGetAddress();
  uwbTxPacketCache.header.destAddress = UWB_DEST_EMPTY;
  uwbTxPacketCache.header.type = UWB_DATA_MESSAGE;
  uwbTxPacketCache.header.length = 0;

  UWB_Data_Packet_t *uwbTxDataPacketCache = (UWB_Data_Packet_t *) &uwbTxPacketCache.payload;
  uwbTxDataPacketCache->header.length = 0;
  uwbTxDataPacketCache->header.ttl = 1;

  UWB_Data_Packet_With_Timestamp_t dataTxPacketBufferCache;
  while (true) {
    /* Consume tx queue */
    if (xQueueReceive(txQueue, uwbTxDataPacketCache, M2T(ROUTING_TX_QUEUE_WAIT_TIME))) {
      ASSERT(uwbTxDataPacketCache->header.type < UWB_DATA_MESSAGE_TYPE_COUNT);
      ASSERT(uwbTxDataPacketCache->header.length <= ROUTING_DATA_PACKET_SIZE_MAX);
      xSemaphoreTake(routingTable.mu, portMAX_DELAY);
      xSemaphoreTake(txBufferMutex, portMAX_DELAY);
      /* Data packet that originate from self. */
      if (uwbTxDataPacketCache->header.srcAddress == uwbGetAddress()) {
        uwbTxDataPacketCache->header.seqNumber = routingSeqNumber++;
      }
      dataTxPacketBufferCache.packet.header.ttl--;
      Time_t curTime = xTaskGetTickCount();
      Route_Entry_t toDest = routingTableFindEntry(&routingTable, uwbTxDataPacketCache->header.destAddress);
      UWB_Address_t nextHopToDest = toDest.destAddress;
      /* Update expiration time of each route to originator & sender (neighbor)*/
      routingTableUpdateExpirationTime(&routingTable, uwbTxDataPacketCache->header.srcAddress);
      routingTableUpdateExpirationTime(&routingTable, uwbTxPacketCache.header.srcAddress);
      if (uwbTxDataPacketCache->header.destAddress == uwbGetAddress()) {
//        DEBUG_PRINT("uwbRoutingTxTask: Send data packet dest to self.\n");
        xSemaphoreGive(txBufferMutex);
        xSemaphoreGive(routingTable.mu);
        xQueueSend(rxQueue, &uwbTxPacketCache, portMAX_DELAY);
      } else if (dataTxPacketBufferCache.packet.header.ttl > 0) {
//        DEBUG_PRINT("uwbRoutingTxTask: dataTxPacketBufferCache.packet.header.ttl > 0.\n");
//        printRouteEntry(&toDest);
//        printRoutingTable(&routingTable);
        /* Unknown dest, start route discovery procedure */
        if (nextHopToDest == UWB_DEST_EMPTY || !toDest.valid) {
          /* Buffer this data packet since there is no certain route to dest */
          bufferDataPacket(uwbTxDataPacketCache);
          /* Then trigger route discovery */
          #ifdef ROUTING_AODV_ENABLE
          DEBUG_PRINT("uwbRoutingTxTask: %u Unknown dest %u, start route discovery.\n",
                      uwbGetAddress(),
                      uwbTxDataPacketCache->header.destAddress);
          aodvDiscoveryRoute(uwbTxDataPacketCache->header.destAddress); // TODO: rate limiter
          #endif
        } else {
//          printRouteEntry(&toDest);
          /* Update expiration time of next hop neighbor. */
          routingTableUpdateExpirationTime(&routingTable, nextHopToDest);
          /* Populate mac layer dest address */
          uwbTxPacketCache.header.srcAddress = uwbGetAddress();
          uwbTxPacketCache.header.destAddress = toDest.nextHop;
          uwbTxPacketCache.header.type = UWB_DATA_MESSAGE;
          uwbTxPacketCache.header.length = sizeof(UWB_Packet_Header_t) + uwbTxDataPacketCache->header.length;
          DEBUG_PRINT(
              "uwbRoutingTxTask: %u send type = %u, len = %u, origin = %u, dest = %u, seq = %lu, ttl = %u.\n",
              uwbGetAddress(),
              uwbTxDataPacketCache->header.type,
              uwbTxDataPacketCache->header.length,
              uwbTxDataPacketCache->header.srcAddress,
              uwbTxDataPacketCache->header.destAddress,
              uwbTxDataPacketCache->header.seqNumber,
              uwbTxDataPacketCache->header.ttl
          );
          /* Update expiration time of dest */
          routingTableUpdateExpirationTime(&routingTable, toDest.destAddress);
          /* Forward */
          uwbSendPacketBlock(&uwbTxPacketCache); // TODO: rate limiter
        }
        xSemaphoreGive(txBufferMutex);
        xSemaphoreGive(routingTable.mu);
      } else {
        DEBUG_PRINT("uwbRoutingTxTask: %u discards packet type = %u, origin = %u, dest = %u, seq = %lu, ttl = %u.\n",
                    uwbGetAddress(),
                    uwbTxDataPacketCache->header.type,
                    uwbTxDataPacketCache->header.srcAddress,
                    uwbTxDataPacketCache->header.destAddress,
                    uwbTxDataPacketCache->header.seqNumber,
                    uwbTxDataPacketCache->header.ttl);
        xSemaphoreGive(txBufferMutex);
        xSemaphoreGive(routingTable.mu);
      }
    }
    // TODO: rate limiter
    /* Try to consume valid tx buffer queue item */
    if (xQueuePeek(txBufferQueue, &dataTxPacketBufferCache, M2T(0))) {
      xSemaphoreTake(routingTable.mu, portMAX_DELAY);
      xSemaphoreTake(txBufferMutex, portMAX_DELAY);

      Time_t curTime = xTaskGetTickCount();
      Route_Entry_t
          routeEntry = routingTableFindEntry(&routingTable, dataTxPacketBufferCache.packet.header.destAddress);
      UWB_Address_t nextHopToDest = routeEntry.destAddress;
      /* Consume if packet is not stale and have found corresponding next hop to dest. */
      if (nextHopToDest != EMPTY_ROUTE_ENTRY.destAddress
          && curTime < routeEntry.expirationTime
          && curTime < dataTxPacketBufferCache.evictTime
          && (routeEntry.valid)) {
        /* Dequeue */
        xQueueReceive(txBufferQueue, &dataTxPacketBufferCache, M2T(0));
        /* Forward */
        uwbTxPacketCache.header.srcAddress = uwbGetAddress();
        uwbTxPacketCache.header.destAddress = routeEntry.nextHop;
        uwbTxPacketCache.header.type = UWB_DATA_MESSAGE;
        uwbTxPacketCache.header.length = sizeof(UWB_Packet_Header_t) + dataTxPacketBufferCache.packet.header.length;
        memcpy(uwbTxPacketCache.payload, &dataTxPacketBufferCache.packet, dataTxPacketBufferCache.packet.header.length);
        DEBUG_PRINT(
            "uwbRoutingTxTask: %u consumes buffered data packet: type = %u, len = %d, origin = %u, seq = %lu, dest = %u, ttl = %u.\n",
            uwbGetAddress(),
            dataTxPacketBufferCache.packet.header.type,
            dataTxPacketBufferCache.packet.header.length,
            dataTxPacketBufferCache.packet.header.srcAddress,
            dataTxPacketBufferCache.packet.header.seqNumber,
            dataTxPacketBufferCache.packet.header.destAddress,
            dataTxPacketBufferCache.packet.header.ttl);
        uwbSendPacketBlock(&uwbTxPacketCache);
      }

      xSemaphoreGive(txBufferMutex);
      xSemaphoreGive(routingTable.mu);
    }

    vTaskDelay(M2T(1)); // TODO: rate limiter
  }
}

static void uwbRoutingRxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t uwbRxPacketCache;
  uwbRxPacketCache.header.type = UWB_DATA_MESSAGE;
  uwbRxPacketCache.header.length = 0;

  UWB_Data_Packet_t *uwbRxDataPacketCache = (UWB_Data_Packet_t *) &uwbRxPacketCache.payload;
  uwbRxDataPacketCache->header.type = UWB_DATA_MESSAGE_RESERVED;
  uwbRxDataPacketCache->header.length = 0;

  while (true) {
    if (uwbReceivePacketBlock(UWB_DATA_MESSAGE, &uwbRxPacketCache)) {
      ASSERT(uwbRxDataPacketCache->header.type < UWB_DATA_MESSAGE_TYPE_COUNT);
      ASSERT(uwbRxDataPacketCache->header.length <= ROUTING_DATA_PACKET_SIZE_MAX);
//      DEBUG_PRINT(
//          "uwbRoutingRxTask: RX type = %u, len = %u, origin = %u, dest = %u, seq = %lu, ttl = %u.\n",
//          uwbRxDataPacketCache->header.type,
//          uwbRxDataPacketCache->header.length,
//          uwbRxDataPacketCache->header.srcAddress,
//          uwbRxDataPacketCache->header.destAddress,
//          uwbRxDataPacketCache->header.seqNumber,
//          uwbRxDataPacketCache->header.ttl
//      );
      if (uwbRxDataPacketCache->header.destAddress == UWB_DEST_ANY
          || uwbRxDataPacketCache->header.destAddress == uwbGetAddress()) {
        /* Dispatch Data Message */
        if (listeners[uwbRxDataPacketCache->header.type].rxQueue) {
          if (xQueueSend(listeners[uwbRxDataPacketCache->header.type].rxQueue, uwbRxDataPacketCache, M2T(1000))
              != pdPASS) {
            DEBUG_PRINT("uwbRoutingRxTask: Timeout when dispatch data message type %d.\n",
                        uwbRxDataPacketCache->header.type);
          }
        }
      } else {
        /* Forward the packet */
        uwbSendDataPacketBlock(uwbRxDataPacketCache);
      }
    }
    vTaskDelay(M2T(1)); // TODO: rate limiter
  }
}

void routingInit() {
  txQueue = xQueueCreate(ROUTING_TX_QUEUE_SIZE, ROUTING_TX_QUEUE_ITEM_SIZE);
  rxQueue = xQueueCreate(ROUTING_RX_QUEUE_SIZE, ROUTING_RX_QUEUE_ITEM_SIZE);
  txBufferQueue = xQueueCreate(ROUTING_TX_BUFFER_QUEUE_SIZE, ROUTING_TX_BUFFER_QUEUE_ITEM_SIZE);
  txBufferMutex = xSemaphoreCreateMutex();
  txBufferEvictionTimer = xTimerCreate("txBufferEvictionTimer",
                                       M2T(ROUTING_TX_BUFFER_QUEUE_ITEM_HOLD_TIME / 2),
                                       pdTRUE,
                                       (void *) 0,
                                       evictDataPacketTimerCallback);
  xTimerStart(txBufferEvictionTimer, M2T(0));
  routeEvictionTimer = xTimerCreate("routeEvictionTimer",
                                    M2T(ROUTING_TABLE_HOLD_TIME / 2),
                                    pdTRUE,
                                    (void *) 0,
                                    evictRouteTimerCallback);
  xTimerStart(routeEvictionTimer, M2T(0));

  routingTableInit(&routingTable);

  UWB_Message_Listener_t listener;
  listener.type = UWB_DATA_MESSAGE;
  listener.rxQueue = rxQueue;
  listener.rxCb = routingRxCallback;
  listener.txCb = routingTxCallback;
  uwbRegisterListener(&listener);

  xTaskCreate(uwbRoutingTxTask,
              ADHOC_DECK_ROUTING_TX_TASK_NAME,
              UWB_TASK_STACK_SIZE,
              NULL,
              ADHOC_DECK_TASK_PRI,
              &uwbRoutingTxTaskHandle);
  xTaskCreate(uwbRoutingRxTask,
              ADHOC_DECK_ROUTING_RX_TASK_NAME,
              UWB_TASK_STACK_SIZE,
              NULL,
              ADHOC_DECK_TASK_PRI,
              &uwbRoutingRxTaskHandle);
  #ifdef ROUTING_AODV_ENABLE
  aodvInit();
  #endif
  #ifdef ROUTING_OLSR_ENABLE
  olsrInit();
  #endif
}

/* Messaging Operations */
int uwbSendDataPacket(UWB_Data_Packet_t *packet) {
  ASSERT(packet);
  return xQueueSend(txQueue, packet, 0);
}

int uwbSendDataPacketBlock(UWB_Data_Packet_t *packet) {
  ASSERT(packet);
  return xQueueSend(txQueue, packet, portMAX_DELAY);
}

int uwbSendDataPacketWait(UWB_Data_Packet_t *packet, int wait) {
  ASSERT(packet);
  return xQueueSend(txQueue, packet, M2T(wait));
}

int uwbReceiveDataPacket(UWB_DATA_MESSAGE_TYPE type, UWB_Data_Packet_t *packet) {
  ASSERT(packet);
  ASSERT(type < UWB_DATA_MESSAGE_TYPE_COUNT);
  return xQueueReceive(queues[type], packet, 0);
}

int uwbReceiveDataPacketBlock(UWB_DATA_MESSAGE_TYPE type, UWB_Data_Packet_t *packet) {
  ASSERT(packet);
  ASSERT(type < UWB_DATA_MESSAGE_TYPE_COUNT);
  return xQueueReceive(queues[type], packet, portMAX_DELAY);
}

int uwbReceiveDataPacketWait(UWB_DATA_MESSAGE_TYPE type, UWB_Data_Packet_t *packet, int wait) {
  ASSERT(packet);
  ASSERT(type < UWB_DATA_MESSAGE_TYPE_COUNT);
  return xQueueReceive(queues[type], packet, M2T(wait));
}

void uwbRegisterDataPacketListener(UWB_Data_Packet_Listener_t *listener) {
  ASSERT(listener->type < UWB_DATA_MESSAGE_TYPE_COUNT);
  queues[listener->type] = listener->rxQueue;
  listeners[listener->type] = *listener;
}

/* Routing Table Operations */
Route_Entry_t emptyRouteEntry() {
  return EMPTY_ROUTE_ENTRY;
}

Routing_Table_t *getGlobalRoutingTable() {
  return &routingTable;
}

void routingTableInit(Routing_Table_t *table) {
  table->size = 0;
  table->mu = xSemaphoreCreateMutex();
  for (int i = 0; i < ROUTING_TABLE_SIZE_MAX; i++) {
    table->entries[i] = EMPTY_ROUTE_ENTRY;
  }
  table->expirationHooks.hook = NULL;
  table->expirationHooks.next = NULL;
}

int routingTableSearchEntry(Routing_Table_t *table, UWB_Address_t targetAddress) {
  /* Binary Search */
  int left = -1, right = table->size, res = -1;
  while (left + 1 != right) {
    int mid = left + (right - left) / 2;
    if (table->entries[mid].destAddress == targetAddress) {
      res = mid;
      break;
    } else if (table->entries[mid].destAddress > targetAddress) {
      right = mid;
    } else {
      left = mid;
    }
  }
  return res;
}

typedef int (*routeEntryCompareFunc)(Route_Entry_t *, Route_Entry_t *);

static int COMPARE_BY_DEST_ADDRESS(Route_Entry_t *first, Route_Entry_t *second) {
  if (first->destAddress == second->destAddress) {
    return 0;
  }
  if (first->destAddress > second->destAddress) {
    return 1;
  }
  return -1;
}

static int COMPARE_BY_EXPIRATION_TIME(Route_Entry_t *first, Route_Entry_t *second) {
  if (first->expirationTime == second->expirationTime) {
    return 0;
  }
  if (first->expirationTime > second->expirationTime) {
    return -1;
  }
  return 1;
}

static int routingTableGetStalestEntry(Routing_Table_t *table) {
  if (table->size == 0) {
    DEBUG_PRINT("routingTableEvictStalestEntry: Routing table is empty, ignore.\n");
    return -1;
  }
  int stalestIndex = 0;
  for (int i = 0; i < table->size; i++) {
    if (COMPARE_BY_EXPIRATION_TIME(&table->entries[i], &table->entries[stalestIndex]) > 0) {
      stalestIndex = i;
    }
  }
  return stalestIndex;
}

/* Build the heap */
static void routingTableArrange(Routing_Table_t *table, int index, int len, routeEntryCompareFunc compare) {
  int leftChild = 2 * index + 1;
  int rightChild = 2 * index + 2;
  int maxIndex = index;
  if (leftChild < len && compare(&table->entries[maxIndex], &table->entries[leftChild]) < 0) {
    maxIndex = leftChild;
  }
  if (rightChild < len && compare(&table->entries[maxIndex], &table->entries[rightChild]) < 0) {
    maxIndex = rightChild;
  }
  if (maxIndex != index) {
    routingTableSwapRouteEntry(table, index, maxIndex);
    routingTableArrange(table, maxIndex, len, compare);
  }
}

/* Sort the routing table */
static void routingTableRearrange(Routing_Table_t *table, routeEntryCompareFunc compare) {
  /* Build max heap */
  for (int i = table->size / 2 - 1; i >= 0; i--) {
    routingTableArrange(table, i, table->size, compare);
  }
  for (int i = table->size - 1; i >= 0; i--) {
    routingTableSwapRouteEntry(table, 0, i);
    routingTableArrange(table, 0, i, compare);
  }
}

void routingTableAddEntry(Routing_Table_t *table, Route_Entry_t entry) {
  int index = routingTableSearchEntry(table, entry.destAddress);
  if (index != -1) {
//    DEBUG_PRINT("routingTableAddEntry: Try to add an already added route entry for dest %u, update it instead.\n",
//                entry.destAddress);
    table->entries[index] = entry;
  } else {
    ASSERT(ROUTING_TABLE_SIZE_MAX > 0);
    if (table->size == ROUTING_TABLE_SIZE_MAX) {
      int evictedIndex = -1;
      #ifdef ROUTING_TABLE_EVICT_POLICY_STALEST
      evictedIndex = routingTableGetStalestEntry(table);
      #else
      /* Randomly drop a route entry */
      evictedIndex = rand() % table->size;
      #endif
//      DEBUG_PRINT("routingTableEvictStalestEntry: Routing table is full, evict stalest route entry for dest %u.\n",
//                  table->entries[evictedIndex].destAddress);
      /* Swap it to last */
      int lastEntryIndex = ROUTING_TABLE_SIZE_MAX - 1;
      routingTableSwapRouteEntry(table, evictedIndex, lastEntryIndex);
      /* Clear the last entry */
      table->entries[lastEntryIndex] = EMPTY_ROUTE_ENTRY;
      table->size--;
    }
    /* Add the new entry to the last */
    uint8_t curIndex = table->size;
    table->entries[curIndex] = entry;
    table->size++;
    /* Sort the routing table */
    routingTableRearrange(table, COMPARE_BY_DEST_ADDRESS);
  }
}

void routingTableUpdateEntry(Routing_Table_t *table, Route_Entry_t entry) {
  int index = routingTableSearchEntry(table, entry.destAddress);
  if (index == -1) {
//    DEBUG_PRINT("routingTableUpdateEntry: Cannot find correspond route entry for dest %u, add it instead.\n",
//                entry.destAddress);
    routingTableAddEntry(table, entry);
  } else {
    table->entries[index] = entry;
//    DEBUG_PRINT("routingTableUpdateEntry: Update route entry for dest %u.\n", entry.destAddress);
  }
}

void routingTableRemoveEntry(Routing_Table_t *table, UWB_Address_t destAddress) {
  if (table->size == 0) {
//    DEBUG_PRINT("routingTableRemoveEntry: Routing table is empty, ignore.\n");
    return;
  }
  int index = routingTableSearchEntry(table, destAddress);
  if (index == -1) {
//    DEBUG_PRINT("routingTableRemoveEntry: Cannot find correspond route entry for dest %u, ignore.\n", destAddress);
    return;
  }
  // TODO: ? trigger hook
  routingTableSwapRouteEntry(table, index, table->size - 1);
  table->entries[table->size - 1] = EMPTY_ROUTE_ENTRY;
  table->size--;
  routingTableRearrange(table, COMPARE_BY_DEST_ADDRESS);
}

Route_Entry_t routingTableFindEntry(Routing_Table_t *table, UWB_Address_t destAddress) {
  int index = routingTableSearchEntry(table, destAddress);
  Route_Entry_t entry = EMPTY_ROUTE_ENTRY;
  if (index == -1) {
//    DEBUG_PRINT("routingTableFindEntry: Cannot find correspond route entry for dest %u.\n", destAddress);
  } else {
    entry = table->entries[index];
  }
  return entry;
}

void routingTableSort(Routing_Table_t *table) {
  routingTableRearrange(table, COMPARE_BY_DEST_ADDRESS);
}

void routingTableRegisterExpirationHook(Routing_Table_t *table, routeExpirationHook hook) {
  Route_Expiration_Hooks_t cur = {
      .hook = hook,
      .next = (struct Route_Expiration_Hook_Node *) table->expirationHooks.hook
  };
  table->expirationHooks = cur;
}

void routeExpirationHooksInvoke(Route_Expiration_Hooks_t *hooks, UWB_Address_t *addresses, int count) {
  routeExpirationHook cur = hooks->hook;
  while (cur != NULL) {
    DEBUG_PRINT("routeExpirationHooksInvoke: Invoke route expiration hook.\n");
    cur(addresses, count);
    cur = (routeExpirationHook) hooks->next;
  }
}

void printRouteEntry(Route_Entry_t *entry) {
  DEBUG_PRINT("dest\t next\t hop\t destSeq\t type\t expire\t validSeq\t valid \t \n");
  DEBUG_PRINT("%u\t %u\t %u\t %lu\t %d\t %lu\t %d\t %d\t \n",
              entry->destAddress,
              entry->nextHop,
              entry->hopCount,
              entry->destSeqNumber,
              entry->type,
              entry->expirationTime,
              entry->validDestSeqFlag,
              entry->valid);
}

void printRoutingTable(Routing_Table_t *table) {
  DEBUG_PRINT("dest\t next\t hop\t destSeq\t type\t expire\t validSeq\t valid \t \n");
  for (int i = 0; i < table->size; i++) {
    if (table->entries[i].destAddress == UWB_DEST_EMPTY) {
      continue;
    }
    DEBUG_PRINT("%u\t %u\t %u\t %lu\t %d\t %lu\t %d\t %d\t \n",
                table->entries[i].destAddress,
                table->entries[i].nextHop,
                table->entries[i].hopCount,
                table->entries[i].destSeqNumber,
                table->entries[i].type,
                table->entries[i].expirationTime,
                table->entries[i].validDestSeqFlag,
                table->entries[i].valid);
  }
  DEBUG_PRINT("---\n");
}