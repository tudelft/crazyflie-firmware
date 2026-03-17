#include "string.h"
#include "FreeRTOS.h"
#include "app.h"
#include "debug.h"
#include "task.h"

#include "routing.h"

/* Constants */
#define APP_RX_QUEUE_SIZE 5
#define APP_RX_QUEUE_ITEM_SIZE sizeof(UWB_Data_Packet_t)
#define APP_TX_DEST 3
#define APP_TX_INTERVAL 100
#define PING_MSG_BUFFER_SIZE 20

typedef enum {
  PING,
  PING_ACK
} MESSAGE_TYPE;

typedef struct {
  MESSAGE_TYPE type;
  uint32_t txSeqNumber;
} Route_Ping_Message_t;

typedef struct {
  uint32_t txSeqNumber;
  Time_t txTime;
} Ping_Msg_Buffer_Item_t;

typedef struct {
  uint8_t index;
  Ping_Msg_Buffer_Item_t buffer[PING_MSG_BUFFER_SIZE];
} Ping_Msg_Buffer_t;

static TaskHandle_t appTxTaskHandle;
static TaskHandle_t appRxTaskHandle;
static QueueHandle_t rxQueue;

static Routing_Table_t *routingTable;

static Ping_Msg_Buffer_t pingMsgBuffer;
static uint32_t pingSeqNumber = 1;

static void appTxTask() {
  while (1) {
    xSemaphoreTake(routingTable->mu, portMAX_DELAY);
    printRoutingTable(routingTable);
    xSemaphoreGive(routingTable->mu);
    UWB_Data_Packet_t dataTxPacket;
    dataTxPacket.header.type = UWB_DATA_MESSAGE_RESERVED;
    dataTxPacket.header.srcAddress = uwbGetAddress();
    dataTxPacket.header.destAddress = APP_TX_DEST;
    dataTxPacket.header.ttl = 10;
    dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t) + sizeof(Route_Ping_Message_t);
    Route_Ping_Message_t *pingMsg = (Route_Ping_Message_t *) &dataTxPacket.payload;
    pingMsg->type = PING;
    pingMsg->txSeqNumber = pingSeqNumber;
    pingMsgBuffer.buffer[pingMsgBuffer.index].txSeqNumber = pingMsg->txSeqNumber;
    pingMsgBuffer.buffer[pingMsgBuffer.index].txTime = xTaskGetTickCount();
    pingSeqNumber++;
    pingMsgBuffer.index = (pingMsgBuffer.index + 1) % PING_MSG_BUFFER_SIZE;
    uwbSendDataPacketBlock(&dataTxPacket);
    vTaskDelay(M2T(APP_TX_INTERVAL));
  }
}

static void appRxTask() {
  UWB_Data_Packet_t dataRxPacket;
  while (1) {
    if (uwbReceiveDataPacketBlock(UWB_DATA_MESSAGE_RESERVED, &dataRxPacket)) {
      Route_Ping_Message_t *msg = (Route_Ping_Message_t *) &dataRxPacket.payload;
      Time_t rxTime = xTaskGetTickCount();
      if (msg->type == PING_ACK) {
        bool found = false;
        for (int i = 0; i < PING_MSG_BUFFER_SIZE; i++) {
          if (pingMsgBuffer.buffer[i].txSeqNumber == msg->txSeqNumber) {
            found = true;
            DEBUG_PRINT("appRxTask: %u received ping ack packet from %u, seq = %lu, RTT = %ums.\n",
                        uwbGetAddress(),
                        dataRxPacket.header.srcAddress,
                        msg->txSeqNumber,
                        T2M(rxTime - pingMsgBuffer.buffer[i].txTime)
            );
            break;
          }
        }
        if (!found) {
          DEBUG_PRINT(
              "appRxTask: %u received ping ack packet from %u, seq = %u, cannot found corresponding msg in buffer.\n",
              uwbGetAddress(),
              dataRxPacket.header.srcAddress,
              msg->txSeqNumber
          );
        }
      } else if (dataRxPacket.header.srcAddress != uwbGetAddress()) {
        UWB_Data_Packet_t dataTxPacket;
        dataTxPacket.header.type = UWB_DATA_MESSAGE_RESERVED;
        dataTxPacket.header.srcAddress = uwbGetAddress();
        dataTxPacket.header.destAddress = dataRxPacket.header.srcAddress;
        dataTxPacket.header.ttl = 10;
        dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t) + sizeof(Route_Ping_Message_t);
        uint32_t txSeqNo = msg->txSeqNumber;
        msg = (Route_Ping_Message_t *) &dataTxPacket.payload;
        msg->type = PING_ACK;
        msg->txSeqNumber = txSeqNo;
        DEBUG_PRINT("appRxTask: %u send ping ack packet to %u, seq = %lu.\n",
                    uwbGetAddress(),
                    dataRxPacket.header.srcAddress,
                    msg->txSeqNumber
        );
        uwbSendDataPacketBlock(&dataTxPacket);
      }
    }
    vTaskDelay(M2T(1));
  }
}

void appMain() {
  rxQueue = xQueueCreate(APP_RX_QUEUE_SIZE, APP_RX_QUEUE_ITEM_SIZE);
  routingTable = getGlobalRoutingTable();
  UWB_Data_Packet_Listener_t listener = {
      .type = UWB_DATA_MESSAGE_RESERVED,
      .rxQueue = rxQueue
  };
  uwbRegisterDataPacketListener(&listener);
  if (uwbGetAddress() != APP_TX_DEST) {
    xTaskCreate(appTxTask, "ADHOC_ROUTING_TEST_TX", UWB_TASK_STACK_SIZE, NULL,
                ADHOC_DECK_TASK_PRI, &appTxTaskHandle);
  }
  xTaskCreate(appRxTask, "ADHOC_ROUTING_TEST_RX", UWB_TASK_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &appRxTaskHandle);
}
