#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "uart2.h"
#include "log.h"
#include "debug.h"

#define DEBUG_MODULE "CURVACE"

#define SERIAL_HEADER 0x00

static bool isInit;

typedef struct __attribute__((packed)) mavicData_s {
  uint8_t header;
  float targetVX;
  float targetVY;
  float targetVZ;
} mavicData_t;


void mavicTask(void *param)
{
  int velXid;
  int velYid;
  int velZid;

  mavicData_t packet;

  systemWaitStart();

  // Setup data to transfer
  velXid = logGetVarId("posCtl", "targetVX");
  velYid = logGetVarId("posCtl", "targetVY");
  velZid = logGetVarId("posCtl", "targetVZ");

  TickType_t lastWakeTime = xTaskGetTickCount();

  while (1)
  {
    // Set the loop unlock time in ms
    vTaskDelayUntil(&lastWakeTime, M2T(10));

    // Assemble the data
    packet.header = SERIAL_HEADER;
    packet.targetVX = logGetFloat(velXid);
    packet.targetVY = logGetFloat(velYid);
    packet.targetVZ = logGetFloat(velZid);

    // Send the three floats, byte by byte, to UART2
    uart2SendDataDmaBlocking(sizeof(mavicData_t), (uint8_t *)(&packet));
  }
}


static void mavicInit()
{
  DEBUG_PRINT("Starting Mavic task: writing velocity commands to UART2\n");

  xTaskCreate(mavicTask, "MAVIC", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  // Configure uart and set the baud rate
  /**
   * TODO: here or in cf-board.mk as flag?
   */
  uart2Init(115200);

  isInit = true;
}


static bool mavicTest()
{
  return isInit;
}


static const DeckDriver mavicDriver = {
  .name = "CurvACE",
  .init = mavicInit,
  .test = mavicTest,
};


DECK_DRIVER(mavicDriver);
