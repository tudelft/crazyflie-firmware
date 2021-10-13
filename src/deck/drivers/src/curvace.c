
#include "deck.h"
#include "debug.h"
#include "system.h"
#include "log.h"
#include "param.h"

#include "FreeRTOS.h"
#include "task.h"
#include "uart2.h"

#define DEBUG_MODULE "CURVACE"

#define SERIAL_HEADER 'C'

#ifdef UART2_LINK_COMM
#warning DO_NOT_USE_LINK_ON_UART2_FOR_CURVACE
#endif

static uint8_t isInit = 0;

typedef struct __attribute__((packed)) curvaceData_s {
  uint8_t header;
  uint8_t v1;
  uint8_t v2;
  uint8_t v3;
} curvaceData_t;


typedef struct __attribute__((packed)) curvaceFlow_s {
  float x1;
  float y1;
  float x2;
  float y2;
  float x3;
  float y3;
  float x4;
  float y4;
} curvaceFlow_t;


curvaceFlow_t curvaceFlow;

void curvaceTask(void *param)
{
  //int velXid;
  //int velYid;
  //int velZid;

  char c = 'A';

  curvaceData_t packet;

  systemWaitStart();

  // Setup data to transfer
  //velXid = logGetVarId("posCtl", "targetVX");
  //velYid = logGetVarId("posCtl", "targetVY");
  //velZid = logGetVarId("posCtl", "targetVZ");

  TickType_t lastWakeTime = xTaskGetTickCount();

  while (1)
  {
    // Set the loop unlock time in ms
    vTaskDelayUntil(&lastWakeTime, M2T(100));


    uart2Getchar(&c);

    // Assemble the data
    packet.header = SERIAL_HEADER;
    packet.v1 = c;
//    if (c > 'Z') {
//      c = 'A';
//    }
    packet.v2 = 10;
    packet.v3 = 13;
    
    //packet.targetVX = logGetFloat(velXid);
    //packet.targetVY = logGetFloat(velYid);
    //packet.targetVZ = logGetFloat(velZid);

    // Send the three floats, byte by byte, to UART2
    uart2SendDataDmaBlocking(sizeof(curvaceData_t), (uint8_t *)(&packet));
  }
}


static void curvaceInit()
{
  DEBUG_PRINT("Starting CurvACE task: reading flow from  UART2\n");

  xTaskCreate(curvaceTask, "CURVACE", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  // Configure uart and set the baud rate
  /**
   * TODO: here or in cf-board.mk as flag?
   */
  uart2Init(115200);

  isInit = 1;

  curvaceFlow.x1 = 1.0f;
  curvaceFlow.y1 = 2.0f;
  curvaceFlow.x2 = 3.0f;
  curvaceFlow.y2 = 4.0f;
  curvaceFlow.x3 = 5.0f;
  curvaceFlow.y3 = 6.0f;
  curvaceFlow.x4 = 7.0f;
  curvaceFlow.y4 = 8.0f;
  
}


static bool curvaceTest()
{
  return isInit == 1;
}


static const DeckDriver curvaceDriver = {
  .name = "CurvACE",
  .init = curvaceInit,
  .test = curvaceTest,
};


DECK_DRIVER(curvaceDriver);




/**
 * Logging variables of the motion sensor of the flowdeck
 */
LOG_GROUP_START(curvace)
/**
 * @brief True if motion occured since the last measurement
 */
LOG_ADD(LOG_FLOAT, x1, &curvaceFlow.x1)
LOG_ADD(LOG_FLOAT, y1, &curvaceFlow.y1)
LOG_ADD(LOG_FLOAT, x2, &curvaceFlow.x2)
LOG_ADD(LOG_FLOAT, y2, &curvaceFlow.y2)
LOG_ADD(LOG_FLOAT, x3, &curvaceFlow.x3)
LOG_ADD(LOG_FLOAT, y3, &curvaceFlow.y3)
LOG_ADD(LOG_FLOAT, x4, &curvaceFlow.x4)
LOG_ADD(LOG_FLOAT, y4, &curvaceFlow.y4)
LOG_GROUP_STOP(curvace)





PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Multi-ranger deck](%https://store.bitcraze.io/collections/decks/products/multi-ranger-deck) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcCurvACEInit, &isInit)

PARAM_GROUP_STOP(deck)
