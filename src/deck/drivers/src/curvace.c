
#include "deck.h"
#include "debug.h"
#include "system.h"
#include "log.h"
#include "param.h"

#include "FreeRTOS.h"
#include "task.h"
#include "uart2.h"

#include "mpu_wrappers.h"
#include "attitude_estimator.h"

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
  int16_t x1;
  int16_t y1;
  int16_t x2;
  int16_t y2;
  int16_t x3;
  int16_t y3;
  int16_t x4;
  int16_t y4;
  uint16_t error;
  int16_t avgx;
  int16_t avgy; 
  float dt;
  float fps;
  float fx;
  float fy;
  uint32_t lastData;
} curvaceFlow_t;


curvaceFlow_t curvaceFlow;

uint8_t hex(char x) {
  if (x < '0') {
    return 16;
  }
  if (x > '9') {
    return 10 + (x - 'A');
  }
  return x-'0';
}

int16_t get_hex(uint8_t* b) {
  int16_t vv = 0;

  vv += hex(*b);
  b++;
  vv = vv << 4;

  vv += hex(*b);
  b++;
  vv = vv << 4;
  
  vv += hex(*b);
  b++;
  vv = vv << 4;

  vv += hex(*b);

  return vv;
}

void curvaceTask(void *param)
{
  //int velXid;
  //int velYid;
  //int velZid;

  char c = 'A';
  uint8_t buf[33];

  //curvaceData_t packet;

  systemWaitStart();

  // Setup data to transfer
  //velXid = logGetVarId("posCtl", "targetVX");
  //velYid = logGetVarId("posCtl", "targetVY");
  //velZid = logGetVarId("posCtl", "targetVZ");

  TickType_t lastWakeTime = xTaskGetTickCount();

  while (1)
  {
    // Set the loop unlock time in ms
    vTaskDelayUntil(&lastWakeTime, M2T(5));


    int cnt=0;

    // Get a line
    while (cnt < 33) {
      uart2Getchar(&c);
      buf[cnt] = c;
      if (c == 10) {
        break;
      }
      cnt++;
    }

    if (cnt == 32) {
      // Received a message!
      curvaceFlow.x1 = get_hex(buf);
      curvaceFlow.y1 = get_hex(buf+4);
      curvaceFlow.x2 = get_hex(buf+8);
      curvaceFlow.y2 = get_hex(buf+12);
      curvaceFlow.x3 = get_hex(buf+16);
      curvaceFlow.y3 = get_hex(buf+20);
      curvaceFlow.x4 = get_hex(buf+24);
      curvaceFlow.y4 = get_hex(buf+28);


      uint32_t osTick = xTaskGetTickCount(); // would be nice if this had a precision higher than 1ms...

      curvaceFlow.dt = T2S(osTick - curvaceFlow.lastData);
      curvaceFlow.lastData = osTick;

      curvaceFlow.avgx = curvaceFlow.x1/4+curvaceFlow.x2/4+curvaceFlow.x3/4+curvaceFlow.x4/4;
      curvaceFlow.avgy = curvaceFlow.y1/4+curvaceFlow.y2/4+curvaceFlow.y3/4+curvaceFlow.y4/4;

      if (curvaceFlow.dt <=0.0f)
      {
        curvaceFlow.dt = 1.0f/100.0f;
      }

      curvaceFlow.fps = 202.0f; //1.0f / curvaceFlow.dt;

#define FOCAL_LENGTH   56205  //26741.0f
#define MANUAL_CORRECTION 1.0f //3.3f

      curvaceFlow.fx = -((float)curvaceFlow.avgx) / FOCAL_LENGTH * curvaceFlow.fps; // / MANUAL_CORRECTION ;
      curvaceFlow.fy = -((float)curvaceFlow.avgy) / FOCAL_LENGTH * curvaceFlow.fps; // / MANUAL_CORRECTION;


      set_flow_measurement(curvaceFlow.fy);



    } else {
      curvaceFlow.error++;
    }



    // Assemble the data
    //packet.header = SERIAL_HEADER;
    //packet.v1 = c;
    //packet.v2 = 10;
    //packet.v3 = 13;
    
    //packet.targetVX = logGetFloat(velXid);
    //packet.targetVY = logGetFloat(velYid);
    //packet.targetVZ = logGetFloat(velZid);

    // Send the three floats, byte by byte, to UART2
    //uart2SendDataDmaBlocking(sizeof(curvaceData_t), (uint8_t *)(&packet));
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

  curvaceFlow.x1 = 0;
  curvaceFlow.y1 = 0;
  curvaceFlow.x2 = 0;
  curvaceFlow.y2 = 0;
  curvaceFlow.x3 = 0;
  curvaceFlow.y3 = 0;
  curvaceFlow.x4 = 0;
  curvaceFlow.y4 = 0;
  curvaceFlow.avgx = 0;
  curvaceFlow.avgy = 0;
  curvaceFlow.error = 0;
  curvaceFlow.fx = 0.0f;
  curvaceFlow.fy = 0.0f;
  curvaceFlow.fps = 0.0f;
  curvaceFlow.dt = 0.01f;
  curvaceFlow.lastData = xTaskGetTickCount();
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
LOG_ADD(LOG_INT16, x1, &curvaceFlow.x1)
LOG_ADD(LOG_INT16, y1, &curvaceFlow.y1)
LOG_ADD(LOG_INT16, x2, &curvaceFlow.x2)
LOG_ADD(LOG_INT16, y2, &curvaceFlow.y2)
LOG_ADD(LOG_INT16, x3, &curvaceFlow.x3)
LOG_ADD(LOG_INT16, y3, &curvaceFlow.y3)
LOG_ADD(LOG_INT16, x4, &curvaceFlow.x4)
LOG_ADD(LOG_INT16, y4, &curvaceFlow.y4)
LOG_ADD(LOG_INT16, avgx, &curvaceFlow.avgx)
LOG_ADD(LOG_INT16, avgy, &curvaceFlow.avgy)
LOG_ADD(LOG_UINT16, err, &curvaceFlow.error)
LOG_ADD(LOG_FLOAT, fps, &curvaceFlow.fps)
LOG_ADD(LOG_FLOAT, fx, &curvaceFlow.fx)
LOG_ADD(LOG_FLOAT, fy, &curvaceFlow.fy)
LOG_GROUP_STOP(curvace)





PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Multi-ranger deck](%https://store.bitcraze.io/collections/decks/products/multi-ranger-deck) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcCurvACEInit, &isInit)

PARAM_GROUP_STOP(deck)
