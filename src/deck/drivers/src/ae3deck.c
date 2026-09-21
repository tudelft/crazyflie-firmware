/**
 * ae3deck.c — read a single float (the AE3 forward camera's median ToF distance)
 * over UART2 and publish it as the log variable ae3.dist.
 *
 * PASSIVE: this deck never commands flight. It only reads the AE3 and exposes a
 * log variable, so it runs alongside a Python/CRTP flight script without any
 * conflict (the Python link is the nRF/CRTP radio; this is a separate UART).
 *
 * Wiring — AE3 on UART2 (UART1 is the MTF-02 flow deck; the 4x4 ToF is I2C;
 * Loco/USD are SPI — so UART2 is free):
 *   PA2 (UART2 TX) -> AE3 RX
 *   PA3 (UART2 RX) <- AE3 TX
 *   GND shared, AE3 powered.
 * On the flapper, disable CPPM (it captures on PA3) so UART2 RX is free.
 *
 * Wire protocol (AE3 -> CF), little-endian:
 *   0xAE 0x51 | float32 dist | xor(the 4 float bytes)      (7 bytes/frame)
 * The float is whatever the AE3 sends (e.g. metres). The CF just stores it.
 */

#define DEBUG_MODULE "AE3"

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"

#include "uart2.h"

#include <string.h>

#define AE3_BAUDRATE        115200
#define AE3_SYNC0           0xAE
#define AE3_SYNC1           0x51
#define AE3_TASK_PRI        3
#define AE3_TASK_STACKSIZE  (2 * configMINIMAL_STACK_SIZE)
#define AE3_TASK_NAME       "AE3"

static bool isInit = false;

// Logged values
static float     ae3Dist   = 0.0f;  // last valid float from the AE3 (e.g. metres)
static uint32_t  ae3Rx     = 0;     // valid frames received (link-alive counter)
static uint32_t  ae3Bad    = 0;     // checksum failures
static uint32_t  ae3AgeMs  = 0;     // ms since the last valid frame
static TickType_t ae3LastTick = 0;  // tick of the last valid frame

// Byte-by-byte frame parser: 0xAE 0x51 | float32 LE | xor(4 float bytes)
static void ae3ParseByte(uint8_t b)
{
  static uint8_t state = 0;    // 0:sync0  1:sync1  2:payload  3:xor
  static uint8_t buf[4];
  static uint8_t idx = 0;

  switch (state) {
    case 0:
      if (b == AE3_SYNC0) { state = 1; }
      break;
    case 1:
      if (b == AE3_SYNC1) { state = 2; idx = 0; }
      else                { state = (b == AE3_SYNC0) ? 1 : 0; }
      break;
    case 2:
      buf[idx++] = b;
      if (idx >= 4) { state = 3; }
      break;
    case 3: {
      uint8_t x = buf[0] ^ buf[1] ^ buf[2] ^ buf[3];
      if (x == b) {
        float f;
        memcpy(&f, buf, sizeof(f));   // little-endian on the STM32
        ae3Dist = f;
        ae3Rx++;
        ae3LastTick = xTaskGetTickCount();
      } else {
        ae3Bad++;
      }
      state = 0;
      break;
    }
    default:
      state = 0;
      break;
  }
}

static void ae3Task(void *param)
{
  (void)param;
  systemWaitStart();
  DEBUG_PRINT("AE3 UART2 reader running\n");
  uint8_t b;
  while (1) {
    if (uart2GetCharWithTimeout(&b, M2T(100))) {
      ae3ParseByte(b);
    }
    ae3AgeMs = T2M((uint32_t)(xTaskGetTickCount() - ae3LastTick));
  }
}

static void ae3Init(DeckInfo *info)
{
  (void)info;
  if (isInit) { return; }
  uart2Init(AE3_BAUDRATE);
  xTaskCreate(ae3Task, AE3_TASK_NAME, AE3_TASK_STACKSIZE, NULL, AE3_TASK_PRI, NULL);
  isInit = true;
  DEBUG_PRINT("AE3 deck initialized (UART2 @ %d baud)\n", AE3_BAUDRATE);
}

static bool ae3Test(void)
{
  return isInit;
}

static const DeckDriver ae3Deck = {
  .name = "bcAE3",
  .usedPeriph = DECK_USING_UART2,
  .init = ae3Init,
  .test = ae3Test,
};

DECK_DRIVER(ae3Deck);

/**
 * AE3 forward-camera link — a single float read off UART2 and exposed here.
 */
LOG_GROUP_START(ae3)
/**
 * @brief Median ToF distance from the AE3 forward camera (float, as sent by the AE3)
 */
LOG_ADD(LOG_FLOAT, dist, &ae3Dist)
/**
 * @brief Valid AE3 frames received (link-alive counter)
 */
LOG_ADD(LOG_UINT32, rx, &ae3Rx)
/**
 * @brief Milliseconds since the last valid AE3 frame (large = no data)
 */
LOG_ADD(LOG_UINT32, age, &ae3AgeMs)
/**
 * @brief AE3 checksum failures
 */
LOG_ADD(LOG_UINT32, bad, &ae3Bad)
LOG_GROUP_STOP(ae3)
