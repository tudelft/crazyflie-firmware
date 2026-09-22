/**
 * ae3deck.c — read the AE3 forward camera's telemetry over UART2 and publish it
 * as log variables the Python flight script can tap into.
 *
 * The AE3 sends three floats + one byte per frame:
 *   ae3.dist  — median ToF distance (metres); NaN = the sensor got no reading.
 *   ae3.x     — horizontal centering offset of the tracked object in the frame.
 *               Sign/scale are the camera script's convention (e.g. -2..2,
 *               negative = object left of centre -> drone should move left).
 *               0.0 = centred / no correction.
 *   ae3.y     — vertical centering offset (adjust height). 0.0 = no correction.
 *   ae3.state — the camera's current tracking mode (enum set by the camera
 *               script): e.g. 1 = black boxes, 2 = blue boxes, 3 = walls.
 *               0 is reserved for "none / unknown" (also the stale default).
 *
 * ae3.x / ae3.y are AUXILIARY: base flight does not depend on them. They default
 * to 0.0 ("do nothing"), and if the AE3 link goes stale (no frames for
 * AE3_STALE_MS) they are forced back to 0.0 (and state back to 0) so a lost
 * detection can never leave a stale push running. (dist is left at its last
 * value — the sender sends NaN on a failed read, and the CF side should treat
 * NaN as "range unknown, hold".)
 *
 * PASSIVE: this deck never commands flight. It only reads the AE3 and exposes
 * log variables, so it runs alongside a Python/CRTP flight script without any
 * conflict (the Python link is the nRF/CRTP radio; this is a separate UART).
 *
 * Wiring — AE3 on UART2 (UART1 is the MTF-02 flow deck; the 4x4 ToF is I2C;
 * Loco/USD are SPI — so UART2 is free):
 *   PA2 (UART2 TX) -> AE3 RX
 *   PA3 (UART2 RX) <- AE3 TX
 *   GND shared, AE3 powered.
 * On the flapper, disable CPPM (it captures on PA3) so UART2 RX is free.
 *
 * Wire protocol (AE3 -> CF), little-endian — matches the camera's
 * struct.pack("<Bfff", state, depth, x, y):
 *   0xAE 0x52 | uint8 state | float32 dist | float32 x | float32 y |
 *   xor(the 13 payload bytes)                         (16 bytes/frame).
 * The CF just stores the state byte and the three floats.
 *
 * NOTE the second sync byte is 0x52 (the camera's "state" frame). The camera
 * also has a 0x51 range-only frame (state absent); this deck reads 0x52.
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
#define AE3_SYNC1           0x52    // the camera's "state" frame (0x51 = range-only)
#define AE3_PAYLOAD_LEN     13      // 3 x float32 (dist, x, y) + 1 x uint8 (state)
#define AE3_STALE_MS        300     // no frame for this long -> zero x/y (do nothing)
#define AE3_TASK_PRI        3
#define AE3_TASK_STACKSIZE  (2 * configMINIMAL_STACK_SIZE)
#define AE3_TASK_NAME       "AE3"

static bool isInit = false;

// Logged values
static float     ae3Dist   = 0.0f;  // median ToF distance (metres); NaN = no reading
static float     ae3X      = 0.0f;  // horizontal centering offset; 0 = no push
static float     ae3Y      = 0.0f;  // vertical centering offset; 0 = no push
static uint8_t   ae3State  = 0;     // camera tracking mode (1/2/3...); 0 = none/unknown
static uint32_t  ae3Rx     = 0;     // valid frames received (link-alive counter)
static uint32_t  ae3Bad    = 0;     // checksum failures
static uint32_t  ae3AgeMs  = 0;     // ms since the last valid frame
static TickType_t ae3LastTick = 0;  // tick of the last valid frame

// Byte-by-byte frame parser (little-endian, matches camera's "<Bfff"):
//   0xAE 0x52 | uint8 state | float32 dist | float32 x | float32 y | xor(13 payload bytes)
static void ae3ParseByte(uint8_t b)
{
  static uint8_t state = 0;    // 0:sync0  1:sync1  2:payload  3:xor
  static uint8_t buf[AE3_PAYLOAD_LEN];
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
      if (idx >= AE3_PAYLOAD_LEN) { state = 3; }
      break;
    case 3: {
      uint8_t x = 0;
      for (uint8_t i = 0; i < AE3_PAYLOAD_LEN; i++) { x ^= buf[i]; }
      if (x == b) {
        // little-endian on the STM32, so a plain memcpy matches struct.pack("<Bfff")
        //   payload: [0] state | [1:5] dist | [5:9] x | [9:13] y
        ae3State = buf[0];
        memcpy(&ae3Dist, buf + 1, 4);   // metres, or NaN (no ToF reading) — kept as-is
        memcpy(&ae3X,    buf + 5, 4);
        memcpy(&ae3Y,    buf + 9, 4);
        // The camera sends NaN for x/y when nothing is tracked (state 0). We
        // present those as 0.0 ("no push") so the flight script always gets a
        // real number; ae3.state==0 is how you tell "nothing seen" from
        // "centred". dist is left as NaN — 0.0 there would read as a wall
        // touching the sensor.
        if (ae3X != ae3X) { ae3X = 0.0f; }   // NaN != NaN
        if (ae3Y != ae3Y) { ae3Y = 0.0f; }
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
    // Auxiliary, fail-safe: if the camera link is stale, the centering offsets
    // default back to "do nothing" and the mode to "none" so an old push or a
    // stale mode can't persist. dist is left as-is (the sender signals a bad
    // reading with NaN).
    if (ae3AgeMs > AE3_STALE_MS) {
      ae3X = 0.0f;
      ae3Y = 0.0f;
      ae3State = 0;
    }
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
 * AE3 forward-camera link — distance + object-centering offsets read off UART2.
 */
LOG_GROUP_START(ae3)
/**
 * @brief Median ToF distance from the AE3 forward camera, metres (NaN = no reading)
 */
LOG_ADD(LOG_FLOAT, dist, &ae3Dist)
/**
 * @brief Horizontal centering offset of the tracked object; 0 = centred / no push
 */
LOG_ADD(LOG_FLOAT, x, &ae3X)
/**
 * @brief Vertical centering offset of the tracked object; 0 = centred / no push
 */
LOG_ADD(LOG_FLOAT, y, &ae3Y)
/**
 * @brief Camera tracking mode (e.g. 1=black boxes, 2=blue boxes, 3=walls); 0 = none/unknown
 */
LOG_ADD(LOG_UINT8, state, &ae3State)
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
