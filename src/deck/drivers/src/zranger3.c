/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2025 BitCraze AB & TU Delft
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * zranger3.c: VL53L8CX multi-zone ToF deck driver
 *
 * The VL53L8CX provides a 4x4 or 8x8 grid of distance measurements.
 * This deck driver reads all zones at the configured resolution and
 * frequency, logging every zone distance for use in obstacle avoidance.
 */

#define DEBUG_MODULE "ZR3"

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "static_mem.h"

#include "i2cdev.h"
#include "zranger3.h"
#include "vl53l8cx.h"

/* ---------------------------------------------------------------------------
 * Configuration
 * --------------------------------------------------------------------------- */

/* Sensor polling interval in ms.
 * 4x4 @ 15 Hz → ~67 ms per frame.  We poll every 50 ms so we usually
 * get a new frame each iteration with minimal latency. */
#define ZRANGER3_POLL_INTERVAL_MS  50

/* Default resolution: 4x4 = 16 zones */
#define ZRANGER3_DEFAULT_RESOLUTION  VL53L8CX_RESOLUTION_4X4
#define ZRANGER3_DEFAULT_FREQ_HZ     15

/* ---------------------------------------------------------------------------
 * State
 * --------------------------------------------------------------------------- */

static bool isInit = false;

static VL53L8CX_Configuration dev;
static VL53L8CX_ResultsData   results;

/* Per-zone distance in mm (4x4 = 16 zones, row-major).
 * Zones are numbered 0..15 in a 4x4 grid as seen from above:
 *   0  1  2  3
 *   4  5  6  7
 *   8  9  10 11
 *   12 13 14 15
 */
static int16_t zoneDistances[16];

/* Column averages in mm (valid zones only; 4000 if all zones invalid).
 * colAvg[0] = right (zones 0,4,8,12)
 * colAvg[1] = mid-right (zones 1,5,9,13)
 * colAvg[2] = mid-left (zones 2,6,10,14)
 * colAvg[3] = left (zones 3,7,11,15)
 */
static int16_t colAvg[4];

/* ---------------------------------------------------------------------------
 * Deck driver callbacks
 * --------------------------------------------------------------------------- */

void zRanger3Init(DeckInfo* info)
{
  if (isInit)
    return;

  if (vl53l8cxInit(&dev, I2C1_DEV))
  {
    DEBUG_PRINT("Z-forward VL53L8CX [OK]\n");
  }
  else
  {
    DEBUG_PRINT("Z-forward VL53L8CX [FAIL]\n");
    return;
  }

  xTaskCreate(zRanger3Task, ZRANGER3_TASK_NAME, ZRANGER3_TASK_STACKSIZE,
              NULL, ZRANGER3_TASK_PRI, NULL);

  isInit = true;
}


bool zRanger3Test(void)
{
  return isInit;
}


void zRanger3Task(void* arg)
{
  (void)arg;
  TickType_t lastWakeTime;
  uint8_t status;
  uint8_t dataReady = 0;

  systemWaitStart();

  /* Configure sensor: 4x4, 15 Hz, continuous ranging */
  status  = vl53l8cx_set_resolution(&dev, ZRANGER3_DEFAULT_RESOLUTION);
  status |= vl53l8cx_set_ranging_frequency_hz(&dev, ZRANGER3_DEFAULT_FREQ_HZ);
  status |= vl53l8cx_set_ranging_mode(&dev, VL53L8CX_RANGING_MODE_CONTINUOUS);
  status |= vl53l8cx_start_ranging(&dev);

  if (status != VL53L8CX_STATUS_OK)
  {
    DEBUG_PRINT("VL53L8CX ranging start failed (%u)\n", status);
    /* Don't return — the task keeps retrying in the loop below */
  }

  lastWakeTime = xTaskGetTickCount();

  while (1)
  {
    vTaskDelayUntil(&lastWakeTime, M2T(ZRANGER3_POLL_INTERVAL_MS));

    /* Check if new data is available */
    status = vl53l8cx_check_data_ready(&dev, &dataReady);
    if (status != VL53L8CX_STATUS_OK || !dataReady)
      continue;

    /* Read the full multi-zone result */
    status = vl53l8cx_get_ranging_data(&dev, &results);
    if (status != VL53L8CX_STATUS_OK)
      continue;

    /* Store all zone distances for logging */
    uint8_t nZones = (ZRANGER3_DEFAULT_RESOLUTION == VL53L8CX_RESOLUTION_4X4) ? 16 : 64;

    for (uint8_t i = 0; i < nZones && i < 16; i++)
    {
#ifndef VL53L8CX_DISABLE_DISTANCE_MM
#ifndef VL53L8CX_DISABLE_TARGET_STATUS
      /* Only accept zones with a valid ranging status (5 = valid, 9 = valid) */
      uint8_t s = results.target_status[i];
      zoneDistances[i] = (s == 5 || s == 9) ? results.distance_mm[i] : 0;
#else
      zoneDistances[i] = results.distance_mm[i];
#endif
#endif
    }

    /* Compute column averages over valid (positive) zones.
     * Column j contains row-major zones: j, j+4, j+8, j+12. */
    for (uint8_t col = 0; col < 4; col++)
    {
      int32_t sum   = 0;
      int32_t count = 0;
      for (uint8_t row = 0; row < 4; row++)
      {
        int16_t d = zoneDistances[row * 4 + col];
        if (d > 0) { sum += d; count++; }
      }
      colAvg[col] = (count > 0) ? (int16_t)(sum / count) : 4000;
    }
  }
}

/* ---------------------------------------------------------------------------
 * Deck registration
 * --------------------------------------------------------------------------- */

static const DeckDriver zranger3_deck = {
  .vid = 0xBC,
  .pid = 0x13,            /* New PID for VL53L8CX deck */
  .name = "bcZRanger3",
  .usedGpio = 0,
  .usedPeriph = DECK_USING_I2C,

  .init = zRanger3Init,
  .test = zRanger3Test,
};

DECK_DRIVER(zranger3_deck);

/* ---------------------------------------------------------------------------
 * Parameters
 * --------------------------------------------------------------------------- */

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if Z-ranger V3 (VL53L8CX) deck is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger3, &isInit)

PARAM_GROUP_STOP(deck)

/* ---------------------------------------------------------------------------
 * Logging
 * --------------------------------------------------------------------------- */

LOG_GROUP_START(range8)
  LOG_ADD(LOG_INT16, z00, &zoneDistances[0])
  LOG_ADD(LOG_INT16, z01, &zoneDistances[1])
  LOG_ADD(LOG_INT16, z02, &zoneDistances[2])
  LOG_ADD(LOG_INT16, z03, &zoneDistances[3])
  LOG_ADD(LOG_INT16, z04, &zoneDistances[4])
  LOG_ADD(LOG_INT16, z05, &zoneDistances[5])
  LOG_ADD(LOG_INT16, z06, &zoneDistances[6])
  LOG_ADD(LOG_INT16, z07, &zoneDistances[7])
  LOG_ADD(LOG_INT16, z08, &zoneDistances[8])
  LOG_ADD(LOG_INT16, z09, &zoneDistances[9])
  LOG_ADD(LOG_INT16, z10, &zoneDistances[10])
  LOG_ADD(LOG_INT16, z11, &zoneDistances[11])
  LOG_ADD(LOG_INT16, z12, &zoneDistances[12])
  LOG_ADD(LOG_INT16, z13, &zoneDistances[13])
  LOG_ADD(LOG_INT16, z14, &zoneDistances[14])
  LOG_ADD(LOG_INT16, z15, &zoneDistances[15])
  /* Column averages for obstacle detection */
  LOG_ADD(LOG_INT16, colR,  &colAvg[0])
  LOG_ADD(LOG_INT16, colMR, &colAvg[1])
  LOG_ADD(LOG_INT16, colML, &colAvg[2])
  LOG_ADD(LOG_INT16, colL,  &colAvg[3])
LOG_GROUP_STOP(range8)
