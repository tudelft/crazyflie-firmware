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
 * vl53l8cx.c: VL53L8CX Time-of-Flight multi-zone sensor driver
 *
 * This file provides the Crazyflie-specific initialization for the VL53L8CX.
 * It mirrors the pattern of vl53l1x.c: dynamic I2C address assignment from
 * the shared ranger address pool, and a thin wrapper around the ST ULD API.
 */

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "i2cdev.h"
#include "vl53l8cx.h"

#define DEBUG_MODULE "VL8CX"

bool vl53l8cxInit(VL53L8CX_Configuration *pdev, I2C_Dev *I2Cx)
{
  uint8_t status = VL53L8CX_STATUS_OK;
  uint8_t isAlive = 0;

  /* Set up the platform layer */
  memset(pdev, 0, sizeof(VL53L8CX_Configuration));
  pdev->platform.I2Cx = I2Cx;
  pdev->platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;  /* 0x52 (8-bit) */

  /* Check sensor is alive at the default address */
  status = vl53l8cx_is_alive(pdev, &isAlive);
  if (status != VL53L8CX_STATUS_OK || !isAlive)
  {
    DEBUG_PRINT("VL53L8CX: not alive at 0x%02X (%u)\n", VL53L8CX_DEFAULT_I2C_ADDRESS, status);
    return false;
  }

  /* Full initialization: loads firmware, configures sensor */
  status = vl53l8cx_init(pdev);
  if (status != VL53L8CX_STATUS_OK)
  {
    DEBUG_PRINT("VL53L8CX: init failed (%u)\n", status);
    return false;
  }

  return true;
}


bool vl53l8cxTestConnection(VL53L8CX_Configuration *pdev)
{
  uint8_t isAlive = 0;
  uint8_t status = vl53l8cx_is_alive(pdev, &isAlive);
  return (status == VL53L8CX_STATUS_OK) && isAlive;
}


uint8_t vl53l8cxSetI2CAddress(VL53L8CX_Configuration *pdev, uint16_t address)
{
  uint8_t status = vl53l8cx_set_i2c_address(pdev, address);
  if (status == VL53L8CX_STATUS_OK)
  {
    pdev->platform.address = address;
  }
  return status;
}
