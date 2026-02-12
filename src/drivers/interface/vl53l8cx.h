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
 * vl53l8cx.h: VL53L8CX Time-of-Flight multi-zone sensor driver interface
 */

#ifndef _VL53L8CX_H_
#define _VL53L8CX_H_

#include "vl53l8cx_api.h"
#include "i2cdev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the VL53L8CX sensor.
 *
 * Assigns a unique I2C address (from the ranger address pool) and runs
 * the full VL53L8CX initialization sequence (firmware upload, etc.).
 *
 * @param pdev   Pointer to the VL53L8CX_Configuration structure.
 * @param I2Cx   Crazyflie I2C bus to use (e.g. I2C1_DEV).
 * @return true on success, false on failure.
 */
bool vl53l8cxInit(VL53L8CX_Configuration *pdev, I2C_Dev *I2Cx);

/**
 * @brief Test the sensor connection.
 *
 * @param pdev   Pointer to the VL53L8CX_Configuration structure.
 * @return true if the sensor responds correctly.
 */
bool vl53l8cxTestConnection(VL53L8CX_Configuration *pdev);

/**
 * @brief Set the I2C address of the sensor.
 *
 * The address is in 8-bit format (ST convention, includes R/W bit).
 * After calling this, all subsequent communication uses the new address.
 *
 * @param pdev    Pointer to the VL53L8CX_Configuration structure.
 * @param address New 8-bit I2C address.
 * @return 0 on success, non-zero on error.
 */
uint8_t vl53l8cxSetI2CAddress(VL53L8CX_Configuration *pdev, uint16_t address);

#ifdef __cplusplus
}
#endif

#endif /* _VL53L8CX_H_ */
