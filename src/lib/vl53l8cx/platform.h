/**
 ******************************************************************************
 * @file    platform.h
 * @brief   Platform abstraction for VL53L8CX on Crazyflie (STM32 + FreeRTOS).
 *
 * This file replaces the Arduino/Teensy-specific platform.h from the
 * VL53L8CX ULD driver. It provides the VL53L8CX_Platform struct and
 * declares the platform I/O functions that the VL53L8CX API requires.
 *
 * Ported to Crazyflie by using i2cdev for I2C and FreeRTOS for timing.
 ******************************************************************************
 */

#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>

#include "platform_config.h"
#include "i2cdev.h"

/**
 * @brief Maximum chunk size for a single I2C transaction.
 * The VL53L8CX firmware upload sends large buffers; the i2cdev layer
 * handles DMA, so we allow large transfers directly.
 */
#define VL53L8CX_COMMS_CHUNK_SIZE  4096

/**
 * @brief Platform-specific device structure for the Crazyflie.
 *
 * The VL53L8CX ULD API accesses this through p_dev->platform.
 * We store the Crazyflie I2C bus pointer and the device's 8-bit I2C address
 * (including R/W bit, as per ST convention: default 0x52).
 */
typedef struct {
  uint16_t  address;   /**< 8-bit I2C address (ST convention, e.g. 0x52) */
  I2C_Dev  *I2Cx;      /**< Crazyflie I2C bus (e.g. I2C1_DEV) */
} VL53L8CX_Platform;

/**
 * @brief Read a single byte from the sensor.
 */
uint8_t VL53L8CX_RdByte(
    VL53L8CX_Platform *p_platform,
    uint16_t RegisterAddress,
    uint8_t *p_value);

/**
 * @brief Write a single byte to the sensor.
 */
uint8_t VL53L8CX_WrByte(
    VL53L8CX_Platform *p_platform,
    uint16_t RegisterAddress,
    uint8_t value);

/**
 * @brief Write multiple bytes to the sensor.
 */
uint8_t VL53L8CX_WrMulti(
    VL53L8CX_Platform *p_platform,
    uint16_t RegisterAddress,
    uint8_t *p_values,
    uint32_t size);

/**
 * @brief Read multiple bytes from the sensor.
 */
uint8_t VL53L8CX_RdMulti(
    VL53L8CX_Platform *p_platform,
    uint16_t RegisterAddress,
    uint8_t *p_values,
    uint32_t size);

/**
 * @brief Wait for a specified number of milliseconds.
 */
uint8_t VL53L8CX_WaitMs(
    VL53L8CX_Platform *p_platform,
    uint32_t TimeMs);

/**
 * @brief Swap a buffer from little-endian to big-endian (or vice versa).
 *
 * The VL53L8CX firmware uses big-endian for multi-byte values; the STM32
 * is little-endian. The API calls this to convert result data.
 */
void VL53L8CX_SwapBuffer(
    uint8_t  *buffer,
    uint16_t  size);

#ifdef __cplusplus
}
#endif

#endif  /* _PLATFORM_H_ */
