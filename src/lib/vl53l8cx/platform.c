/**
 ******************************************************************************
 * @file    platform.c
 * @brief   Platform abstraction implementation for VL53L8CX on Crazyflie.
 *
 * Implements the I/O and timing functions required by the VL53L8CX ULD API,
 * using the Crazyflie's i2cdev HAL and FreeRTOS primitives.
 ******************************************************************************
 */

#include "platform.h"

#include "FreeRTOS.h"
#include "task.h"

/* --------------------------------------------------------------------------
 * Helper: convert 8-bit ST address to 7-bit i2cdev address
 * ST convention stores address with R/W bit (e.g. 0x52), while i2cdev
 * expects 7-bit left-aligned (e.g. 0x29).
 * -------------------------------------------------------------------------- */
static inline uint8_t addr7bit(const VL53L8CX_Platform *p)
{
  return (uint8_t)(p->address >> 1);
}


uint8_t VL53L8CX_RdByte(
    VL53L8CX_Platform *p_platform,
    uint16_t RegisterAddress,
    uint8_t *p_value)
{
  bool ok = i2cdevReadReg16(p_platform->I2Cx, addr7bit(p_platform),
                            RegisterAddress, 1, p_value);
  return ok ? 0 : 1;
}


uint8_t VL53L8CX_WrByte(
    VL53L8CX_Platform *p_platform,
    uint16_t RegisterAddress,
    uint8_t value)
{
  bool ok = i2cdevWriteReg16(p_platform->I2Cx, addr7bit(p_platform),
                             RegisterAddress, 1, &value);
  return ok ? 0 : 1;
}


uint8_t VL53L8CX_WrMulti(
    VL53L8CX_Platform *p_platform,
    uint16_t RegisterAddress,
    uint8_t *p_values,
    uint32_t size)
{
  /* The i2cdev layer handles DMA transfers.  For very large writes
   * (e.g. firmware upload ~80 KB), we chunk to VL53L8CX_COMMS_CHUNK_SIZE
   * to stay within the driver's capabilities.  */
  uint32_t offset = 0;

  while (offset < size)
  {
    uint32_t chunk = size - offset;
    if (chunk > VL53L8CX_COMMS_CHUNK_SIZE)
      chunk = VL53L8CX_COMMS_CHUNK_SIZE;

    bool ok = i2cdevWriteReg16(p_platform->I2Cx, addr7bit(p_platform),
                               RegisterAddress + (uint16_t)offset,
                               (uint16_t)chunk, p_values + offset);
    if (!ok)
      return 1;

    offset += chunk;
  }
  return 0;
}


uint8_t VL53L8CX_RdMulti(
    VL53L8CX_Platform *p_platform,
    uint16_t RegisterAddress,
    uint8_t *p_values,
    uint32_t size)
{
  uint32_t offset = 0;

  while (offset < size)
  {
    uint32_t chunk = size - offset;
    if (chunk > VL53L8CX_COMMS_CHUNK_SIZE)
      chunk = VL53L8CX_COMMS_CHUNK_SIZE;

    bool ok = i2cdevReadReg16(p_platform->I2Cx, addr7bit(p_platform),
                              RegisterAddress + (uint16_t)offset,
                              (uint16_t)chunk, p_values + offset);
    if (!ok)
      return 1;

    offset += chunk;
  }
  return 0;
}


void VL53L8CX_SwapBuffer(
    uint8_t  *buffer,
    uint16_t  size)
{
  uint32_t i, tmp;

  for (i = 0; i < size; i = i + 4)
  {
    tmp = (  (uint32_t)buffer[i]     << 24)
        | (  (uint32_t)buffer[i + 1] << 16)
        | (  (uint32_t)buffer[i + 2] <<  8)
        | (  (uint32_t)buffer[i + 3]);

    memcpy(&(buffer[i]), &tmp, 4);
  }
}


uint8_t VL53L8CX_WaitMs(
    VL53L8CX_Platform *p_platform,
    uint32_t TimeMs)
{
  (void)p_platform;

  if (TimeMs == 0)
    TimeMs = 1;

  vTaskDelay(M2T(TimeMs));
  return 0;
}
