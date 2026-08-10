#include "stm32_i2c_bus.h"

#include <limits.h>

#define STM32_I2C_TIMEOUT_MS 100U

static I2cBus_Status map_status(HAL_StatusTypeDef status)
{
  if (status == HAL_OK)
  {
    return I2C_BUS_OK;
  }
  return (status == HAL_TIMEOUT) ? I2C_BUS_TIMEOUT : I2C_BUS_ERROR;
}

static I2cBus_Status read_registers(void *context, uint8_t address,
                                    uint8_t reg, uint8_t *data, size_t length)
{
  if ((context == NULL) || (data == NULL) || (length > UINT16_MAX))
  {
    return I2C_BUS_ERROR;
  }

  return map_status(HAL_I2C_Mem_Read(
      context, (uint16_t)address << 1, reg, I2C_MEMADD_SIZE_8BIT, data,
      (uint16_t)length, STM32_I2C_TIMEOUT_MS));
}

static I2cBus_Status write_registers(void *context, uint8_t address,
                                     uint8_t reg, const uint8_t *data,
                                     size_t length)
{
  if ((context == NULL) || (data == NULL) || (length > UINT16_MAX))
  {
    return I2C_BUS_ERROR;
  }

  return map_status(HAL_I2C_Mem_Write(
      context, (uint16_t)address << 1, reg, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)data, (uint16_t)length, STM32_I2C_TIMEOUT_MS));
}

I2cBus Stm32I2cBus_Create(I2C_HandleTypeDef *handle)
{
  const I2cBus bus = {
      .context = handle,
      .read_registers = read_registers,
      .write_registers = write_registers,
      .delay_ms = HAL_Delay,
  };
  return bus;
}
