#include "stm32_i2c_bus.h"

#include <limits.h>

#define STM32_I2C_TIMEOUT_MS 100U
#define I2C_RECOVERY_PULSES   9U
#define I2C_RECOVERY_DELAY_MS 1U

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

static void recover_i2c1_lines(void)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(I2C_RECOVERY_DELAY_MS);
  for (uint8_t pulse = 0U;
       (pulse < I2C_RECOVERY_PULSES) &&
       (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET);
       ++pulse)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(I2C_RECOVERY_DELAY_MS);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(I2C_RECOVERY_DELAY_MS);
  }

  /* Generate a STOP: SDA transitions low-to-high while SCL is high. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(I2C_RECOVERY_DELAY_MS);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_Delay(I2C_RECOVERY_DELAY_MS);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(I2C_RECOVERY_DELAY_MS);
}

static bool recover_bus(void *context)
{
  I2C_HandleTypeDef *handle = context;
  if (handle == NULL)
  {
    return false;
  }

  (void)HAL_I2C_DeInit(handle);
  if (handle->Instance == I2C1)
  {
    recover_i2c1_lines();
  }
  return HAL_I2C_Init(handle) == HAL_OK;
}

I2cBus Stm32I2cBus_Create(I2C_HandleTypeDef *handle)
{
  const I2cBus bus = {
      .context = handle,
      .read_registers = read_registers,
      .write_registers = write_registers,
      .delay_ms = HAL_Delay,
      .recover = recover_bus,
  };
  return bus;
}
