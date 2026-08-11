#ifndef STM32_I2C_BUS_H
#define STM32_I2C_BUS_H

#include "i2c_bus.h"
#include "stm32f4xx_hal.h"

/** Create an I2C bus adapter backed by an initialized STM32 HAL handle. */
I2cBus Stm32I2cBus_Create(I2C_HandleTypeDef *handle);

#endif /* STM32_I2C_BUS_H */
