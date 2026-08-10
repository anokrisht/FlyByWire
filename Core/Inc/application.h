#ifndef APPLICATION_H
#define APPLICATION_H

#include "stm32f4xx_hal.h"

void Application_Init(I2C_HandleTypeDef *i2c, UART_HandleTypeDef *uart);
void Application_Run(void);

#endif /* APPLICATION_H */
