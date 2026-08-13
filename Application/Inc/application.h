#ifndef APPLICATION_H
#define APPLICATION_H

#include "stm32f4xx_hal.h"
#include "sensor_health.h"

void Application_Init(I2C_HandleTypeDef *i2c, ADC_HandleTypeDef *adc,
                      UART_HandleTypeDef *console_uart,
                      UART_HandleTypeDef *gps_uart);
void Application_Run(void);
const SensorHealth *Application_GetImuHealth(void);
const SensorHealth *Application_GetGpsHealth(void);
const SensorHealth *Application_GetBarometerHealth(void);

#endif /* APPLICATION_H */
