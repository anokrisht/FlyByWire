#ifndef APPLICATION_H
#define APPLICATION_H

#include "stm32f4xx_hal.h"
#include "data_acquisition.h"

void Application_Init(I2C_HandleTypeDef *i2c, ADC_HandleTypeDef *adc,
                      UART_HandleTypeDef *console_uart,
                      UART_HandleTypeDef *gps_uart);
void Application_Run(void);
/** Latest data-acquisition snapshot for all current and future sections. */
const DataAcquisitionData *Application_GetData(void);

#endif /* APPLICATION_H */
