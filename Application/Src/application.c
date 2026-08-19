#include "application.h"

#include "mavlink_telemetry.h"
#include "uart_console.h"

static MavlinkTelemetry telemetry;

void Application_Init(I2C_HandleTypeDef *i2c, ADC_HandleTypeDef *adc,
                      UART_HandleTypeDef *console_uart,
                      UART_HandleTypeDef *gps_uart)
{
  DataAcquisition_Init(i2c, adc, console_uart, gps_uart);
  /* From this point onward the console UART carries binary MAVLink only. */
  UartConsole_SetOutputEnabled(false);
  MavlinkTelemetry_Init(&telemetry, console_uart, HAL_GetTick());
}

void Application_Run(void)
{
  DataAcquisition_Run();
  MavlinkTelemetry_Run(&telemetry, DataAcquisition_GetData(), HAL_GetTick());
}

const DataAcquisitionData *Application_GetData(void)
{
  return DataAcquisition_GetData();
}
