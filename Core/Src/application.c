#include "application.h"

#include "imu.h"
#include "main.h"
#include "stm32_i2c_bus.h"
#include "uart_console.h"

#include <stdio.h>

#define ICM20948_I2C_ADDRESS 0x68U
#define IMU_SAMPLE_PERIOD_MS  10U
#define TELEMETRY_PERIOD_MS   100U

static Imu imu;
static uint32_t last_sample_ms;
static uint32_t last_telemetry_ms;

void Application_Init(I2C_HandleTypeDef *i2c, UART_HandleTypeDef *uart)
{
  UartConsole_Init(uart);
  const I2cBus i2c_bus = Stm32I2cBus_Create(i2c);

  if (Imu_Init(&imu, &i2c_bus, ICM20948_I2C_ADDRESS) != ICM20948_OK)
  {
    UartConsole_WriteLine("ERROR: ICM-20948 initialization failed");
    Error_Handler();
  }

  UartConsole_WriteLine("FlyByWire: ICM-20948 ready");
}

void Application_Run(void)
{
  const uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - last_sample_ms) < IMU_SAMPLE_PERIOD_MS)
  {
    return;
  }
  last_sample_ms = now;

  const Icm20948_Status status = Imu_Update(&imu, now);
  if ((status != ICM20948_OK) && (status != ICM20948_NOT_READY))
  {
    return;
  }

  if ((uint32_t)(now - last_telemetry_ms) >= TELEMETRY_PERIOD_MS)
  {
    last_telemetry_ms = now;
    const Icm20948_Data *data = Imu_GetData(&imu);
    const Imu_Orientation *orientation = Imu_GetOrientation(&imu);
    if ((data != NULL) && (orientation != NULL))
    {
      printf("A[m/s2] %.2f %.2f %.2f | RPY[deg] %.1f %.1f %.1f | H %.1f\r\n",
             data->acceleration_mps2[0], data->acceleration_mps2[1],
             data->acceleration_mps2[2], orientation->roll_deg,
             orientation->pitch_deg, orientation->yaw_deg,
             orientation->heading_deg);
    }
  }
}
