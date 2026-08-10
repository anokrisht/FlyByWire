#include "application.h"

#include "imu.h"
#include "main.h"
#include "neo6m.h"
#include "stm32_i2c_bus.h"
#include "stm32_uart_stream.h"
#include "uart_console.h"

#include <stdio.h>
#include <string.h>

#define ICM20948_I2C_ADDRESS 0x68U
#define IMU_SAMPLE_PERIOD_MS  10U
#define TELEMETRY_PERIOD_MS   100U

static Imu imu;
static Neo6m gps;
static Stm32UartStream gps_uart_stream;
static uint32_t last_sample_ms;
static uint32_t last_telemetry_ms;

static void print_gps_fix(void)
{
  Neo6m_Coordinates coordinates;
  Neo6m_Time utc;
  const Neo6m_Data *data = Neo6m_GetData(&gps);

  if ((data != NULL) && Neo6m_GetCoordinates(&gps, &coordinates) &&
      Neo6m_GetUtcTime(&gps, &utc))
  {
    printf("GPS FIX: UTC %02u:%02u:%02u | %.7f, %.7f | "
           "ALT %.1f m | SPEED %.2f m/s | SAT %u\r\n",
           utc.hours, utc.minutes, utc.seconds,
           coordinates.latitude_deg, coordinates.longitude_deg,
           data->altitude_valid ? data->altitude_m : 0.0F,
           data->speed_valid ? data->speed_mps : 0.0F,
           data->satellites);
  }
}

void Application_Init(I2C_HandleTypeDef *i2c, UART_HandleTypeDef *console_uart,
                      UART_HandleTypeDef *gps_uart)
{
  UartConsole_Init(console_uart);
  const I2cBus i2c_bus = Stm32I2cBus_Create(i2c);
  if (!Stm32UartStream_Start(&gps_uart_stream, gps_uart))
  {
    UartConsole_WriteLine("ERROR: GPS UART reception failed");
    Error_Handler();
  }
  const ByteStream gps_stream =
      Stm32UartStream_AsByteStream(&gps_uart_stream);
  if (!Neo6m_Init(&gps, &gps_stream))
  {
    UartConsole_WriteLine("ERROR: GPS driver initialization failed");
    Error_Handler();
  }

  if (Imu_Init(&imu, &i2c_bus, ICM20948_I2C_ADDRESS) != ICM20948_OK)
  {
    UartConsole_WriteLine("ERROR: ICM-20948 initialization failed");
    Error_Handler();
  }

  UartConsole_WriteLine("FlyByWire: IMU and GPS interfaces ready");
}

void Application_Run(void)
{
  while (Neo6m_Poll(&gps))
  {
    const char *sentence = Neo6m_GetSentence(&gps);
    UartConsole_WriteString("GPS: ");
    UartConsole_WriteLine(sentence);
    if ((sentence != NULL) && (strlen(sentence) >= 6U) &&
        (strncmp(&sentence[3], "RMC", 3U) == 0))
    {
      print_gps_fix();
    }
  }

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
      /*printf("A[m/s2] %.2f %.2f %.2f | RPY[deg] %.1f %.1f %.1f | H %.1f\r\n",
             data->acceleration_mps2[0], data->acceleration_mps2[1],
             data->acceleration_mps2[2], orientation->roll_deg,
             orientation->pitch_deg, orientation->yaw_deg,
             orientation->heading_deg);*/
    }
  }
}
