#include "application.h"

#include "imu.h"
#include "main.h"
#include "gps_service.h"
#include "sensor_health.h"
#include "stm32_i2c_bus.h"
#include "stm32_uart_stream.h"
#include "uart_console.h"

#include <stdio.h>
#include <string.h>

#define ICM20948_I2C_ADDRESS 0x68U
#define IMU_SAMPLE_PERIOD_MS  10U
#define TELEMETRY_PERIOD_MS   100U
#define IMU_STALE_TIMEOUT_MS  250U
#define IMU_RETRY_INTERVAL_MS 1000U
#define GPS_STALE_TIMEOUT_MS  3000U
#define GPS_RETRY_INTERVAL_MS 2000U
#define SENSOR_FAILURE_LIMIT  3U

static Imu imu;
static GpsService gps;
static I2cBus imu_i2c_bus;
static Stm32UartStream gps_uart_stream;
static SensorHealth imu_health;
static SensorHealth gps_health;
static uint32_t last_sample_ms;
static uint32_t last_telemetry_ms;

static const char *gps_fix_name(const GpsData *data, bool fix_valid)
{
  if (!fix_valid)
  {
    return "NO FIX";
  }
  if (data->fix_dimension == GPS_FIX_3D)
  {
    return "3D FIX";
  }
  if (data->fix_dimension == GPS_FIX_2D)
  {
    return "2D FIX";
  }
  return "FIX";
}

static void print_gps_fix(void)
{
  GpsCoordinates coordinates = {0};
  GpsTime utc = {0};
  const GpsData *data = GpsService_GetData(&gps);

  if (data != NULL)
  {
    const bool fix_valid =
        GpsService_GetCoordinates(&gps, &coordinates);
    (void)GpsService_GetUtcTime(&gps, &utc);

    printf("GPS %s: UTC %02u:%02u:%02u | %.7f, %.7f | "
           "ALT %.1f m | SPEED %.2f m/s | COURSE %.1f deg | "
           "SAT %u/%u | DOP %.2f/%.2f/%.2f\r\n",
           gps_fix_name(data, fix_valid),
           utc.hours, utc.minutes, utc.seconds,
           coordinates.latitude_deg, coordinates.longitude_deg,
           (fix_valid && data->altitude_valid) ? data->altitude_m : 0.0F,
           (fix_valid && data->speed_valid) ? data->speed_mps : 0.0F,
           (fix_valid && data->course_valid) ? data->course_deg : 0.0F,
           data->satellites, data->satellites_in_view,
           data->dilution_valid ? data->position_dilution : 0.0F,
           data->dilution_valid ? data->horizontal_dilution : 0.0F,
           data->dilution_valid ? data->vertical_dilution : 0.0F);
  }
}

void Application_Init(I2C_HandleTypeDef *i2c, UART_HandleTypeDef *console_uart,
                      UART_HandleTypeDef *gps_uart)
{
  UartConsole_Init(console_uart);
  const uint32_t now = HAL_GetTick();
  SensorHealth_Init(&imu_health, now, IMU_STALE_TIMEOUT_MS,
                    IMU_RETRY_INTERVAL_MS, SENSOR_FAILURE_LIMIT);
  SensorHealth_Init(&gps_health, now, GPS_STALE_TIMEOUT_MS,
                    GPS_RETRY_INTERVAL_MS, SENSOR_FAILURE_LIMIT);
  imu_i2c_bus = Stm32I2cBus_Create(i2c);

  const bool gps_uart_started =
      Stm32UartStream_Start(&gps_uart_stream, gps_uart);
  const ByteStream gps_stream =
      Stm32UartStream_AsByteStream(&gps_uart_stream);
  if (!GpsService_Init(&gps, &gps_stream))
  {
    UartConsole_WriteLine("FATAL: invalid GPS driver configuration");
    Error_Handler();
  }
  if (!gps_uart_started)
  {
    SensorHealth_MarkOffline(&gps_health, now);
    UartConsole_WriteLine("WARN: GPS UART offline; recovery scheduled");
  }

  if (Imu_Init(&imu, &imu_i2c_bus, ICM20948_I2C_ADDRESS) == ICM20948_OK)
  {
    SensorHealth_RecordSuccess(&imu_health, HAL_GetTick());
  }
  else
  {
    SensorHealth_MarkOffline(&imu_health, HAL_GetTick());
    UartConsole_WriteLine("WARN: ICM-20948 offline; recovery scheduled");
  }

  UartConsole_WriteLine("FlyByWire: sensor supervision active");
}

void Application_Run(void)
{
  uint32_t now = HAL_GetTick();
  while (GpsService_Update(&gps))
  {
    SensorHealth_RecordSuccess(&gps_health, now);
    const char *sentence = GpsService_GetRawSentence(&gps);
    if ((sentence != NULL) && (strlen(sentence) >= 6U) &&
        (strncmp(&sentence[3], "RMC", 3U) == 0))
    {
      print_gps_fix();
    }
  }

  SensorHealth_Update(&gps_health, now);
  if ((gps_health.state == SENSOR_HEALTH_STALE) ||
      (gps_health.state == SENSOR_HEALTH_OFFLINE))
  {
    GpsService_Invalidate(&gps);
  }
  if (SensorHealth_ShouldRetry(&gps_health, now))
  {
    (void)Stm32UartStream_Recover(&gps_uart_stream);
    SensorHealth_MarkOffline(&gps_health, now);
  }

  SensorHealth_Update(&imu_health, now);
  if (SensorHealth_ShouldRetry(&imu_health, now))
  {
    if (Imu_Reinitialize(&imu) == ICM20948_OK)
    {
      SensorHealth_RecordSuccess(&imu_health, HAL_GetTick());
      UartConsole_WriteLine("INFO: ICM-20948 recovered");
    }
    else
    {
      SensorHealth_MarkOffline(&imu_health, HAL_GetTick());
    }
    return;
  }

  if ((uint32_t)(now - last_sample_ms) < IMU_SAMPLE_PERIOD_MS)
  {
    return;
  }
  last_sample_ms = now;

  const Icm20948_Status status = Imu_Update(&imu, now);
  if (status == ICM20948_OK)
  {
    SensorHealth_RecordSuccess(&imu_health, now);
  }
  else if (status != ICM20948_NOT_READY)
  {
    SensorHealth_RecordFailure(&imu_health, now);
    if (imu_health.state == SENSOR_HEALTH_OFFLINE)
    {
      Imu_Invalidate(&imu);
    }
    return;
  }
  else
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

const SensorHealth *Application_GetImuHealth(void)
{
  return &imu_health;
}

const SensorHealth *Application_GetGpsHealth(void)
{
  return &gps_health;
}
