#include "sensor_supervision.h"

#include "uart_console.h"

#include <stddef.h>

#define IMU_STALE_TIMEOUT_MS         250U
#define IMU_RETRY_INTERVAL_MS        1000U
#define BAROMETER_STALE_TIMEOUT_MS   500U
#define BAROMETER_RETRY_INTERVAL_MS  1000U
#define GPS_STALE_TIMEOUT_MS         3000U
#define GPS_RETRY_INTERVAL_MS        2000U
#define SENSOR_FAILURE_LIMIT         3U

static bool valid_sensor(SupervisedSensor sensor)
{
  return (uint32_t)sensor < (uint32_t)SUPERVISED_SENSOR_COUNT;
}

static void initialize_health(SensorHealth *health, uint32_t now_ms,
                              uint32_t stale_ms, uint32_t retry_ms,
                              bool online)
{
  SensorHealth_Init(health, now_ms, stale_ms, retry_ms, SENSOR_FAILURE_LIMIT);
  if (online)
  {
    SensorHealth_RecordSuccess(health, now_ms);
  }
  else
  {
    SensorHealth_MarkOffline(health, now_ms);
  }
}

void SensorSupervision_Init(SensorSupervision *supervision, Imu *imu,
                            Barometer *barometer, GpsService *gps,
                            Stm32UartStream *gps_uart, uint32_t now_ms,
                            bool imu_online, bool barometer_online,
                            bool gps_online)
{
  if (supervision == NULL)
  {
    return;
  }
  supervision->imu = imu;
  supervision->barometer = barometer;
  supervision->gps = gps;
  supervision->gps_uart = gps_uart;
  initialize_health(&supervision->health[SUPERVISED_IMU], now_ms,
                    IMU_STALE_TIMEOUT_MS, IMU_RETRY_INTERVAL_MS, imu_online);
  initialize_health(&supervision->health[SUPERVISED_BAROMETER], now_ms,
                    BAROMETER_STALE_TIMEOUT_MS,
                    BAROMETER_RETRY_INTERVAL_MS, barometer_online);
  initialize_health(&supervision->health[SUPERVISED_GPS], now_ms,
                    GPS_STALE_TIMEOUT_MS, GPS_RETRY_INTERVAL_MS, gps_online);
}

void SensorSupervision_Report(SensorSupervision *supervision,
                              SupervisedSensor sensor,
                              SensorSampleResult result, uint32_t now_ms)
{
  if ((supervision == NULL) || !valid_sensor(sensor) ||
      (result == SENSOR_SAMPLE_NOT_READY))
  {
    return;
  }
  SensorHealth *health = &supervision->health[sensor];
  if (result == SENSOR_SAMPLE_SUCCESS)
  {
    SensorHealth_RecordSuccess(health, now_ms);
    return;
  }

  SensorHealth_RecordFailure(health, now_ms);
  if (health->state != SENSOR_HEALTH_OFFLINE)
  {
    return;
  }
  if ((sensor == SUPERVISED_IMU) && (supervision->imu != NULL))
  {
    Imu_Invalidate(supervision->imu);
  }
  else if ((sensor == SUPERVISED_BAROMETER) &&
           (supervision->barometer != NULL))
  {
    Barometer_Invalidate(supervision->barometer);
  }
  else if ((sensor == SUPERVISED_GPS) && (supervision->gps != NULL))
  {
    GpsService_Invalidate(supervision->gps);
  }
}

void SensorSupervision_Check(SensorSupervision *supervision, uint32_t now_ms)
{
  if (supervision == NULL)
  {
    return;
  }
  for (uint32_t i = 0U; i < (uint32_t)SUPERVISED_SENSOR_COUNT; ++i)
  {
    SensorHealth_Update(&supervision->health[i], now_ms);
  }

  SensorHealth *gps_health = &supervision->health[SUPERVISED_GPS];
  if ((gps_health->state == SENSOR_HEALTH_STALE) ||
      (gps_health->state == SENSOR_HEALTH_OFFLINE))
  {
    GpsService_Invalidate(supervision->gps);
  }
  if (SensorHealth_ShouldRetry(gps_health, now_ms))
  {
    (void)Stm32UartStream_Recover(supervision->gps_uart);
    SensorHealth_MarkOffline(gps_health, now_ms);
  }

  SensorHealth *imu_health = &supervision->health[SUPERVISED_IMU];
  if (SensorHealth_ShouldRetry(imu_health, now_ms))
  {
    if (Imu_Reinitialize(supervision->imu) == ICM20948_OK)
    {
      SensorHealth_RecordSuccess(imu_health, now_ms);
      UartConsole_WriteLine("INFO: ICM-20948 recovered");
    }
    else
    {
      SensorHealth_MarkOffline(imu_health, now_ms);
    }
  }

  SensorHealth *barometer_health =
      &supervision->health[SUPERVISED_BAROMETER];
  if (SensorHealth_ShouldRetry(barometer_health, now_ms))
  {
    if (Barometer_Reinitialize(supervision->barometer) == BMP390_OK)
    {
      SensorHealth_RecordSuccess(barometer_health, now_ms);
      UartConsole_WriteLine("INFO: BMP3xx recovered");
    }
    else
    {
      SensorHealth_MarkOffline(barometer_health, now_ms);
    }
  }
}

bool SensorSupervision_IsOperational(const SensorSupervision *supervision,
                                     SupervisedSensor sensor)
{
  if ((supervision == NULL) || !valid_sensor(sensor))
  {
    return false;
  }
  const SensorHealth_State state = supervision->health[sensor].state;
  return (state == SENSOR_HEALTH_OK) || (state == SENSOR_HEALTH_DEGRADED);
}

const SensorHealth *SensorSupervision_GetHealth(
    const SensorSupervision *supervision, SupervisedSensor sensor)
{
  return ((supervision != NULL) && valid_sensor(sensor))
             ? &supervision->health[sensor] : NULL;
}
