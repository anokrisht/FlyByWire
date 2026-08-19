#ifndef DATA_ACQUISITION_SENSOR_SUPERVISION_H
#define DATA_ACQUISITION_SENSOR_SUPERVISION_H

#include "barometer.h"
#include "gps_service.h"
#include "imu.h"
#include "sensor_health.h"
#include "stm32_uart_stream.h"

typedef enum
{
  SUPERVISED_IMU = 0,
  SUPERVISED_BAROMETER,
  SUPERVISED_GPS,
  SUPERVISED_SENSOR_COUNT
} SupervisedSensor;

typedef enum
{
  SENSOR_SAMPLE_SUCCESS = 0,
  SENSOR_SAMPLE_NOT_READY,
  SENSOR_SAMPLE_FAILURE
} SensorSampleResult;

typedef struct
{
  SensorHealth health[SUPERVISED_SENSOR_COUNT];
  Imu *imu;
  Barometer *barometer;
  GpsService *gps;
  Stm32UartStream *gps_uart;
} SensorSupervision;

void SensorSupervision_Init(SensorSupervision *supervision, Imu *imu,
                            Barometer *barometer, GpsService *gps,
                            Stm32UartStream *gps_uart, uint32_t now_ms,
                            bool imu_online, bool barometer_online,
                            bool gps_online);

/** Perform stale detection, invalidation, and all scheduled recovery attempts. */
void SensorSupervision_Check(SensorSupervision *supervision, uint32_t now_ms);

void SensorSupervision_Report(SensorSupervision *supervision,
                              SupervisedSensor sensor,
                              SensorSampleResult result, uint32_t now_ms);
bool SensorSupervision_IsOperational(const SensorSupervision *supervision,
                                     SupervisedSensor sensor);
const SensorHealth *SensorSupervision_GetHealth(
    const SensorSupervision *supervision, SupervisedSensor sensor);

#endif /* DATA_ACQUISITION_SENSOR_SUPERVISION_H */
