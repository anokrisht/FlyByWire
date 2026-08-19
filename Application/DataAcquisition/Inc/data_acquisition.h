#ifndef DATA_ACQUISITION_H
#define DATA_ACQUISITION_H

#include "barometer.h"
#include "gps_service.h"
#include "imu.h"
#include "sensor_health.h"
#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

/** Coherent, read-only snapshot produced by the acquisition subsystem. */
typedef struct
{
  uint32_t timestamp_ms;
  Icm20948_Data imu;
  Imu_Orientation orientation;
  bool imu_valid;
  bool orientation_valid;
  Bmp390_Data barometer;
  float altitude_m;
  bool barometer_valid;
  float indicated_airspeed_mps;
  float differential_pressure_pa;
  uint16_t airspeed_raw_adc;
  bool airspeed_valid;
  GpsData gps;
  bool gps_valid;
  SensorHealth imu_health;
  SensorHealth barometer_health;
  SensorHealth gps_health;
} DataAcquisitionData;

void DataAcquisition_Init(I2C_HandleTypeDef *i2c, ADC_HandleTypeDef *adc,
                          UART_HandleTypeDef *console_uart,
                          UART_HandleTypeDef *gps_uart);
void DataAcquisition_Run(void);
const DataAcquisitionData *DataAcquisition_GetData(void);

#endif /* DATA_ACQUISITION_H */
