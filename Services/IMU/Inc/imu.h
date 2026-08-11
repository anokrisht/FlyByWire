#ifndef IMU_H
#define IMU_H

#include "icm20948.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  float heading_deg;
} Imu_Orientation;

typedef struct
{
  Icm20948 sensor;
  Icm20948_RawData raw;
  Icm20948_Data data;
  Imu_Orientation orientation;
  float declination_deg;
  uint32_t previous_update_ms;
  bool data_valid;
  bool orientation_valid;
} Imu;

Icm20948_Status Imu_Init(Imu *imu, const I2cBus *bus, uint8_t address);

/** Reconfigure a reset sensor while preserving calibration and declination. */
Icm20948_Status Imu_Reinitialize(Imu *imu);

/** Sample all nine axes and update the filtered orientation. */
Icm20948_Status Imu_Update(Imu *imu, uint32_t timestamp_ms);

/** Most recent unscaled ADC readings. Valid after a successful update. */
const Icm20948_RawData *Imu_GetRaw(const Imu *imu);

/** Calibrated acceleration, angular rate, magnetic field, and temperature. */
const Icm20948_Data *Imu_GetData(const Imu *imu);

/** Filtered roll, pitch, magnetic yaw, and heading in degrees. */
const Imu_Orientation *Imu_GetOrientation(const Imu *imu);

void Imu_SetCalibration(Imu *imu, const Icm20948_Calibration *calibration);
void Imu_SetMagneticDeclination(Imu *imu, float declination_deg);
void Imu_Invalidate(Imu *imu);

#endif /* IMU_H */
