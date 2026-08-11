#ifndef ICM20948_H
#define ICM20948_H

#include "i2c_bus.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  ICM20948_OK = 0,
  ICM20948_ERROR_ARGUMENT,
  ICM20948_ERROR_BUS,
  ICM20948_ERROR_ID,
  ICM20948_NOT_READY,
  ICM20948_MAGNETOMETER_OVERFLOW
} Icm20948_Status;

typedef struct
{
  int16_t acceleration[3];
  int16_t angular_rate[3];
  int16_t magnetic_field[3];
  int16_t temperature;
} Icm20948_RawData;

typedef struct
{
  float acceleration_mps2[3];
  float angular_rate_rps[3];
  float magnetic_field_ut[3];
  float temperature_c;
} Icm20948_Data;

typedef struct
{
  float acceleration_bias_mps2[3];
  float angular_rate_bias_rps[3];
  float magnetic_offset_ut[3];
  float magnetic_scale[3];
} Icm20948_Calibration;

typedef struct
{
  I2cBus bus;
  uint8_t address;
  uint8_t selected_bank;
  Icm20948_Calibration calibration;
  bool initialized;
} Icm20948;

/** Initialize an ICM-20948 at address 0x68 or 0x69. */
Icm20948_Status Icm20948_Init(Icm20948 *device, const I2cBus *bus,
                              uint8_t address);
Icm20948_Status Icm20948_ReadRaw(Icm20948 *device, Icm20948_RawData *data);
Icm20948_Status Icm20948_Convert(const Icm20948 *device,
                                 const Icm20948_RawData *raw,
                                 Icm20948_Data *data);
Icm20948_Status Icm20948_Read(Icm20948 *device, Icm20948_Data *data);
void Icm20948_SetCalibration(Icm20948 *device,
                             const Icm20948_Calibration *calibration);

#endif /* ICM20948_H */
