#ifndef BMP390_H
#define BMP390_H

#include "i2c_bus.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  BMP390_OK = 0,
  BMP390_ERROR_ARGUMENT,
  BMP390_ERROR_BUS,
  BMP390_ERROR_ID,
  BMP390_NOT_READY
} Bmp390_Status;

typedef struct
{
  double t1, t2, t3;
  double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
} Bmp390_Calibration;

typedef struct
{
  float pressure_pa;
  float temperature_c;
} Bmp390_Data;

typedef struct
{
  I2cBus bus;
  Bmp390_Calibration calibration;
  uint8_t address;
  uint8_t chip_id;
  bool initialized;
} Bmp390;

/** Initialize a BMP390 at address 0x76 (SDO low) or 0x77 (SDO high). */
Bmp390_Status Bmp390_Init(Bmp390 *device, const I2cBus *bus, uint8_t address);
Bmp390_Status Bmp390_Read(Bmp390 *device, Bmp390_Data *data);
const char *Bmp390_StatusName(Bmp390_Status status);
const char *Bmp390_ModelName(const Bmp390 *device);

#endif /* BMP390_H */
