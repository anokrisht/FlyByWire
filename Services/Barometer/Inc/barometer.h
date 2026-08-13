#ifndef BAROMETER_H
#define BAROMETER_H

#include "bmp390.h"

#include <stdbool.h>

typedef struct
{
  Bmp390 sensor;
  Bmp390_Data data;
  float sea_level_pressure_pa;
  float altitude_m;
  bool data_valid;
} Barometer;

Bmp390_Status Barometer_Init(Barometer *barometer, const I2cBus *bus,
                             uint8_t address);
Bmp390_Status Barometer_Reinitialize(Barometer *barometer);
Bmp390_Status Barometer_Update(Barometer *barometer);
const Bmp390_Data *Barometer_GetData(const Barometer *barometer);
float Barometer_GetAltitude(const Barometer *barometer);
void Barometer_SetSeaLevelPressure(Barometer *barometer, float pressure_pa);
void Barometer_Invalidate(Barometer *barometer);

#endif /* BAROMETER_H */
