#include "barometer.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define STANDARD_SEA_LEVEL_PRESSURE_PA 101325.0F

Bmp390_Status Barometer_Init(Barometer *barometer, const I2cBus *bus,
                             uint8_t address)
{
  if (barometer == NULL)
  {
    return BMP390_ERROR_ARGUMENT;
  }
  memset(barometer, 0, sizeof(*barometer));
  barometer->sea_level_pressure_pa = STANDARD_SEA_LEVEL_PRESSURE_PA;
  return Bmp390_Init(&barometer->sensor, bus, address);
}

Bmp390_Status Barometer_Reinitialize(Barometer *barometer)
{
  if (barometer == NULL)
  {
    return BMP390_ERROR_ARGUMENT;
  }
  const I2cBus bus = barometer->sensor.bus;
  const uint8_t address = barometer->sensor.address;
  const float sea_level_pressure = barometer->sea_level_pressure_pa;
  Barometer candidate;
  const Bmp390_Status status = Barometer_Init(&candidate, &bus, address);
  if (status == BMP390_OK)
  {
    candidate.sea_level_pressure_pa = sea_level_pressure;
    *barometer = candidate;
  }
  return status;
}

Bmp390_Status Barometer_Update(Barometer *barometer)
{
  if (barometer == NULL)
  {
    return BMP390_ERROR_ARGUMENT;
  }
  const Bmp390_Status status = Bmp390_Read(&barometer->sensor, &barometer->data);
  if (status == BMP390_OK)
  {
    barometer->altitude_m = 44330.0F *
        (1.0F - powf(barometer->data.pressure_pa /
                     barometer->sea_level_pressure_pa, 0.19029496F));
    barometer->data_valid = true;
  }
  return status;
}

const Bmp390_Data *Barometer_GetData(const Barometer *barometer)
{
  return ((barometer != NULL) && barometer->data_valid) ? &barometer->data : NULL;
}

float Barometer_GetAltitude(const Barometer *barometer)
{
  return ((barometer != NULL) && barometer->data_valid) ?
      barometer->altitude_m : 0.0F;
}

void Barometer_SetSeaLevelPressure(Barometer *barometer, float pressure_pa)
{
  if ((barometer != NULL) && (pressure_pa > 0.0F))
  {
    barometer->sea_level_pressure_pa = pressure_pa;
  }
}

void Barometer_Invalidate(Barometer *barometer)
{
  if (barometer != NULL)
  {
    barometer->data_valid = false;
  }
}
