#ifndef AIRSPEED_H
#define AIRSPEED_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  AIRSPEED_OK = 0,
  AIRSPEED_ERROR_ARGUMENT,
  AIRSPEED_ERROR_ADC
} Airspeed_Status;

typedef struct
{
  ADC_HandleTypeDef *adc;
  float zero_adc_counts;
  float differential_pressure_pa;
  float indicated_airspeed_mps;
  uint16_t raw_adc;
  bool data_valid;
} Airspeed;

/** Initialize and auto-zero the sensor. Keep both pressure ports equal. */
Airspeed_Status Airspeed_Init(Airspeed *airspeed, ADC_HandleTypeDef *adc);
Airspeed_Status Airspeed_Update(Airspeed *airspeed, float air_density_kg_m3);
void Airspeed_Invalidate(Airspeed *airspeed);

#endif /* AIRSPEED_H */
