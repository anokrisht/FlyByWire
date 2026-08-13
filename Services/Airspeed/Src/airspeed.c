#include "airspeed.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define ADC_FULL_SCALE_COUNTS       4095.0F
#define ADC_REFERENCE_VOLTAGE       3.3F
#define SENSOR_SUPPLY_VOLTAGE       5.0F
/* 1 kOhm from sensor output to ADC, 2 kOhm from ADC to ground. */
#define DIVIDER_TOP_OHMS            1000.0F
#define DIVIDER_BOTTOM_OHMS         2000.0F
#define MPXV7002_SENSITIVITY_PER_KPA 0.2F
#define ZERO_SAMPLE_COUNT           128U
#define UPDATE_SAMPLE_COUNT         16U
#define ADC_TIMEOUT_MS              2U
#define STANDARD_AIR_DENSITY_KG_M3  1.225F

static Airspeed_Status sample_adc(Airspeed *airspeed, uint32_t samples,
                                  float *average_counts)
{
  uint32_t sum = 0U;
  for (uint32_t i = 0U; i < samples; ++i)
  {
    if ((HAL_ADC_Start(airspeed->adc) != HAL_OK) ||
        (HAL_ADC_PollForConversion(airspeed->adc, ADC_TIMEOUT_MS) != HAL_OK))
    {
      (void)HAL_ADC_Stop(airspeed->adc);
      return AIRSPEED_ERROR_ADC;
    }
    sum += HAL_ADC_GetValue(airspeed->adc);
    (void)HAL_ADC_Stop(airspeed->adc);
  }
  *average_counts = (float)sum / (float)samples;
  return AIRSPEED_OK;
}

Airspeed_Status Airspeed_Init(Airspeed *airspeed, ADC_HandleTypeDef *adc)
{
  if ((airspeed == NULL) || (adc == NULL))
  {
    return AIRSPEED_ERROR_ARGUMENT;
  }
  memset(airspeed, 0, sizeof(*airspeed));
  airspeed->adc = adc;
  HAL_Delay(25U);
  return sample_adc(airspeed, ZERO_SAMPLE_COUNT, &airspeed->zero_adc_counts);
}

Airspeed_Status Airspeed_Update(Airspeed *airspeed, float air_density_kg_m3)
{
  if ((airspeed == NULL) || (airspeed->adc == NULL))
  {
    return AIRSPEED_ERROR_ARGUMENT;
  }

  float average_counts = 0.0F;
  const Airspeed_Status status =
      sample_adc(airspeed, UPDATE_SAMPLE_COUNT, &average_counts);
  if (status != AIRSPEED_OK)
  {
    airspeed->data_valid = false;
    return status;
  }

  const float divider_gain =
      (DIVIDER_TOP_OHMS + DIVIDER_BOTTOM_OHMS) / DIVIDER_BOTTOM_OHMS;
  const float sensor_voltage_delta =
      (average_counts - airspeed->zero_adc_counts) *
      ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE_COUNTS * divider_gain;
  airspeed->differential_pressure_pa =
      sensor_voltage_delta / SENSOR_SUPPLY_VOLTAGE /
      MPXV7002_SENSITIVITY_PER_KPA * 1000.0F;

  const float density = (air_density_kg_m3 > 0.1F) ?
      air_density_kg_m3 : STANDARD_AIR_DENSITY_KG_M3;
  const float dynamic_pressure = (airspeed->differential_pressure_pa > 0.0F) ?
      airspeed->differential_pressure_pa : 0.0F;
  airspeed->indicated_airspeed_mps = sqrtf(2.0F * dynamic_pressure / density);
  airspeed->raw_adc = (uint16_t)(average_counts + 0.5F);
  airspeed->data_valid = true;
  return AIRSPEED_OK;
}

void Airspeed_Invalidate(Airspeed *airspeed)
{
  if (airspeed != NULL)
  {
    airspeed->data_valid = false;
  }
}
