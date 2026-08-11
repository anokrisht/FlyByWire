#include "imu.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define RAD_TO_DEG                 57.29577951308232F
#define ORIENTATION_TIME_CONSTANT  0.5F
#define MAX_UPDATE_INTERVAL_S      0.1F

static float wrap_degrees(float angle)
{
  while (angle >= 360.0F)
  {
    angle -= 360.0F;
  }
  while (angle < 0.0F)
  {
    angle += 360.0F;
  }
  return angle;
}

static float wrap_signed_degrees(float angle)
{
  angle = wrap_degrees(angle);
  return (angle > 180.0F) ? angle - 360.0F : angle;
}

static float blend_angle(float predicted, float measured, float correction)
{
  const float error = wrap_signed_degrees(measured - predicted);
  return predicted + correction * error;
}

Icm20948_Status Imu_Init(Imu *imu, const I2cBus *bus, uint8_t address)
{
  if (imu == NULL)
  {
    return ICM20948_ERROR_ARGUMENT;
  }
  memset(imu, 0, sizeof(*imu));
  return Icm20948_Init(&imu->sensor, bus, address);
}

Icm20948_Status Imu_Reinitialize(Imu *imu)
{
  if ((imu == NULL) || (imu->sensor.bus.read_registers == NULL))
  {
    return ICM20948_ERROR_ARGUMENT;
  }

  const I2cBus bus = imu->sensor.bus;
  const uint8_t address = imu->sensor.address;
  const Icm20948_Calibration calibration = imu->sensor.calibration;
  const float declination_deg = imu->declination_deg;
  Imu candidate;
  const Icm20948_Status status = Imu_Init(&candidate, &bus, address);
  if (status == ICM20948_OK)
  {
    Icm20948_SetCalibration(&candidate.sensor, &calibration);
    candidate.declination_deg = declination_deg;
    *imu = candidate;
  }
  return status;
}

Icm20948_Status Imu_Update(Imu *imu, uint32_t timestamp_ms)
{
  if (imu == NULL)
  {
    return ICM20948_ERROR_ARGUMENT;
  }

  Icm20948_Status status = Icm20948_ReadRaw(&imu->sensor, &imu->raw);
  if (status != ICM20948_OK)
  {
    return status;
  }
  status = Icm20948_Convert(&imu->sensor, &imu->raw, &imu->data);
  if (status != ICM20948_OK)
  {
    return status;
  }

  const float ax = imu->data.acceleration_mps2[0];
  const float ay = imu->data.acceleration_mps2[1];
  const float az = imu->data.acceleration_mps2[2];
  const float mx = imu->data.magnetic_field_ut[0];
  const float my = imu->data.magnetic_field_ut[1];
  const float mz = imu->data.magnetic_field_ut[2];

  const float measured_roll = atan2f(ay, az) * RAD_TO_DEG;
  const float measured_pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * RAD_TO_DEG;
  const float roll_rad = measured_roll / RAD_TO_DEG;
  const float pitch_rad = measured_pitch / RAD_TO_DEG;
  const float horizontal_x = mx * cosf(pitch_rad) + mz * sinf(pitch_rad);
  const float horizontal_y = mx * sinf(roll_rad) * sinf(pitch_rad) +
                             my * cosf(roll_rad) -
                             mz * sinf(roll_rad) * cosf(pitch_rad);
  const float measured_heading =
      wrap_degrees(atan2f(-horizontal_y, horizontal_x) * RAD_TO_DEG +
                   imu->declination_deg);

  if (!imu->orientation_valid)
  {
    imu->orientation.roll_deg = measured_roll;
    imu->orientation.pitch_deg = measured_pitch;
    imu->orientation.yaw_deg = measured_heading;
    imu->orientation.heading_deg = measured_heading;
    imu->orientation_valid = true;
  }
  else
  {
    float dt = (float)(timestamp_ms - imu->previous_update_ms) * 0.001F;
    if (dt > MAX_UPDATE_INTERVAL_S)
    {
      dt = MAX_UPDATE_INTERVAL_S;
    }

    const float correction = dt / (ORIENTATION_TIME_CONSTANT + dt);
    const float predicted_roll = imu->orientation.roll_deg +
                                 imu->data.angular_rate_rps[0] * dt * RAD_TO_DEG;
    const float predicted_pitch = imu->orientation.pitch_deg +
                                  imu->data.angular_rate_rps[1] * dt * RAD_TO_DEG;
    const float predicted_yaw = imu->orientation.yaw_deg +
                                imu->data.angular_rate_rps[2] * dt * RAD_TO_DEG;

    imu->orientation.roll_deg =
        blend_angle(predicted_roll, measured_roll, correction);
    imu->orientation.pitch_deg =
        blend_angle(predicted_pitch, measured_pitch, correction);
    imu->orientation.yaw_deg =
        wrap_degrees(blend_angle(predicted_yaw, measured_heading, correction));
    imu->orientation.heading_deg = measured_heading;
  }

  imu->previous_update_ms = timestamp_ms;
  imu->data_valid = true;
  return ICM20948_OK;
}

const Icm20948_RawData *Imu_GetRaw(const Imu *imu)
{
  return ((imu != NULL) && imu->data_valid) ? &imu->raw : NULL;
}

const Icm20948_Data *Imu_GetData(const Imu *imu)
{
  return ((imu != NULL) && imu->data_valid) ? &imu->data : NULL;
}

const Imu_Orientation *Imu_GetOrientation(const Imu *imu)
{
  return ((imu != NULL) && imu->orientation_valid) ? &imu->orientation : NULL;
}

void Imu_SetCalibration(Imu *imu, const Icm20948_Calibration *calibration)
{
  if (imu != NULL)
  {
    Icm20948_SetCalibration(&imu->sensor, calibration);
  }
}

void Imu_SetMagneticDeclination(Imu *imu, float declination_deg)
{
  if (imu != NULL)
  {
    imu->declination_deg = declination_deg;
  }
}

void Imu_Invalidate(Imu *imu)
{
  if (imu != NULL)
  {
    imu->data_valid = false;
    imu->orientation_valid = false;
  }
}
