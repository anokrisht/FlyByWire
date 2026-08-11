#include "icm20948.h"

#include <string.h>

#define ICM20948_WHO_AM_I             0x00U
#define ICM20948_EXPECTED_ID          0xEAU
#define ICM20948_REG_BANK_SEL         0x7FU
#define ICM20948_PWR_MGMT_1           0x06U
#define ICM20948_PWR_MGMT_2           0x07U
#define ICM20948_INT_PIN_CFG          0x0FU
#define ICM20948_ACCEL_XOUT_H         0x2DU
#define ICM20948_GYRO_CONFIG_1        0x01U
#define ICM20948_ACCEL_CONFIG         0x14U

#define AK09916_ADDRESS               0x0CU
#define AK09916_WHO_AM_I              0x01U
#define AK09916_EXPECTED_ID           0x09U
#define AK09916_STATUS_1              0x10U
#define AK09916_CONTROL_2             0x31U
#define AK09916_CONTROL_3             0x32U

#define GRAVITY_MPS2                  9.80665F
#define DEG_TO_RAD                    0.017453292519943295F
#define ACCEL_LSB_PER_G               16384.0F
#define GYRO_LSB_PER_DPS              131.0F
#define MAG_UT_PER_LSB                0.15F

static Icm20948_Status write_reg(Icm20948 *device, uint8_t address,
                                 uint8_t reg, uint8_t value)
{
  return (device->bus.write_registers(device->bus.context, address, reg,
                                      &value, 1U) == I2C_BUS_OK)
             ? ICM20948_OK
             : ICM20948_ERROR_BUS;
}

static Icm20948_Status read_regs(Icm20948 *device, uint8_t address,
                                 uint8_t reg, uint8_t *data, size_t length)
{
  return (device->bus.read_registers(device->bus.context, address, reg,
                                     data, length) == I2C_BUS_OK)
             ? ICM20948_OK
             : ICM20948_ERROR_BUS;
}

static Icm20948_Status select_bank(Icm20948 *device, uint8_t bank)
{
  if (device->selected_bank == bank)
  {
    return ICM20948_OK;
  }

  Icm20948_Status status = write_reg(device, device->address,
                                     ICM20948_REG_BANK_SEL, bank << 4);
  if (status == ICM20948_OK)
  {
    device->selected_bank = bank;
  }
  return status;
}

static int16_t big_endian_i16(const uint8_t *bytes)
{
  return (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

static int16_t little_endian_i16(const uint8_t *bytes)
{
  return (int16_t)(((uint16_t)bytes[1] << 8) | bytes[0]);
}

Icm20948_Status Icm20948_Init(Icm20948 *device, const I2cBus *bus,
                              uint8_t address)
{
  if ((device == NULL) || (bus == NULL) || (bus->read_registers == NULL) ||
      (bus->write_registers == NULL) || (bus->delay_ms == NULL) ||
      ((address != 0x68U) && (address != 0x69U)))
  {
    return ICM20948_ERROR_ARGUMENT;
  }

  memset(device, 0, sizeof(*device));
  device->bus = *bus;
  device->address = address;
  device->selected_bank = 0xFFU;
  device->calibration.magnetic_scale[0] = 1.0F;
  device->calibration.magnetic_scale[1] = 1.0F;
  device->calibration.magnetic_scale[2] = 1.0F;

  Icm20948_Status status = select_bank(device, 0U);
  uint8_t id = 0U;
  if (status == ICM20948_OK)
  {
    status = read_regs(device, address, ICM20948_WHO_AM_I, &id, 1U);
  }
  if ((status == ICM20948_OK) && (id != ICM20948_EXPECTED_ID))
  {
    return ICM20948_ERROR_ID;
  }
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = write_reg(device, address, ICM20948_PWR_MGMT_1, 0x80U);
  device->bus.delay_ms(100U);
  device->selected_bank = 0xFFU;
  if ((status != ICM20948_OK) || (select_bank(device, 0U) != ICM20948_OK) ||
      (write_reg(device, address, ICM20948_PWR_MGMT_1, 0x01U) != ICM20948_OK) ||
      (write_reg(device, address, ICM20948_PWR_MGMT_2, 0x00U) != ICM20948_OK))
  {
    return ICM20948_ERROR_BUS;
  }
  device->bus.delay_ms(10U);

  /* ±250 dps and ±2 g, with the low-pass filters enabled. */
  if ((select_bank(device, 2U) != ICM20948_OK) ||
      (write_reg(device, address, ICM20948_GYRO_CONFIG_1, 0x01U) != ICM20948_OK) ||
      (write_reg(device, address, ICM20948_ACCEL_CONFIG, 0x01U) != ICM20948_OK) ||
      (select_bank(device, 0U) != ICM20948_OK) ||
      (write_reg(device, address, ICM20948_INT_PIN_CFG, 0x02U) != ICM20948_OK))
  {
    return ICM20948_ERROR_BUS;
  }
  device->bus.delay_ms(10U);

  if ((read_regs(device, AK09916_ADDRESS, AK09916_WHO_AM_I, &id, 1U) != ICM20948_OK) ||
      (id != AK09916_EXPECTED_ID))
  {
    return ICM20948_ERROR_ID;
  }

  /* Reset the compass, then select continuous measurement mode 4 (100 Hz). */
  if (write_reg(device, AK09916_ADDRESS, AK09916_CONTROL_3, 0x01U) != ICM20948_OK)
  {
    return ICM20948_ERROR_BUS;
  }
  device->bus.delay_ms(10U);
  if (write_reg(device, AK09916_ADDRESS, AK09916_CONTROL_2, 0x08U) != ICM20948_OK)
  {
    return ICM20948_ERROR_BUS;
  }

  device->initialized = true;
  return ICM20948_OK;
}

Icm20948_Status Icm20948_ReadRaw(Icm20948 *device, Icm20948_RawData *data)
{
  uint8_t imu_bytes[14];
  uint8_t mag_bytes[9];

  if ((device == NULL) || (data == NULL) || !device->initialized)
  {
    return ICM20948_ERROR_ARGUMENT;
  }
  if ((select_bank(device, 0U) != ICM20948_OK) ||
      (read_regs(device, device->address, ICM20948_ACCEL_XOUT_H,
                 imu_bytes, sizeof(imu_bytes)) != ICM20948_OK) ||
      (read_regs(device, AK09916_ADDRESS, AK09916_STATUS_1,
                 mag_bytes, sizeof(mag_bytes)) != ICM20948_OK))
  {
    return ICM20948_ERROR_BUS;
  }
  if ((mag_bytes[0] & 0x01U) == 0U)
  {
    return ICM20948_NOT_READY;
  }
  if ((mag_bytes[8] & 0x08U) != 0U)
  {
    return ICM20948_MAGNETOMETER_OVERFLOW;
  }

  for (uint32_t axis = 0U; axis < 3U; ++axis)
  {
    data->acceleration[axis] = big_endian_i16(&imu_bytes[axis * 2U]);
    data->angular_rate[axis] = big_endian_i16(&imu_bytes[8U + axis * 2U]);
    data->magnetic_field[axis] = little_endian_i16(&mag_bytes[1U + axis * 2U]);
  }
  data->temperature = big_endian_i16(&imu_bytes[6]);
  return ICM20948_OK;
}

Icm20948_Status Icm20948_Convert(const Icm20948 *device,
                                 const Icm20948_RawData *raw,
                                 Icm20948_Data *data)
{
  if ((device == NULL) || (raw == NULL) || (data == NULL))
  {
    return ICM20948_ERROR_ARGUMENT;
  }

  for (uint32_t axis = 0U; axis < 3U; ++axis)
  {
    data->acceleration_mps2[axis] =
        ((float)raw->acceleration[axis] * GRAVITY_MPS2 / ACCEL_LSB_PER_G) -
        device->calibration.acceleration_bias_mps2[axis];
    data->angular_rate_rps[axis] =
        ((float)raw->angular_rate[axis] / GYRO_LSB_PER_DPS * DEG_TO_RAD) -
        device->calibration.angular_rate_bias_rps[axis];
    data->magnetic_field_ut[axis] =
        (((float)raw->magnetic_field[axis] * MAG_UT_PER_LSB) -
         device->calibration.magnetic_offset_ut[axis]) *
        device->calibration.magnetic_scale[axis];
  }
  data->temperature_c = ((float)raw->temperature / 333.87F) + 21.0F;
  return ICM20948_OK;
}

Icm20948_Status Icm20948_Read(Icm20948 *device, Icm20948_Data *data)
{
  Icm20948_RawData raw;
  Icm20948_Status status = Icm20948_ReadRaw(device, &raw);
  return (status == ICM20948_OK) ? Icm20948_Convert(device, &raw, data) : status;
}

void Icm20948_SetCalibration(Icm20948 *device,
                             const Icm20948_Calibration *calibration)
{
  if ((device != NULL) && (calibration != NULL))
  {
    device->calibration = *calibration;
  }
}
