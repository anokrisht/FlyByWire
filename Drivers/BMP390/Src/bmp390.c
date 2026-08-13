#include "bmp390.h"

#include <stddef.h>
#include <string.h>

#define BMP390_CHIP_ID_REG       0x00U
#define BMP388_CHIP_ID           0x50U
#define BMP390_CHIP_ID           0x60U
#define BMP390_STATUS_REG        0x03U
#define BMP390_DATA_REG          0x04U
#define BMP390_PWR_CTRL_REG      0x1BU
#define BMP390_OSR_REG           0x1CU
#define BMP390_ODR_REG           0x1DU
#define BMP390_CONFIG_REG        0x1FU
#define BMP390_CALIB_REG         0x31U
#define BMP390_CMD_REG           0x7EU
#define BMP390_SOFT_RESET        0xB6U
#define BMP390_STATUS_DATA_READY 0x60U

static Bmp390_Status read_regs(Bmp390 *device, uint8_t reg, uint8_t *data,
                               size_t length)
{
  return (device->bus.read_registers(device->bus.context, device->address,
                                     reg, data, length) == I2C_BUS_OK)
             ? BMP390_OK : BMP390_ERROR_BUS;
}

static Bmp390_Status write_reg(Bmp390 *device, uint8_t reg, uint8_t value)
{
  return (device->bus.write_registers(device->bus.context, device->address,
                                      reg, &value, 1U) == I2C_BUS_OK)
             ? BMP390_OK : BMP390_ERROR_BUS;
}

static uint16_t u16_le(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t i16_le(const uint8_t *data)
{
  return (int16_t)u16_le(data);
}

static uint32_t u24_le(const uint8_t *data)
{
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16);
}

static void decode_calibration(Bmp390_Calibration *cal, const uint8_t *raw)
{
  cal->t1 = (double)u16_le(&raw[0]) * 256.0;
  cal->t2 = (double)u16_le(&raw[2]) / 1073741824.0;
  cal->t3 = (double)(int8_t)raw[4] / 281474976710656.0;
  cal->p1 = ((double)i16_le(&raw[5]) - 16384.0) / 1048576.0;
  cal->p2 = ((double)i16_le(&raw[7]) - 16384.0) / 536870912.0;
  cal->p3 = (double)(int8_t)raw[9] / 4294967296.0;
  cal->p4 = (double)(int8_t)raw[10] / 137438953472.0;
  cal->p5 = (double)u16_le(&raw[11]) * 8.0;
  cal->p6 = (double)u16_le(&raw[13]) / 64.0;
  cal->p7 = (double)(int8_t)raw[15] / 256.0;
  cal->p8 = (double)(int8_t)raw[16] / 32768.0;
  cal->p9 = (double)i16_le(&raw[17]) / 281474976710656.0;
  cal->p10 = (double)(int8_t)raw[19] / 281474976710656.0;
  cal->p11 = (double)(int8_t)raw[20] / 36893488147419103232.0;
}

Bmp390_Status Bmp390_Init(Bmp390 *device, const I2cBus *bus, uint8_t address)
{
  if ((device == NULL) || (bus == NULL) || (bus->read_registers == NULL) ||
      (bus->write_registers == NULL) || (bus->delay_ms == NULL) ||
      ((address != 0x76U) && (address != 0x77U)))
  {
    return BMP390_ERROR_ARGUMENT;
  }

  memset(device, 0, sizeof(*device));
  device->bus = *bus;
  device->address = address;

  uint8_t id = 0U;
  if (read_regs(device, BMP390_CHIP_ID_REG, &id, 1U) != BMP390_OK)
  {
    return BMP390_ERROR_BUS;
  }
  device->chip_id = id;
  if ((id != BMP388_CHIP_ID) && (id != BMP390_CHIP_ID))
  {
    return BMP390_ERROR_ID;
  }
  if (write_reg(device, BMP390_CMD_REG, BMP390_SOFT_RESET) != BMP390_OK)
  {
    return BMP390_ERROR_BUS;
  }
  device->bus.delay_ms(10U);

  uint8_t raw_calibration[21];
  if (read_regs(device, BMP390_CALIB_REG, raw_calibration,
                sizeof(raw_calibration)) != BMP390_OK)
  {
    return BMP390_ERROR_BUS;
  }
  decode_calibration(&device->calibration, raw_calibration);

  /* Pressure x8, temperature x2, 25 Hz, IIR coefficient 3, normal mode. */
  if ((write_reg(device, BMP390_OSR_REG, 0x0BU) != BMP390_OK) ||
      (write_reg(device, BMP390_ODR_REG, 0x03U) != BMP390_OK) ||
      (write_reg(device, BMP390_CONFIG_REG, 0x04U) != BMP390_OK) ||
      (write_reg(device, BMP390_PWR_CTRL_REG, 0x33U) != BMP390_OK))
  {
    return BMP390_ERROR_BUS;
  }
  device->initialized = true;
  return BMP390_OK;
}

const char *Bmp390_StatusName(Bmp390_Status status)
{
  static const char *const names[] = {
      "OK", "INVALID ARGUMENT", "I2C BUS ERROR", "WRONG CHIP ID", "NOT READY"};
  return ((uint32_t)status < (sizeof(names) / sizeof(names[0])))
             ? names[status] : "UNKNOWN";
}

const char *Bmp390_ModelName(const Bmp390 *device)
{
  if (device == NULL)
  {
    return "BMP3xx";
  }
  if (device->chip_id == BMP388_CHIP_ID)
  {
    return "BMP388";
  }
  if (device->chip_id == BMP390_CHIP_ID)
  {
    return "BMP390";
  }
  return "BMP3xx";
}

Bmp390_Status Bmp390_Read(Bmp390 *device, Bmp390_Data *data)
{
  if ((device == NULL) || (data == NULL) || !device->initialized)
  {
    return BMP390_ERROR_ARGUMENT;
  }
  uint8_t status = 0U;
  if (read_regs(device, BMP390_STATUS_REG, &status, 1U) != BMP390_OK)
  {
    return BMP390_ERROR_BUS;
  }
  if ((status & BMP390_STATUS_DATA_READY) != BMP390_STATUS_DATA_READY)
  {
    return BMP390_NOT_READY;
  }

  uint8_t raw[6];
  if (read_regs(device, BMP390_DATA_REG, raw, sizeof(raw)) != BMP390_OK)
  {
    return BMP390_ERROR_BUS;
  }
  const double raw_pressure = (double)u24_le(&raw[0]);
  const double raw_temperature = (double)u24_le(&raw[3]);
  const Bmp390_Calibration *cal = &device->calibration;
  const double dt = raw_temperature - cal->t1;
  const double temperature = dt * cal->t2 + dt * dt * cal->t3;
  const double temperature2 = temperature * temperature;
  const double temperature3 = temperature2 * temperature;
  const double pressure2 = raw_pressure * raw_pressure;
  const double pressure3 = pressure2 * raw_pressure;
  const double offset = cal->p5 + cal->p6 * temperature +
                        cal->p7 * temperature2 + cal->p8 * temperature3;
  const double sensitivity = cal->p1 + cal->p2 * temperature +
                             cal->p3 * temperature2 + cal->p4 * temperature3;
  const double nonlinear = pressure2 * (cal->p9 + cal->p10 * temperature) +
                           pressure3 * cal->p11;
  data->temperature_c = (float)temperature;
  data->pressure_pa = (float)(offset + raw_pressure * sensitivity + nonlinear);
  return BMP390_OK;
}
