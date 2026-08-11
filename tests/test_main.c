#include "icm20948.h"
#include "imu.h"
#include "gps_service.h"
#include "sensor_health.h"
#include "byte_ring_buffer.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned tests_run;
static unsigned tests_failed;

#define CHECK(condition)                                                       \
  do                                                                           \
  {                                                                            \
    ++tests_run;                                                               \
    if (!(condition))                                                          \
    {                                                                          \
      ++tests_failed;                                                          \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);             \
    }                                                                          \
  } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                                \
  CHECK(fabs((double)(actual) - (double)(expected)) <= (double)(tolerance))

typedef struct
{
  const uint8_t *data;
  size_t length;
  size_t position;
} FakeStream;

static bool fake_read_byte(void *context, uint8_t *byte)
{
  FakeStream *stream = context;
  if (stream->position >= stream->length)
  {
    return false;
  }
  *byte = stream->data[stream->position++];
  return true;
}

static void set_stream(FakeStream *fake, const char *text)
{
  fake->data = (const uint8_t *)text;
  fake->length = strlen(text);
  fake->position = 0U;
}

static void make_nmea(char *sentence, size_t capacity, const char *payload)
{
  static const char hex[] = "0123456789ABCDEF";
  uint8_t checksum = 0U;
  for (const char *character = payload; *character != '\0'; ++character)
  {
    checksum ^= (uint8_t)*character;
  }
  const size_t length = strlen(payload);
  CHECK(capacity >= length + 7U);
  if (capacity < length + 7U)
  {
    return;
  }
  sentence[0] = '$';
  memcpy(&sentence[1], payload, length);
  sentence[length + 1U] = '*';
  sentence[length + 2U] = hex[checksum >> 4];
  sentence[length + 3U] = hex[checksum & 0x0FU];
  sentence[length + 4U] = '\r';
  sentence[length + 5U] = '\n';
  sentence[length + 6U] = '\0';
}

static void test_nmea_parsing(void)
{
  FakeStream fake = {0};
  const ByteStream stream = {.context = &fake, .read_byte = fake_read_byte};
  GpsService gps;
  char sentence[NMEA_MAX_SENTENCE_LENGTH];
  CHECK(GpsService_Init(&gps, &stream));

  make_nmea(sentence, sizeof(sentence),
            "GNGLL,1234.50000,N,01234.50000,E,120000.00,A,A");
  set_stream(&fake, sentence);
  CHECK(GpsService_Update(&gps));
  GpsCoordinates coordinates;
  CHECK(GpsService_GetCoordinates(&gps, &coordinates));
  CHECK_NEAR(coordinates.latitude_deg, 12.575, 0.0000001);
  CHECK_NEAR(coordinates.longitude_deg, 12.575, 0.0000001);
  GpsTime time;
  CHECK(GpsService_GetUtcTime(&gps, &time));
  CHECK((time.hours == 12U) && (time.minutes == 0U) && (time.seconds == 0U));

  make_nmea(sentence, sizeof(sentence),
            "GNGGA,120001.00,1234.50000,N,01234.50000,E,1,07,1.47,"
            "100.0,M,-20.5,M,2.5,0042");
  set_stream(&fake, sentence);
  CHECK(GpsService_Update(&gps));
  const GpsData *data = GpsService_GetData(&gps);
  CHECK(data->altitude_valid);
  CHECK_NEAR(data->altitude_m, 100.0, 0.01);
  CHECK(data->satellites == 7U);
  CHECK(data->fix_quality == 1U);
  CHECK_NEAR(data->horizontal_dilution, 1.47, 0.001);
  CHECK(data->geoid_separation_valid);
  CHECK_NEAR(data->geoid_separation_m, -20.5, 0.001);
  CHECK(data->differential_age_valid);
  CHECK_NEAR(data->differential_age_s, 2.5, 0.001);
  CHECK(data->differential_station_valid);
  CHECK(data->differential_station_id == 42U);

  make_nmea(sentence, sizeof(sentence),
            "GNRMC,230500.00,A,1234.50000,S,01234.50000,W,10.000,"
            "90.0,010124,,,A,V");
  set_stream(&fake, sentence);
  CHECK(GpsService_Update(&gps));
  data = GpsService_GetData(&gps);
  CHECK(data->speed_valid);
  CHECK_NEAR(data->speed_mps, 5.144444, 0.0001);
  CHECK_NEAR(data->speed_knots, 10.0, 0.001);
  CHECK_NEAR(data->speed_kph, 18.52, 0.001);
  CHECK_NEAR(data->course_deg, 90.0, 0.001);
  CHECK_NEAR(data->coordinates.latitude_deg, -12.575, 0.0000001);
  CHECK_NEAR(data->coordinates.longitude_deg, -12.575, 0.0000001);
  GpsDate date;
  CHECK(GpsService_GetUtcDate(&gps, &date));
  CHECK((date.day == 1U) && (date.month == 1U) && (date.year == 2024U));

  int8_t day_offset;
  CHECK(GpsService_GetLocalTime(&gps, 120, &time, &day_offset));
  CHECK((time.hours == 1U) && (time.minutes == 5U) && (day_offset == 1));

  make_nmea(sentence, sizeof(sentence),
            "GNVTG,90.0,T,85.0,M,10.0,N,18.52,K,A");
  set_stream(&fake, sentence);
  CHECK(GpsService_Update(&gps));
  data = GpsService_GetData(&gps);
  CHECK_NEAR(data->course_deg, 90.0, 0.001);
  CHECK_NEAR(data->magnetic_course_deg, 85.0, 0.001);
  CHECK_NEAR(data->speed_kph, 18.52, 0.001);
  CHECK(data->positioning_mode == 'A');

  make_nmea(sentence, sizeof(sentence),
            "GNGSA,A,3,01,02,03,,,,,,,,,,1.50,0.90,1.20,1");
  set_stream(&fake, sentence);
  CHECK(GpsService_Update(&gps));
  data = GpsService_GetData(&gps);
  CHECK(data->fix_dimension == GPS_FIX_3D);
  CHECK(data->selection_mode == 'A');
  CHECK(data->active_constellation == GPS_CONSTELLATION_GPS);
  CHECK(data->used_satellite_count == 3U);
  CHECK(data->used_satellites[0].id == 1U);
  CHECK(data->dilution_valid);
  CHECK_NEAR(data->position_dilution, 1.50, 0.001);
  CHECK_NEAR(data->horizontal_dilution, 0.90, 0.001);
  CHECK_NEAR(data->vertical_dilution, 1.20, 0.001);

  make_nmea(sentence, sizeof(sentence),
            "GPGSV,2,1,05,01,45,100,40,02,30,200,35,03,15,300,20,"
            "04,60,050,45,1");
  set_stream(&fake, sentence);
  CHECK(GpsService_Update(&gps));
  make_nmea(sentence, sizeof(sentence),
            "GPGSV,2,2,05,05,10,150,,1");
  set_stream(&fake, sentence);
  CHECK(GpsService_Update(&gps));
  make_nmea(sentence, sizeof(sentence),
            "GLGSV,1,1,02,71,25,020,15,72,50,180,30,1");
  set_stream(&fake, sentence);
  CHECK(GpsService_Update(&gps));
  data = GpsService_GetData(&gps);
  CHECK(data->visible_satellite_count == 7U);
  CHECK(data->satellites_in_view == 7U);
  CHECK(data->visible_satellites[0].constellation == GPS_CONSTELLATION_GPS);
  CHECK(data->visible_satellites[0].signal_strength_dbhz == 40U);
  CHECK(data->visible_satellites[4].id == 5U);
  CHECK(!data->visible_satellites[4].signal_strength_valid);
  CHECK(data->visible_satellites[5].constellation == GPS_CONSTELLATION_GLONASS);

  make_nmea(sentence, sizeof(sentence),
            "GNGLL,1234.50000,N,01234.50000,E,120000.00,A,A");
  sentence[strlen(sentence) - 4U] =
      (sentence[strlen(sentence) - 4U] == '0') ? '1' : '0';
  set_stream(&fake, sentence);
  CHECK(!GpsService_Update(&gps));
  GpsDiagnostics diagnostics;
  CHECK(GpsService_GetDiagnostics(&gps, &diagnostics));
  CHECK(diagnostics.checksum_error_count == 1U);
  GpsService_Invalidate(&gps);
  CHECK(!GpsService_GetCoordinates(&gps, &coordinates));
  CHECK(!GpsService_GetUtcTime(&gps, &time));
}

static void test_sensor_health(void)
{
  SensorHealth health;
  SensorHealth_Init(&health, 100U, 1000U, 500U, 3U);
  CHECK(health.state == SENSOR_HEALTH_STARTING);
  CHECK(!SensorHealth_ShouldRetry(&health, 1099U));
  CHECK(SensorHealth_ShouldRetry(&health, 1100U));

  SensorHealth_RecordSuccess(&health, 1200U);
  CHECK(health.state == SENSOR_HEALTH_OK);
  SensorHealth_RecordFailure(&health, 1300U);
  CHECK(health.state == SENSOR_HEALTH_DEGRADED);
  SensorHealth_RecordFailure(&health, 1400U);
  SensorHealth_RecordFailure(&health, 1500U);
  CHECK(health.state == SENSOR_HEALTH_OFFLINE);
  CHECK(!SensorHealth_ShouldRetry(&health, 1999U));
  CHECK(SensorHealth_ShouldRetry(&health, 2000U));

  SensorHealth_RecordSuccess(&health, 3000U);
  SensorHealth_Update(&health, 3999U);
  CHECK(health.state == SENSOR_HEALTH_OK);
  SensorHealth_Update(&health, 4000U);
  CHECK(health.state == SENSOR_HEALTH_STALE);

  SensorHealth_Init(&health, UINT32_MAX - 100U, 200U, 50U, 1U);
  CHECK(SensorHealth_ShouldRetry(&health, 99U));
}

static void test_ring_buffer(void)
{
  ByteRingBuffer ring;
  uint8_t storage[4];
  uint8_t byte;
  CHECK(!ByteRingBuffer_Init(NULL, storage, 4U));
  CHECK(!ByteRingBuffer_Init(&ring, storage, 1U));
  CHECK(ByteRingBuffer_Init(&ring, storage, 4U));
  CHECK(ByteRingBuffer_Push(&ring, 10U));
  CHECK(ByteRingBuffer_Push(&ring, 20U));
  CHECK(ByteRingBuffer_Push(&ring, 30U));
  CHECK(!ByteRingBuffer_Push(&ring, 40U));
  CHECK(ring.overflow_count == 1U);
  CHECK(ByteRingBuffer_Pop(&ring, &byte) && (byte == 10U));
  CHECK(ByteRingBuffer_Push(&ring, 40U));
  CHECK(ByteRingBuffer_Pop(&ring, &byte) && (byte == 20U));
  CHECK(ByteRingBuffer_Pop(&ring, &byte) && (byte == 30U));
  CHECK(ByteRingBuffer_Pop(&ring, &byte) && (byte == 40U));
  CHECK(!ByteRingBuffer_Pop(&ring, &byte));
  ByteRingBuffer_Clear(&ring);
  CHECK(!ByteRingBuffer_Pop(&ring, &byte));
}

typedef struct
{
  bool powered;
  uint8_t bank;
  unsigned delay_total_ms;
} FakeI2c;

static I2cBus_Status fake_i2c_read(void *context, uint8_t address,
                                   uint8_t reg, uint8_t *data, size_t length)
{
  FakeI2c *fake = context;
  if (!fake->powered)
  {
    return I2C_BUS_ERROR;
  }
  memset(data, 0, length);
  if ((address == 0x68U) && (fake->bank == 0U) && (reg == 0x00U))
  {
    data[0] = 0xEAU;
  }
  else if ((address == 0x0CU) && (reg == 0x01U))
  {
    data[0] = 0x09U;
  }
  else if ((address == 0x68U) && (reg == 0x2DU) && (length == 14U))
  {
    data[4] = 0x40U; /* acceleration Z = +16384 = 1 g */
    data[8] = 0x00U;
    data[9] = 0x83U; /* gyro X = 131 = 1 dps */
  }
  else if ((address == 0x0CU) && (reg == 0x10U) && (length == 9U))
  {
    data[0] = 0x01U;
    data[1] = 100U; /* magnetic X = 100, little endian */
  }
  return I2C_BUS_OK;
}

static I2cBus_Status fake_i2c_write(void *context, uint8_t address,
                                    uint8_t reg, const uint8_t *data,
                                    size_t length)
{
  FakeI2c *fake = context;
  if (!fake->powered || (length != 1U))
  {
    return I2C_BUS_ERROR;
  }
  if ((address == 0x68U) && (reg == 0x7FU))
  {
    fake->bank = (uint8_t)(data[0] >> 4);
  }
  return I2C_BUS_OK;
}

static FakeI2c *delay_context;

static void fake_delay(uint32_t delay_ms)
{
  delay_context->delay_total_ms += delay_ms;
}

static void test_icm20948(void)
{
  FakeI2c fake = {.powered = true};
  delay_context = &fake;
  const I2cBus bus = {
      .context = &fake,
      .read_registers = fake_i2c_read,
      .write_registers = fake_i2c_write,
      .delay_ms = fake_delay,
  };
  Icm20948 device;
  CHECK(Icm20948_Init(&device, &bus, 0x68U) == ICM20948_OK);
  CHECK(device.initialized);
  CHECK(fake.delay_total_ms == 130U);

  Icm20948_RawData raw;
  CHECK(Icm20948_ReadRaw(&device, &raw) == ICM20948_OK);
  CHECK(raw.acceleration[2] == 16384);
  CHECK(raw.angular_rate[0] == 131);
  CHECK(raw.magnetic_field[0] == 100);

  Icm20948_Data data;
  CHECK(Icm20948_Convert(&device, &raw, &data) == ICM20948_OK);
  CHECK_NEAR(data.acceleration_mps2[2], 9.80665, 0.0001);
  CHECK_NEAR(data.angular_rate_rps[0], 0.0174533, 0.000001);
  CHECK_NEAR(data.magnetic_field_ut[0], 15.0, 0.001);

  Icm20948_Calibration calibration = {0};
  calibration.acceleration_bias_mps2[2] = 0.1F;
  calibration.magnetic_offset_ut[0] = 5.0F;
  calibration.magnetic_scale[0] = 2.0F;
  Icm20948_SetCalibration(&device, &calibration);
  CHECK(Icm20948_Convert(&device, &raw, &data) == ICM20948_OK);
  CHECK_NEAR(data.acceleration_mps2[2], 9.70665, 0.0001);
  CHECK_NEAR(data.magnetic_field_ut[0], 20.0, 0.001);

  fake.powered = false;
  CHECK(Icm20948_ReadRaw(&device, &raw) == ICM20948_ERROR_BUS);
  Icm20948 offline;
  CHECK(Icm20948_Init(&offline, &bus, 0x68U) == ICM20948_ERROR_BUS);
  fake.powered = true;
  CHECK(Icm20948_Init(&offline, &bus, 0x68U) == ICM20948_OK);
}

static void test_imu_orientation(void)
{
  FakeI2c fake = {.powered = true};
  delay_context = &fake;
  const I2cBus bus = {
      .context = &fake,
      .read_registers = fake_i2c_read,
      .write_registers = fake_i2c_write,
      .delay_ms = fake_delay,
  };
  Imu imu;
  CHECK(Imu_Init(&imu, &bus, 0x68U) == ICM20948_OK);
  CHECK(Imu_Update(&imu, 10U) == ICM20948_OK);
  const Imu_Orientation *orientation = Imu_GetOrientation(&imu);
  CHECK(orientation != NULL);
  CHECK_NEAR(orientation->roll_deg, 0.0, 0.01);
  CHECK_NEAR(orientation->pitch_deg, 0.0, 0.01);
  CHECK_NEAR(orientation->heading_deg, 0.0, 0.01);
  Imu_Invalidate(&imu);
  CHECK(Imu_GetRaw(&imu) == NULL);
  CHECK(Imu_GetData(&imu) == NULL);
  CHECK(Imu_GetOrientation(&imu) == NULL);

  Icm20948_Calibration calibration = {0};
  calibration.magnetic_scale[0] = 1.1F;
  calibration.magnetic_scale[1] = 1.2F;
  calibration.magnetic_scale[2] = 1.3F;
  calibration.angular_rate_bias_rps[0] = 0.02F;
  Imu_SetCalibration(&imu, &calibration);
  Imu_SetMagneticDeclination(&imu, 4.5F);
  fake.powered = false;
  CHECK(Imu_Reinitialize(&imu) == ICM20948_ERROR_BUS);
  fake.powered = true;
  CHECK(Imu_Reinitialize(&imu) == ICM20948_OK);
  CHECK_NEAR(imu.sensor.calibration.magnetic_scale[2], 1.3, 0.0001);
  CHECK_NEAR(imu.sensor.calibration.angular_rate_bias_rps[0], 0.02, 0.0001);
  CHECK_NEAR(imu.declination_deg, 4.5, 0.0001);
}

int main(void)
{
  test_nmea_parsing();
  test_sensor_health();
  test_ring_buffer();
  test_icm20948();
  test_imu_orientation();
  printf("%u checks, %u failures\n", tests_run, tests_failed);
  return (tests_failed == 0U) ? 0 : 1;
}
