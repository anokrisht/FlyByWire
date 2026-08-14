#include "application.h"

#include "airspeed.h"
#include "barometer.h"
#include "imu.h"
#include "main.h"
#include "gps_service.h"
#include "sensor_supervision.h"
#include "stm32_i2c_bus.h"
#include "stm32_uart_stream.h"
#include "uart_console.h"

#include <stdio.h>
#include <string.h>

#define ICM20948_I2C_ADDRESS 0x68U
#define BMP390_I2C_ADDRESS   0x76U
#define IMU_SAMPLE_PERIOD_MS  10U
#define BAROMETER_SAMPLE_PERIOD_MS 40U
#define AIRSPEED_SAMPLE_PERIOD_MS  20U
#define TELEMETRY_PERIOD_MS   100U

static Imu imu;
static Barometer barometer;
static Airspeed airspeed;
static GpsService gps;
static I2cBus imu_i2c_bus;
static Stm32UartStream gps_uart_stream;
static SensorSupervision sensor_supervision;
static uint32_t last_sample_ms;
static uint32_t last_barometer_sample_ms;
static uint32_t last_airspeed_sample_ms;
static uint32_t last_telemetry_ms;

static Bmp390_Status initialize_barometer(void)
{
  const uint8_t addresses[] = {BMP390_I2C_ADDRESS,
                               (BMP390_I2C_ADDRESS == 0x76U) ? 0x77U : 0x76U};
  Bmp390_Status status = BMP390_ERROR_BUS;
  for (uint32_t i = 0U; i < (sizeof(addresses) / sizeof(addresses[0])); ++i)
  {
    status = Barometer_Init(&barometer, &imu_i2c_bus, addresses[i]);
    if (status == BMP390_OK)
    {
      printf("INFO: %s found at I2C address 0x%02X\r\n",
             Bmp390_ModelName(&barometer.sensor), addresses[i]);
      return status;
    }
    printf("WARN: BMP390 probe 0x%02X: %s (chip ID 0x%02X)\r\n",
           addresses[i], Bmp390_StatusName(status), barometer.sensor.chip_id);
  }
  return status;
}

static const char *gps_fix_name(const GpsData *data, bool fix_valid)
{
  if (!fix_valid)
  {
    return "NO FIX";
  }
  if (data->fix_dimension == GPS_FIX_3D)
  {
    return "3D FIX";
  }
  if (data->fix_dimension == GPS_FIX_2D)
  {
    return "2D FIX";
  }
  return "FIX";
}

static void print_gps_fix(void)
{
  GpsCoordinates coordinates = {0};
  GpsTime utc = {0};
  const GpsData *data = GpsService_GetData(&gps);

  if (data != NULL)
  {
    const bool fix_valid =
        GpsService_GetCoordinates(&gps, &coordinates);
    (void)GpsService_GetUtcTime(&gps, &utc);

    printf("GPS %s: UTC %02u:%02u:%02u | %.7f, %.7f | "
           "ALT %.1f m | SPEED %.2f m/s | COURSE %.1f deg | "
           "SAT %u/%u | DOP %.2f/%.2f/%.2f\r\n",
           gps_fix_name(data, fix_valid),
           utc.hours, utc.minutes, utc.seconds,
           coordinates.latitude_deg, coordinates.longitude_deg,
           (fix_valid && data->altitude_valid) ? data->altitude_m : 0.0F,
           (fix_valid && data->speed_valid) ? data->speed_mps : 0.0F,
           (fix_valid && data->course_valid) ? data->course_deg : 0.0F,
           data->satellites, data->satellites_in_view,
           data->dilution_valid ? data->position_dilution : 0.0F,
           data->dilution_valid ? data->horizontal_dilution : 0.0F,
           data->dilution_valid ? data->vertical_dilution : 0.0F);
  }
}

static void print_imu(void)
{
  const Icm20948_Data *data = Imu_GetData(&imu);
  const Imu_Orientation *orientation = Imu_GetOrientation(&imu);
  if ((data == NULL) || (orientation == NULL))
  {
    return;
  }

  printf("A[m/s2] %.2f %.2f %.2f | RPY[deg] %.1f %.1f %.1f | H %.1f\r\n",
         data->acceleration_mps2[0], data->acceleration_mps2[1],
         data->acceleration_mps2[2], orientation->roll_deg,
         orientation->pitch_deg, orientation->yaw_deg,
         orientation->heading_deg);
}

static void print_altitude(void)
{
  const Bmp390_Data *data = Barometer_GetData(&barometer);
  if (data == NULL)
  {
    return;
  }

  printf("BARO %.2f hPa | %.2f C | ALT %.1f m\r\n",
         data->pressure_pa * 0.01F, data->temperature_c,
         Barometer_GetAltitude(&barometer));
}

static void print_airspeed(void)
{
  if (!airspeed.data_valid)
  {
    return;
  }

  printf("AIR %.2f m/s | DP %.1f Pa | ADC %u\r\n",
         airspeed.indicated_airspeed_mps, airspeed.differential_pressure_pa,
         airspeed.raw_adc);
}

void Application_Init(I2C_HandleTypeDef *i2c, ADC_HandleTypeDef *adc,
                      UART_HandleTypeDef *console_uart,
                      UART_HandleTypeDef *gps_uart)
{
  UartConsole_Init(console_uart);
  const uint32_t now = HAL_GetTick();
  imu_i2c_bus = Stm32I2cBus_Create(i2c);

  const bool gps_uart_started =
      Stm32UartStream_Start(&gps_uart_stream, gps_uart);
  const ByteStream gps_stream =
      Stm32UartStream_AsByteStream(&gps_uart_stream);
  if (!GpsService_Init(&gps, &gps_stream))
  {
    UartConsole_WriteLine("FATAL: invalid GPS driver configuration");
    Error_Handler();
  }
  if (!gps_uart_started)
  {
    UartConsole_WriteLine("WARN: GPS UART offline; recovery scheduled");
  }

  const bool imu_online =
      Imu_Init(&imu, &imu_i2c_bus, ICM20948_I2C_ADDRESS) == ICM20948_OK;
  if (!imu_online)
  {
    UartConsole_WriteLine("WARN: ICM-20948 offline; recovery scheduled");
  }

  const bool barometer_online = initialize_barometer() == BMP390_OK;
  if (!barometer_online)
  {
    UartConsole_WriteLine("WARN: BMP390 offline; recovery scheduled");
  }

  SensorSupervision_Init(&sensor_supervision, &imu, &barometer, &gps,
                         &gps_uart_stream, now, imu_online, barometer_online,
                         gps_uart_started);
  if (Airspeed_Init(&airspeed, adc) == AIRSPEED_OK)
  {
    printf("INFO: airspeed zero calibrated at ADC %.1f\r\n",
           airspeed.zero_adc_counts);
  }
  else
  {
    UartConsole_WriteLine("WARN: airspeed ADC unavailable");
  }
  UartConsole_WriteLine("FlyByWire: sensor supervision active");
}

void Application_Run(void)
{
  uint32_t now = HAL_GetTick();
  while (GpsService_Update(&gps))
  {
    SensorSupervision_Report(&sensor_supervision, SUPERVISED_GPS,
                             SENSOR_SAMPLE_SUCCESS, now);
    const char *sentence = GpsService_GetRawSentence(&gps);
    if ((sentence != NULL) && (strlen(sentence) >= 6U) &&
        (strncmp(&sentence[3], "RMC", 3U) == 0))
    {
      //print_gps_fix();
    }
  }

  SensorSupervision_Check(&sensor_supervision, now);

  if (SensorSupervision_IsOperational(&sensor_supervision,
                                      SUPERVISED_BAROMETER) &&
      ((uint32_t)(now - last_barometer_sample_ms) >= BAROMETER_SAMPLE_PERIOD_MS))
  {
    last_barometer_sample_ms = now;
    const Bmp390_Status barometer_status = Barometer_Update(&barometer);
    if (barometer_status == BMP390_OK)
    {
      SensorSupervision_Report(&sensor_supervision, SUPERVISED_BAROMETER,
                               SENSOR_SAMPLE_SUCCESS, now);
    }
    else if (barometer_status != BMP390_NOT_READY)
    {
      SensorSupervision_Report(&sensor_supervision, SUPERVISED_BAROMETER,
                               SENSOR_SAMPLE_FAILURE, now);
    }
  }

  if ((uint32_t)(now - last_airspeed_sample_ms) >= AIRSPEED_SAMPLE_PERIOD_MS)
  {
    last_airspeed_sample_ms = now;
    float air_density = 1.225F;
    const Bmp390_Data *density_data = Barometer_GetData(&barometer);
    if ((density_data != NULL) && (density_data->pressure_pa > 0.0F))
    {
      air_density = density_data->pressure_pa /
          (287.05F * (density_data->temperature_c + 273.15F));
    }
    (void)Airspeed_Update(&airspeed, air_density);
  }

  if (!SensorSupervision_IsOperational(&sensor_supervision, SUPERVISED_IMU))
  {
    goto telemetry;
  }

  if ((uint32_t)(now - last_sample_ms) < IMU_SAMPLE_PERIOD_MS)
  {
    goto telemetry;
  }
  last_sample_ms = now;

  const Icm20948_Status status = Imu_Update(&imu, now);
  if (status == ICM20948_OK)
  {
    SensorSupervision_Report(&sensor_supervision, SUPERVISED_IMU,
                             SENSOR_SAMPLE_SUCCESS, now);
  }
  else if (status != ICM20948_NOT_READY)
  {
    SensorSupervision_Report(&sensor_supervision, SUPERVISED_IMU,
                             SENSOR_SAMPLE_FAILURE, now);
    return;
  }
  else
  {
    goto telemetry;
  }

telemetry:
  if ((uint32_t)(now - last_telemetry_ms) >= TELEMETRY_PERIOD_MS)
  {
    last_telemetry_ms = now;
    //print_imu();
    //print_altitude();
    print_airspeed();
  }
}

const SensorHealth *Application_GetImuHealth(void)
{
  return SensorSupervision_GetHealth(&sensor_supervision, SUPERVISED_IMU);
}

const SensorHealth *Application_GetGpsHealth(void)
{
  return SensorSupervision_GetHealth(&sensor_supervision, SUPERVISED_GPS);
}

const SensorHealth *Application_GetBarometerHealth(void)
{
  return SensorSupervision_GetHealth(&sensor_supervision,
                                     SUPERVISED_BAROMETER);
}
