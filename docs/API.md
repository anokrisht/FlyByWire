# FlyByWire API and usage guide

This document describes the public interfaces grouped under `Application`,
`Drivers`, `Services`, `Platform`, and `Utilities`. Include public headers from
application code; do not call private `static` functions from implementation
files.

## Architecture

```text
main
  -> application
      -> IMU service -> ICM-20948 driver -> generic I2C bus -> STM32 HAL
      -> NMEA driver -> generic byte stream -> STM32 UART HAL
      -> UART console -> STM32 UART HAL
      -> sensor-health supervisor
```

`Core/Src/main.c` owns generated peripheral initialization.
`Application/Src/application.c` coordinates the modules. Sensor algorithms and
parsers do not depend directly on STM32 HAL.

## Application lifecycle

Header: `application.h`

### `Application_Init`

```c
void Application_Init(I2C_HandleTypeDef *i2c,
                      UART_HandleTypeDef *console_uart,
                      UART_HandleTypeDef *gps_uart);
```

Attaches I2C1, the USART2 console, and the USART1 GPS receiver. It initializes
sensor supervision and schedules recovery when hardware is unavailable. Call it
once after CubeMX has initialized all three peripherals.

```c
MX_I2C1_Init();
MX_USART2_UART_Init();
MX_USART1_UART_Init();
Application_Init(&hi2c1, &huart2, &huart1);
```

### `Application_Run`

```c
void Application_Run(void);
```

Runs one non-blocking application iteration. Call it continuously from the main
loop. It consumes GPS input, samples the IMU, updates health states, and performs
scheduled recovery.

```c
while (1)
{
  Application_Run();
}
```

### Application health getters

```c
const SensorHealth *Application_GetImuHealth(void);
const SensorHealth *Application_GetGpsHealth(void);
```

Return read-only health information. Never modify the returned objects.

```c
const SensorHealth *health = Application_GetGpsHealth();
printf("GPS state: %s, failures: %lu\r\n",
       SensorHealth_StateName(health->state),
       (unsigned long)health->total_failures);
```

## IMU service

Header: `imu.h`

The IMU service combines the ICM-20948 driver with calibration, unit conversion,
and filtered roll, pitch, yaw, and magnetic heading.

### Initialization and sampling

```c
Icm20948_Status Imu_Init(Imu *imu, const I2cBus *bus, uint8_t address);
Icm20948_Status Imu_Reinitialize(Imu *imu);
Icm20948_Status Imu_Update(Imu *imu, uint32_t timestamp_ms);
```

- `Imu_Init` initializes a new object at address `0x68` or `0x69`.
- `Imu_Reinitialize` restores a reset sensor while retaining calibration and
  magnetic declination.
- `Imu_Update` samples and processes the sensor. Supply a monotonic millisecond
  timestamp such as `HAL_GetTick()`.

```c
I2cBus bus = Stm32I2cBus_Create(&hi2c1);
Imu imu;

if (Imu_Init(&imu, &bus, 0x68U) == ICM20948_OK)
{
  (void)Imu_Update(&imu, HAL_GetTick());
}
```

### Reading IMU results

```c
const Icm20948_RawData *Imu_GetRaw(const Imu *imu);
const Icm20948_Data *Imu_GetData(const Imu *imu);
const Imu_Orientation *Imu_GetOrientation(const Imu *imu);
```

These return `NULL` until a successful update and after data is invalidated.

```c
const Icm20948_Data *data = Imu_GetData(&imu);
const Imu_Orientation *orientation = Imu_GetOrientation(&imu);

if ((data != NULL) && (orientation != NULL))
{
  float ax = data->acceleration_mps2[0];
  float yaw = orientation->yaw_deg;
  float heading = orientation->heading_deg;
}
```

Available converted values:

- `acceleration_mps2[3]`
- `angular_rate_rps[3]`
- `magnetic_field_ut[3]`
- `temperature_c`
- `roll_deg`, `pitch_deg`, `yaw_deg`, and `heading_deg`

### Calibration and invalidation

```c
void Imu_SetCalibration(Imu *imu,
                        const Icm20948_Calibration *calibration);
void Imu_SetMagneticDeclination(Imu *imu, float declination_deg);
void Imu_Invalidate(Imu *imu);
```

```c
Icm20948_Calibration calibration = {
    .acceleration_bias_mps2 = {0.01F, -0.02F, 0.04F},
    .angular_rate_bias_rps = {0.001F, 0.0F, -0.001F},
    .magnetic_offset_ut = {12.0F, -8.0F, 3.0F},
    .magnetic_scale = {1.02F, 0.98F, 1.00F},
};

Imu_SetCalibration(&imu, &calibration);
Imu_SetMagneticDeclination(&imu, 2.5F);
```

Declination is location-dependent. Do not copy the example value into a real
installation without determining the correct value.

## ICM-20948 register driver

Header: `icm20948.h`

Use this lower-level API when orientation processing is not required.

```c
Icm20948_Status Icm20948_Init(Icm20948 *device, const I2cBus *bus,
                              uint8_t address);
Icm20948_Status Icm20948_ReadRaw(Icm20948 *device,
                                 Icm20948_RawData *data);
Icm20948_Status Icm20948_Convert(const Icm20948 *device,
                                 const Icm20948_RawData *raw,
                                 Icm20948_Data *data);
Icm20948_Status Icm20948_Read(Icm20948 *device, Icm20948_Data *data);
void Icm20948_SetCalibration(Icm20948 *device,
                             const Icm20948_Calibration *calibration);
```

Always inspect the returned status. `ICM20948_NOT_READY` means the
magnetometer has not produced a new sample and is not necessarily a fault.

## NMEA protocol driver

Header: `nmea_parser.h`

The driver owns only transport-facing behavior: byte framing, maximum sentence
length, checksum validation, and raw sentence counters. It has no knowledge of
coordinates or navigation policy.

```c
bool NmeaParser_Init(NmeaParser *parser, const ByteStream *stream);
bool NmeaParser_Poll(NmeaParser *parser);
const char *NmeaParser_GetSentence(const NmeaParser *parser);
```

Most application code should use `GpsService` instead of calling this driver
directly.

## GPS service

Header: `gps_service.h`

The service owns structured navigation state. Its current backend is the NMEA
driver, and it extracts values from GLL, GGA, and RMC sentences.

### Initialization and updates

```c
bool GpsService_Init(GpsService *service, const ByteStream *stream);
bool GpsService_Update(GpsService *service);
```

Call `GpsService_Update` repeatedly until it returns `false` to drain buffered
sentences:

```c
while (GpsService_Update(&gps))
{
  const char *raw_sentence = GpsService_GetRawSentence(&gps);
  if (raw_sentence != NULL)
  {
    UartConsole_WriteLine(raw_sentence);
  }
}
```

### Raw and structured GPS data

```c
const char *GpsService_GetRawSentence(const GpsService *service);
const GpsData *GpsService_GetData(const GpsService *service);
bool GpsService_GetDiagnostics(const GpsService *service,
                               GpsDiagnostics *diagnostics);
bool GpsService_GetCoordinates(const GpsService *service,
                               GpsCoordinates *coordinates);
bool GpsService_GetUtcTime(const GpsService *service, GpsTime *time);
bool GpsService_GetUtcDate(const GpsService *service, GpsDate *date);
```

Prefer the checked getters for individual values:

```c
GpsCoordinates coordinates;
if (GpsService_GetCoordinates(&gps, &coordinates))
{
  printf("%.7f, %.7f\r\n",
         coordinates.latitude_deg,
         coordinates.longitude_deg);
}
```

`GpsService_GetData` also provides:

- `altitude_m` with `altitude_valid`
- `speed_mps`, `speed_knots`, and `speed_kph` with `speed_valid`
- true and magnetic course with independent validity flags
- magnetic variation from RMC
- `position_dilution`, `horizontal_dilution`, and `vertical_dilution`
- geoid separation and differential-correction information from GGA
- satellites used and satellites in view
- `fix_quality` and 2D/3D `fix_dimension`
- automatic/manual selection mode and receiver positioning/navigation status
- a used-satellite list including constellation identity
- a visible-satellite list containing ID, constellation, elevation, azimuth,
  signal strength, and per-field validity

Supported standard sentences:

- GLL: position, UTC, validity, positioning mode
- GGA: fix quality, satellites used, HDOP, altitude, geoid and differential data
- RMC: position, UTC date/time, speed, course, magnetic variation and status
- VTG: true/magnetic course, speed and positioning mode
- GSA: 2D/3D fix, used satellites, constellation and PDOP/HDOP/VDOP
- GSV: multipart per-satellite visibility and signal information

Unknown and proprietary sentences are still checksum-validated and available
through `GpsService_GetRawSentence`, even when no structured decoder exists.

### Local time conversion

```c
bool GpsService_GetLocalTime(const GpsService *service,
                             int16_t utc_offset_minutes,
                             GpsTime *time,
                             int8_t *day_offset);
```

GPS supplies UTC, not a named civil time zone. Pass the applicable fixed offset:

```c
GpsTime local;
int8_t day_offset;

if (GpsService_GetLocalTime(&gps, 120, &local, &day_offset))
{
  printf("Local time: %02u:%02u:%02u\r\n",
         local.hours, local.minutes, local.seconds);
}
```

`day_offset` is `-1`, `0`, or `1` when conversion crosses midnight. Daylight
saving rules require an external time-zone policy.

### Invalidating cached GPS values

```c
void GpsService_Invalidate(GpsService *service);
```

Call this after a transport timeout so old coordinates are not treated as a
current fix. The application supervisor already does this automatically.

## Sensor health supervisor

Header: `sensor_health.h`

States:

- `SENSOR_HEALTH_STARTING`
- `SENSOR_HEALTH_OK`
- `SENSOR_HEALTH_DEGRADED`
- `SENSOR_HEALTH_STALE`
- `SENSOR_HEALTH_OFFLINE`

Public operations:

```c
void SensorHealth_Init(SensorHealth *health, uint32_t now_ms,
                       uint32_t stale_timeout_ms,
                       uint32_t retry_interval_ms,
                       uint16_t failure_threshold);
void SensorHealth_RecordSuccess(SensorHealth *health, uint32_t now_ms);
void SensorHealth_RecordFailure(SensorHealth *health, uint32_t now_ms);
void SensorHealth_MarkOffline(SensorHealth *health, uint32_t now_ms);
void SensorHealth_Update(SensorHealth *health, uint32_t now_ms);
bool SensorHealth_ShouldRetry(const SensorHealth *health, uint32_t now_ms);
const char *SensorHealth_StateName(SensorHealth_State state);
```

Use unsigned millisecond timestamps. Deadline calculations are safe across the
normal 32-bit tick wraparound.

## UART console

Header: `uart_console.h`

```c
void UartConsole_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef UartConsole_Write(const uint8_t *data, size_t length);
HAL_StatusTypeDef UartConsole_WriteString(const char *text);
HAL_StatusTypeDef UartConsole_WriteLine(const char *text);
HAL_StatusTypeDef UartConsole_ReadByte(uint8_t *byte, uint32_t timeout_ms);
```

`UartConsole_Init` also enables the project's `printf` and standard-input
retargeting through `__io_putchar` and `__io_getchar`.

```c
UartConsole_Init(&huart2);
UartConsole_WriteLine("System ready");
printf("Temperature: %.2f C\r\n", temperature);
```

Console operations are blocking and should not be used from interrupt handlers.

## Hardware adapters

### STM32 I2C adapter

Header: `stm32_i2c_bus.h`

```c
I2cBus Stm32I2cBus_Create(I2C_HandleTypeDef *handle);
```

Converts an initialized STM32 HAL I2C handle into the platform-independent
`I2cBus` interface used by sensor drivers.

### STM32 UART stream

Header: `stm32_uart_stream.h`

```c
bool Stm32UartStream_Start(Stm32UartStream *stream,
                           UART_HandleTypeDef *handle);
bool Stm32UartStream_Recover(Stm32UartStream *stream);
ByteStream Stm32UartStream_AsByteStream(Stm32UartStream *stream);
uint32_t Stm32UartStream_GetOverflowCount(const Stm32UartStream *stream);
```

Reception is interrupt-driven. A nonzero overflow count means the application
did not consume bytes quickly enough.

## Byte ring buffer

Header: `byte_ring_buffer.h`

```c
bool ByteRingBuffer_Init(ByteRingBuffer *buffer, uint8_t *storage,
                         uint16_t capacity);
void ByteRingBuffer_Clear(ByteRingBuffer *buffer);
bool ByteRingBuffer_Push(ByteRingBuffer *buffer, uint8_t byte);
bool ByteRingBuffer_Pop(ByteRingBuffer *buffer, uint8_t *byte);
```

The usable capacity is one byte smaller than the storage size because one slot
distinguishes full from empty. The UART adapter pushes from its receive callback
and application code pops from the main loop.

## Error-handling rules

- Check status values and validity flags before using sensor data.
- Treat `NOT_READY` differently from a bus failure.
- Do not use cached values after the associated health state becomes stale or
  offline.
- Use `Error_Handler` only for unrecoverable configuration/programming errors.
- Keep blocking console output out of interrupts and high-rate control paths.
- Use the health supervisor for retry timing instead of tight retry loops.

## Tests

Portable unit tests are under `tests/`:

```sh
cmake -S tests -B build/host-tests
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

Electrical and physical behavior must be checked using
`tests/HARDWARE_TESTS.md`.
