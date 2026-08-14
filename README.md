# FlyByWire

STM32F446-based sensor platform with an ICM-20948 IMU, BMP388/BMP390
barometer, analog differential-pressure airspeed sensor, NMEA GNSS receiver,
UART telemetry, automatic sensor recovery, and host-side unit tests.

## Developer documentation

- [Public API and usage guide](docs/API.md)
- [Host test instructions](tests/README.md)
- [Hardware-in-the-loop test checklist](tests/HARDWARE_TESTS.md)

Public contracts are grouped with their components under `Application`,
`Drivers`, `Services`, `Platform`, and `Utilities`. CubeMX-generated code stays
under `Core`.

## Project structure

```text
Application/          Application orchestration and scheduling
Drivers/
  BMP390/             BMP388/BMP390 pressure and temperature driver
  GNSS/               NMEA framing and checksum driver
  ICM20948/           ICM-20948 and AK09916 register driver
Platform/
  Interfaces/         Hardware-independent bus contracts
  STM32/              STM32 HAL adapters and UART console
Services/
  Airspeed/           ADC pressure conversion and indicated airspeed
  Barometer/          Pressure, temperature, and altitude service
  GPS/                Structured navigation data and GPS getters
  Health/             Sensor supervision and recovery state machine
  IMU/                Calibration, conversion, and orientation service
Utilities/            Reusable platform-independent utilities
Core/                  STM32CubeMX-generated startup and peripheral code
tests/                 Portable unit and hardware test definitions
docs/                  Developer-facing documentation
```

Each component keeps public headers in `Inc` and implementations in `Src`.
