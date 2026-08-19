# FlyByWire

STM32F446-based sensor platform with an ICM-20948 IMU, BMP388/BMP390
barometer, analog differential-pressure airspeed sensor, NMEA GNSS receiver,
UART telemetry, automatic sensor recovery, and host-side unit tests.

## Developer documentation

- [Public API and usage guide](docs/API.md)
- [Host test instructions](tests/README.md)
- [Hardware-in-the-loop test checklist](tests/HARDWARE_TESTS.md)
- [MAVLink graphical sensor station](GroundStation/README.md)

Public contracts are grouped with their components under `Application`,
`Drivers`, `Services`, `Platform`, and `Utilities`. CubeMX-generated code stays
under `Core`.

## Project structure

```text
Application/          Top-level orchestration for all flight-software sections
  DataAcquisition/    Sensor sampling, unified data snapshot, and supervision
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
  Telemetry/          Standard MAVLink 2 encoding and UART publication
Utilities/            Reusable platform-independent utilities
Core/                  STM32CubeMX-generated startup and peripheral code
tests/                 Portable unit and hardware test definitions
docs/                  Developer-facing documentation
GroundStation/         Python MAVLink dashboard and telemetry simulator
```

Each component keeps public headers in `Inc` and implementations in `Src`.

The data-acquisition section publishes every current sensor value and health
state through one typed `DataAcquisitionData` snapshot. Future application
sections can access it with `Application_GetData()` without depending on sensor
drivers or acquisition internals.
