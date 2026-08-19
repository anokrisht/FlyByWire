# FlyByWire host tests

These tests currently exercise the ring buffer, NMEA/GPS, ICM-20948/IMU, and
base sensor-health state machine with deterministic fake streams and I2C
devices. A separate golden-packet test verifies the compact MAVLink 2 encoder
against a known `HEARTBEAT` frame. Tests do not yet cover the BMP3xx driver,
barometer service, ADC-backed
airspeed service, or application-level sensor supervision.

The ADC airspeed acquisition and physical pressure conversions require target
hardware checks. See `HARDWARE_TESTS.md` for the current target checklist.

Run with a native C compiler:

```sh
cmake -S tests -B build/host-tests
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

Electrical disconnection, voltage stability, antenna reception, interrupt
latency, and physical sensor calibration require separate hardware-in-the-loop
tests on the target board.

Python tests under `GroundStation/tests` cover MAVLink decoding, invalid GPS
sentinels, status-lamp transitions, CSV output, and flight statistics:

```sh
python -m pip install -r GroundStation/requirements-dev.txt
python -m pytest GroundStation/tests
```

GitHub Actions runs both test suites, checks Python syntax/lint, builds the ARM
firmware, and uploads `.elf`, `.bin`, and `.map` artifacts. Tags beginning with
`v` publish those firmware files as a GitHub release.
