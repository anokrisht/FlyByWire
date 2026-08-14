# FlyByWire host tests

These tests currently exercise the ring buffer, NMEA/GPS, ICM-20948/IMU, and
base sensor-health state machine with deterministic fake streams and I2C
devices. They do not yet cover the BMP3xx driver, barometer service, ADC-backed
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
