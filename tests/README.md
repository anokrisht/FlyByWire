# FlyByWire host tests

These tests exercise the platform-independent sensor code with deterministic
fake byte streams and I2C devices. They do not require an STM32 board.

Run with a native C compiler:

```sh
cmake -S tests -B build/host-tests
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

Electrical disconnection, voltage stability, antenna reception, interrupt
latency, and physical sensor calibration require separate hardware-in-the-loop
tests on the target board.
