# Hardware-in-the-loop test checklist

Run these tests on the assembled board after every sensor, power-system, or
interrupt-handling change. Record firmware revision, supply voltage, test time,
and observed recovery time.

## Startup

- Boot with IMU, BMP388/BMP390, and GPS connected: all health states reach `OK`.
- Boot without the BMP390: application continues and barometer becomes `OFFLINE`.
- Reconnect the BMP390: it reaches `OK` without resetting the STM32.
- Boot without the IMU: application continues and IMU becomes `OFFLINE`.
- Reconnect the IMU: it reaches `OK` without resetting the STM32.
- Boot without the GPS: application continues; GPS becomes `STALE`/`OFFLINE`.
- Reconnect the GPS: valid NMEA reception and structured fixes resume.

## Runtime disconnection and power loss

- Remove and reconnect I2C SDA, then SCL: no firmware lockup; IMU recovers.
- Power-cycle only the ICM-20948: register configuration and sampling recover.
- Power-cycle only the BMP390: calibration, configuration, and sampling recover.
- Remove and reconnect GPS TX: NMEA reception resumes within the retry period.
- Power-cycle only the GPS: UART reception resumes after the receiver boots.
- Disconnect each sensor for at least 30 seconds: retry counters do not wrap or
  block the main loop.

## Data integrity

- Corrupt an NMEA checksum: sentence is rejected and the checksum-error count
  increments.
- Remove GPS antenna: fix becomes invalid/stale without retaining a valid fix as
  current data; reconnect antenna and verify recovery.
- Generate UART traffic above the configured rate: overflow count increments
  and normal parsing resumes after the overload stops.
- Move the IMU through known orientations and verify axis signs, roll, pitch,
  yaw, and heading conventions.
- Compare BMP390 pressure and temperature with a reference; verify altitude
  changes in the expected direction when raising and lowering the board.
- Boot with both pitot ports at equal pressure and verify airspeed auto-zeroes
  near 0 m/s. Apply pressure to the positive port and verify positive airspeed.
- Verify the airspeed startup log reports a stable zero ADC count over repeated
  boots, and compare differential pressure against a calibrated manometer.
- Verify the 1 kOhm series / 2 kOhm-to-ground divider keeps PA0 at or below
  3.3 V across the full CJMCU-36 output range.

## Timing and load

- Run maximum UART telemetry while receiving all NMEA messages: no UART overflow.
- Run for at least one hour and verify update counters, failure counters, and
  memory remain stable.
- Exercise `HAL_GetTick()` wraparound or accelerated equivalent and confirm
  stale/retry deadlines continue operating.

## Electrical fault safety

- Test only with current-limited supplies.
- Do not short signal pins directly to supply rails.
- Verify both modules use compatible logic levels before fault injection.
