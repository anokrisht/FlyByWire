# Hardware-in-the-loop test checklist

Run these tests on the assembled board after every sensor, power-system, or
interrupt-handling change. Record firmware revision, supply voltage, test time,
and observed recovery time.

## Startup

- Boot with both sensors connected: both health states reach `OK`.
- Boot without the IMU: application continues and IMU becomes `OFFLINE`.
- Reconnect the IMU: it reaches `OK` without resetting the STM32.
- Boot without the GPS: application continues; GPS becomes `STALE`/`OFFLINE`.
- Reconnect the GPS: valid NMEA reception and structured fixes resume.

## Runtime disconnection and power loss

- Remove and reconnect I2C SDA, then SCL: no firmware lockup; IMU recovers.
- Power-cycle only the ICM-20948: register configuration and sampling recover.
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
