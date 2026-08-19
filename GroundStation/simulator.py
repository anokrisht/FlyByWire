"""Generate realistic MAVLink sensor traffic for dashboard development."""

from __future__ import annotations

import argparse
import math
import time

from pymavlink import mavutil


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="udpout:127.0.0.1:14550")
    args = parser.parse_args()
    link = mavutil.mavlink_connection(args.endpoint, source_system=1,
                                      source_component=1, dialect="common")
    link.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO,
                             b"Ground-station simulator active")
    started = time.monotonic()
    last_heartbeat = -1.0
    while True:
        elapsed = time.monotonic() - started
        boot_ms = int(elapsed * 1000) & 0xFFFFFFFF
        if elapsed - last_heartbeat >= 1.0:
            link.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_FIXED_WING,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            last_heartbeat = elapsed

        roll = math.radians(20.0 * math.sin(elapsed * .6))
        pitch = math.radians(8.0 * math.sin(elapsed * .35))
        yaw = (elapsed * .12) % (2.0 * math.pi)
        altitude = 120.0 + 12.0 * math.sin(elapsed * .12)
        airspeed = 18.0 + 2.5 * math.sin(elapsed * .5)
        pressure = 1013.25 * pow(1.0 - altitude / 44330.0, 5.255)
        latitude = 48.8566 + .002 * math.sin(elapsed * .025)
        longitude = 2.3522 + .003 * math.cos(elapsed * .025)

        link.mav.attitude_send(boot_ms, roll, pitch, yaw,
                               .2 * math.cos(elapsed * .6), 0.05, .12)
        link.mav.highres_imu_send(
            int(elapsed * 1e6),
            .5 * math.sin(elapsed), .3 * math.cos(elapsed), 9.80665,
            .2 * math.cos(elapsed * .6), .05, .12,
            .22, .03, .41, pressure, .5 * airspeed * airspeed / 100.0,
            altitude, 28.0 + math.sin(elapsed * .1), 0x1FFF)
        link.mav.scaled_pressure_send(
            boot_ms, pressure, .5 * airspeed * airspeed / 100.0, 2450)
        link.mav.vfr_hud_send(airspeed, airspeed * .95,
                              int(math.degrees(yaw)) % 360, 0, altitude, 0.0)
        link.mav.gps_raw_int_send(
            int(elapsed * 1e6), 3, int(latitude * 1e7), int(longitude * 1e7),
            int(altitude * 1000), 85, 120, int(airspeed * 95),
            int(math.degrees(yaw) * 100) % 36000, 14)
        time.sleep(.05)


if __name__ == "__main__":
    main()
