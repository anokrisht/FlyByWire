import math
import pytest

from telemetry import TelemetryState


class Message:
    def __init__(self, message_type, **fields):
        self.message_type = message_type
        self.__dict__.update(fields)

    def get_type(self):
        return self.message_type


def test_invalid_gps_sentinels_are_not_displayed_as_measurements():
    state = TelemetryState()
    state.ingest(Message(
        "GPS_RAW_INT", fix_type=1, lat=0, lon=0, alt=0, eph=0xFFFF,
        epv=0xFFFF, vel=0xFFFF, cog=0xFFFF, satellites_visible=0,
    ))
    assert math.isnan(state.ground_speed_mps)
    assert math.isnan(state.latitude_deg)
    assert math.isnan(state.gps_altitude_m)
    assert math.isnan(state.hdop)


def test_attitude_is_converted_to_degrees():
    state = TelemetryState()
    state.ingest(Message("ATTITUDE", roll=math.pi / 6,
                         pitch=-math.pi / 4, yaw=math.pi))
    assert state.roll_deg == pytest.approx(30.0)
    assert state.pitch_deg == pytest.approx(-45.0)
    assert state.yaw_deg == pytest.approx(180.0)


def test_timestamped_fault_sets_and_recovery_clears_lamp():
    state = TelemetryState()
    state.ingest(Message(
        "STATUSTEXT", severity=2,
        text="[2026-08-19T14:32:08.514Z] IMU: OFFLINE",
    ))
    assert state.alert_level() == "red"
    state.ingest(Message(
        "STATUSTEXT", severity=6,
        text="[2026-08-19T14:32:10.000Z] IMU: OK",
    ))
    assert state.alert_level() == "green"


def test_warning_selects_yellow_lamp():
    state = TelemetryState()
    state.ingest(Message("STATUSTEXT", severity=4,
                         text="[T+00:00:05.000] GPS: DEGRADED"))
    assert state.alert_level() == "yellow"
