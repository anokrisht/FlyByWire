"""Transport-independent telemetry state and MAVLink message decoding."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from typing import Any


@dataclass
class TelemetryState:
    connected: bool = False
    last_message_monotonic: float = 0.0
    system_id: int = 0
    component_id: int = 0
    message_count: int = 0
    message_rates: dict[str, float] = field(default_factory=dict)
    _rate_counts: dict[str, int] = field(default_factory=dict)
    _rate_started: float = field(default_factory=time.monotonic)

    roll_deg: float = math.nan
    pitch_deg: float = math.nan
    yaw_deg: float = math.nan
    acceleration_mps2: list[float] = field(
        default_factory=lambda: [math.nan] * 3)
    angular_rate_rps: list[float] = field(
        default_factory=lambda: [math.nan] * 3)
    magnetic_field_gauss: list[float] = field(
        default_factory=lambda: [math.nan] * 3)
    imu_temperature_c: float = math.nan
    pressure_hpa: float = math.nan
    differential_pressure_hpa: float = math.nan
    barometer_temperature_c: float = math.nan
    altitude_m: float = math.nan
    airspeed_mps: float = math.nan

    latitude_deg: float = math.nan
    longitude_deg: float = math.nan
    gps_altitude_m: float = math.nan
    ground_speed_mps: float = math.nan
    course_deg: float = math.nan
    gps_fix_type: int = 0
    satellites_visible: int = 0
    hdop: float = math.nan
    vdop: float = math.nan

    heartbeat_type: int = 0
    autopilot: int = 0
    base_mode: int = 0
    system_status: int = 0
    status_messages: list[str] = field(default_factory=list)
    active_alerts: dict[str, int] = field(default_factory=dict)

    def ingest(self, message: Any) -> None:
        """Merge a pymavlink message (or a compatible test double)."""
        kind = message.get_type()
        if kind == "BAD_DATA":
            return

        self.connected = True
        self.last_message_monotonic = time.monotonic()
        self.message_count += 1
        self._rate_counts[kind] = self._rate_counts.get(kind, 0) + 1
        header = getattr(message, "_header", None)
        if header is not None:
            self.system_id = int(getattr(header, "srcSystem", self.system_id))
            self.component_id = int(
                getattr(header, "srcComponent", self.component_id))

        handler = getattr(self, f"_decode_{kind.lower()}", None)
        if handler is not None:
            handler(message)
        self._update_rates()

    def update_connection(self, timeout_s: float = 3.0) -> None:
        if self.last_message_monotonic:
            self.connected = (
                time.monotonic() - self.last_message_monotonic) <= timeout_s
        self._update_rates()

    def _update_rates(self) -> None:
        now = time.monotonic()
        elapsed = now - self._rate_started
        if elapsed < 1.0:
            return
        self.message_rates = {
            name: count / elapsed for name, count in self._rate_counts.items()
        }
        self._rate_counts.clear()
        self._rate_started = now

    def _decode_heartbeat(self, msg: Any) -> None:
        self.heartbeat_type = int(msg.type)
        self.autopilot = int(msg.autopilot)
        self.base_mode = int(msg.base_mode)
        self.system_status = int(msg.system_status)

    def _decode_highres_imu(self, msg: Any) -> None:
        self.acceleration_mps2 = [float(msg.xacc), float(msg.yacc), float(msg.zacc)]
        self.angular_rate_rps = [float(msg.xgyro), float(msg.ygyro), float(msg.zgyro)]
        self.magnetic_field_gauss = [float(msg.xmag), float(msg.ymag), float(msg.zmag)]
        self.pressure_hpa = float(msg.abs_pressure)
        self.differential_pressure_hpa = float(msg.diff_pressure)
        self.altitude_m = float(msg.pressure_alt)
        self.imu_temperature_c = float(msg.temperature)

    def _decode_attitude(self, msg: Any) -> None:
        self.roll_deg = math.degrees(float(msg.roll))
        self.pitch_deg = math.degrees(float(msg.pitch))
        self.yaw_deg = math.degrees(float(msg.yaw)) % 360.0

    def _decode_scaled_pressure(self, msg: Any) -> None:
        self.pressure_hpa = float(msg.press_abs)
        self.differential_pressure_hpa = float(msg.press_diff)
        self.barometer_temperature_c = float(msg.temperature) / 100.0

    def _decode_vfr_hud(self, msg: Any) -> None:
        self.airspeed_mps = float(msg.airspeed)
        self.ground_speed_mps = (float(msg.groundspeed)
                                 if self.gps_fix_type >= 2 else math.nan)
        self.course_deg = float(msg.heading) % 360.0
        self.altitude_m = float(msg.alt)

    def _decode_gps_raw_int(self, msg: Any) -> None:
        self.gps_fix_type = int(msg.fix_type)
        position_valid = self.gps_fix_type >= 2
        self.latitude_deg = (float(msg.lat) / 1.0e7
                             if position_valid else math.nan)
        self.longitude_deg = (float(msg.lon) / 1.0e7
                              if position_valid else math.nan)
        self.gps_altitude_m = (float(msg.alt) / 1000.0
                               if position_valid else math.nan)
        self.hdop = _scaled_optional(msg.eph, 100.0, 0xFFFF)
        self.vdop = _scaled_optional(msg.epv, 100.0, 0xFFFF)
        self.ground_speed_mps = _scaled_optional(msg.vel, 100.0, 0xFFFF)
        if position_valid and int(msg.cog) != 0xFFFF:
            self.course_deg = float(msg.cog) / 100.0
        else:
            self.course_deg = math.nan
        self.satellites_visible = int(msg.satellites_visible)

    def _decode_statustext(self, msg: Any) -> None:
        raw_text = msg.text
        text = (raw_text.decode("utf-8", errors="replace")
                if isinstance(raw_text, bytes) else str(raw_text))
        text = text.rstrip("\x00").strip()
        if not text:
            return
        severity = int(msg.severity)
        entry = f"[{_severity_name(severity)}] {text}"
        if not self.status_messages or self.status_messages[-1] != entry:
            self.status_messages.append(entry)
            del self.status_messages[:-20]

        source_text = text
        if source_text.startswith("[") and "] " in source_text:
            source_text = source_text.split("] ", 1)[1]
        source = source_text.split(":", 1)[0].strip().upper()
        if severity <= 4:
            self.active_alerts[source] = severity
        else:
            self.active_alerts.pop(source, None)

    def alert_level(self) -> str:
        """Return the lamp color for currently active MAVLink reports."""
        if any(severity <= 3 for severity in self.active_alerts.values()):
            return "red"
        if any(severity == 4 for severity in self.active_alerts.values()):
            return "yellow"
        return "green"


def _scaled_optional(value: Any, scale: float, invalid: int) -> float:
    numeric = int(value)
    return math.nan if numeric == invalid else numeric / scale


def _severity_name(severity: int) -> str:
    names = {
        0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL", 3: "ERROR",
        4: "WARNING", 5: "NOTICE", 6: "INFO", 7: "DEBUG",
    }
    return names.get(severity, f"LEVEL {severity}")
