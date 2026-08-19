"""Flight-session statistics and CSV recording for the ground station."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import math
from pathlib import Path
from statistics import fmean
import time

from telemetry import TelemetryState


CSV_FIELDS = (
    "computer_utc", "message_count", "link_connected", "alert_level",
    "roll_deg", "pitch_deg", "heading_deg", "airspeed_mps",
    "ground_speed_mps", "pressure_altitude_m", "gps_altitude_m",
    "latitude_deg", "longitude_deg", "gps_fix_type", "satellites",
    "hdop", "vdop", "pressure_hpa", "differential_pressure_hpa",
    "barometer_temperature_c", "imu_temperature_c",
    "accel_x_mps2", "accel_y_mps2", "accel_z_mps2",
    "gyro_x_rps", "gyro_y_rps", "gyro_z_rps",
    "mag_x_gauss", "mag_y_gauss", "mag_z_gauss",
)


class CsvRecorder:
    def __init__(self) -> None:
        self.file = None
        self.writer: csv.DictWriter | None = None
        self.path: Path | None = None

    @property
    def active(self) -> bool:
        return self.file is not None

    def start(self, path: str) -> None:
        self.stop()
        self.path = Path(path)
        self.file = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        self.file.flush()

    def append(self, state: TelemetryState) -> None:
        if self.writer is None or self.file is None:
            return
        row = {
            "computer_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "message_count": state.message_count,
            "link_connected": state.connected,
            "alert_level": state.alert_level(),
            "roll_deg": state.roll_deg,
            "pitch_deg": state.pitch_deg,
            "heading_deg": state.yaw_deg,
            "airspeed_mps": state.airspeed_mps,
            "ground_speed_mps": state.ground_speed_mps,
            "pressure_altitude_m": state.altitude_m,
            "gps_altitude_m": state.gps_altitude_m,
            "latitude_deg": state.latitude_deg,
            "longitude_deg": state.longitude_deg,
            "gps_fix_type": state.gps_fix_type,
            "satellites": state.satellites_visible,
            "hdop": state.hdop,
            "vdop": state.vdop,
            "pressure_hpa": state.pressure_hpa,
            "differential_pressure_hpa": state.differential_pressure_hpa,
            "barometer_temperature_c": state.barometer_temperature_c,
            "imu_temperature_c": state.imu_temperature_c,
        }
        for prefix, values in (("accel", state.acceleration_mps2),
                               ("gyro", state.angular_rate_rps),
                               ("mag", state.magnetic_field_gauss)):
            unit = {"accel": "mps2", "gyro": "rps", "mag": "gauss"}[prefix]
            for axis, value in zip(("x", "y", "z"), values):
                row[f"{prefix}_{axis}_{unit}"] = value
        self.writer.writerow(row)
        self.file.flush()

    def stop(self) -> Path | None:
        path = self.path
        if self.file is not None:
            self.file.close()
        self.file = None
        self.writer = None
        self.path = None
        return path


@dataclass
class FlightStatistics:
    started_monotonic: float | None = None
    maximum_airspeed_mps: float = math.nan
    maximum_ground_speed_mps: float = math.nan
    minimum_pressure_altitude_m: float = math.nan
    maximum_pressure_altitude_m: float = math.nan
    minimum_temperature_c: float = math.nan
    maximum_temperature_c: float = math.nan
    distance_m: float = 0.0
    sample_count: int = 0
    previous_position: tuple[float, float] | None = None

    def reset(self) -> None:
        self.__dict__.update(FlightStatistics().__dict__)

    def update(self, state: TelemetryState) -> None:
        if not state.connected:
            return
        if self.started_monotonic is None:
            self.started_monotonic = time.monotonic()
        self.sample_count += 1
        self.maximum_airspeed_mps = _maximum(
            self.maximum_airspeed_mps, state.airspeed_mps)
        self.maximum_ground_speed_mps = _maximum(
            self.maximum_ground_speed_mps, state.ground_speed_mps)
        self.minimum_pressure_altitude_m = _minimum(
            self.minimum_pressure_altitude_m, state.altitude_m)
        self.maximum_pressure_altitude_m = _maximum(
            self.maximum_pressure_altitude_m, state.altitude_m)
        self.minimum_temperature_c = _minimum(
            self.minimum_temperature_c, state.barometer_temperature_c)
        self.maximum_temperature_c = _maximum(
            self.maximum_temperature_c, state.barometer_temperature_c)
        if state.gps_fix_type >= 2 and not math.isnan(state.latitude_deg):
            position = (state.latitude_deg, state.longitude_deg)
            if self.previous_position is not None and position != self.previous_position:
                self.distance_m += _haversine(self.previous_position, position)
            self.previous_position = position

    @property
    def duration_s(self) -> float:
        return (0.0 if self.started_monotonic is None else
                time.monotonic() - self.started_monotonic)


def _minimum(current: float, candidate: float) -> float:
    if math.isnan(candidate):
        return current
    return candidate if math.isnan(current) else min(current, candidate)


def _maximum(current: float, candidate: float) -> float:
    if math.isnan(candidate):
        return current
    return candidate if math.isnan(current) else max(current, candidate)


def _haversine(first: tuple[float, float], second: tuple[float, float]) -> float:
    radius_m = 6371000.0
    lat1, lon1 = map(math.radians, first)
    lat2, lon2 = map(math.radians, second)
    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1
    value = (math.sin(delta_lat / 2.0) ** 2 + math.cos(lat1) *
             math.cos(lat2) * math.sin(delta_lon / 2.0) ** 2)
    return 2.0 * radius_m * math.asin(min(1.0, math.sqrt(value)))


def calculate_stationary_calibration(samples: list[dict]) -> dict:
    if not samples:
        raise ValueError("at least one stationary sample is required")
    acceleration = [
        fmean(float(sample["acceleration"][axis]) for sample in samples)
        for axis in range(3)
    ]
    gyro = [
        fmean(float(sample["gyro"][axis]) for sample in samples)
        for axis in range(3)
    ]
    pressure = [float(sample["differential_pressure_hpa"])
                for sample in samples
                if not math.isnan(float(sample["differential_pressure_hpa"]))]
    return {
        "sample_count": len(samples),
        "acceleration_mean_mps2": acceleration,
        "suggested_acceleration_bias_mps2": [
            acceleration[0], acceleration[1], acceleration[2] - 9.80665],
        "suggested_gyro_bias_rps": gyro,
        "suggested_differential_pressure_zero_hpa":
            fmean(pressure) if pressure else None,
    }


def calculate_magnetometer_calibration(samples: list[dict]) -> dict:
    if len(samples) < 2:
        raise ValueError("at least two magnetometer samples are required")
    axes = [[float(sample["magnetic"][axis]) for sample in samples]
            for axis in range(3)]
    minimum = [min(axis) for axis in axes]
    maximum = [max(axis) for axis in axes]
    offset = [(low + high) / 2.0 for low, high in zip(minimum, maximum)]
    radii = [(high - low) / 2.0 for low, high in zip(minimum, maximum)]
    average_radius = fmean(radii)
    scale = [(average_radius / radius) if radius > 1e-6 else 1.0
             for radius in radii]
    return {
        "sample_count": len(samples), "minimum_gauss": minimum,
        "maximum_gauss": maximum, "suggested_offset_gauss": offset,
        "suggested_scale": scale,
    }
