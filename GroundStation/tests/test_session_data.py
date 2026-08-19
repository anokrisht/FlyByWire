import csv
import math
import pytest

from session_data import (
    CSV_FIELDS, CsvRecorder, FlightStatistics,
    calculate_magnetometer_calibration, calculate_stationary_calibration,
)
from telemetry import TelemetryState


def populated_state():
    state = TelemetryState(connected=True, message_count=10)
    state.roll_deg = 1.0
    state.pitch_deg = 2.0
    state.yaw_deg = 3.0
    state.airspeed_mps = 20.0
    state.ground_speed_mps = 18.0
    state.altitude_m = 120.0
    state.gps_altitude_m = 125.0
    state.latitude_deg = 48.0
    state.longitude_deg = 2.0
    state.gps_fix_type = 3
    state.barometer_temperature_c = 22.5
    state.acceleration_mps2 = [1.0, 2.0, 3.0]
    state.angular_rate_rps = [0.1, 0.2, 0.3]
    state.magnetic_field_gauss = [0.4, 0.5, 0.6]
    return state


def test_csv_recorder_writes_complete_row(tmp_path):
    path = tmp_path / "flight.csv"
    recorder = CsvRecorder()
    recorder.start(str(path))
    recorder.append(populated_state())
    assert recorder.stop() == path
    with path.open(newline="", encoding="utf-8") as recorded:
        rows = list(csv.DictReader(recorded))
    assert tuple(rows[0]) == CSV_FIELDS
    assert rows[0]["airspeed_mps"] == "20.0"
    assert rows[0]["mag_z_gauss"] == "0.6"


def test_flight_statistics_extrema_and_distance():
    statistics = FlightStatistics()
    state = populated_state()
    statistics.update(state)
    state.airspeed_mps = 25.0
    state.ground_speed_mps = 21.0
    state.altitude_m = 150.0
    state.barometer_temperature_c = 18.0
    state.latitude_deg = 48.001
    statistics.update(state)
    assert statistics.maximum_airspeed_mps == 25.0
    assert statistics.maximum_pressure_altitude_m == 150.0
    assert statistics.minimum_temperature_c == 18.0
    assert statistics.distance_m > 100.0
    assert not math.isnan(statistics.maximum_ground_speed_mps)


def test_stationary_calibration_biases():
    samples = [
        {"acceleration": [0.1, -0.2, 9.90665], "gyro": [0.01, 0.02, -0.03],
         "magnetic": [0.0, 0.0, 0.0], "differential_pressure_hpa": 0.04},
        {"acceleration": [0.1, -0.2, 9.90665], "gyro": [0.01, 0.02, -0.03],
         "magnetic": [0.0, 0.0, 0.0], "differential_pressure_hpa": 0.06},
    ]
    result = calculate_stationary_calibration(samples)
    assert result["suggested_acceleration_bias_mps2"] == pytest.approx(
        [0.1, -0.2, 0.1])
    assert result["suggested_gyro_bias_rps"] == pytest.approx(
        [0.01, 0.02, -0.03])
    assert result["suggested_differential_pressure_zero_hpa"] == pytest.approx(0.05)


def test_magnetometer_calibration_offsets_and_scale():
    samples = [
        {"magnetic": [-2.0, -4.0, -6.0]},
        {"magnetic": [4.0, 2.0, 6.0]},
    ]
    result = calculate_magnetometer_calibration(samples)
    assert result["suggested_offset_gauss"] == [1.0, -1.0, 0.0]
    assert all(value > 0.0 for value in result["suggested_scale"])
