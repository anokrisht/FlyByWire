"""Qt/pyqtgraph dashboard widgets for MAVLink sensor telemetry."""

from __future__ import annotations

from collections import deque
from datetime import datetime
import json
import math
import time

import pyqtgraph as pg
from PySide6.QtCore import QPointF, Qt, QTimer, QUrl
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QComboBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QFileDialog, QLineEdit, QMainWindow, QMessageBox, QPlainTextEdit,
    QPushButton, QTabWidget, QVBoxLayout, QWidget,
)

try:
    from PySide6.QtWebEngineCore import QWebEngineSettings
    from PySide6.QtWebEngineWidgets import QWebEngineView
except ImportError:  # Keep the offline ground-track plot available.
    QWebEngineSettings = None
    QWebEngineView = None

from receiver import MavlinkReceiver
from session_data import (
    CsvRecorder, FlightStatistics, calculate_magnetometer_calibration,
    calculate_stationary_calibration,
)
from telemetry import TelemetryState


MAP_HTML = """<!doctype html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>
html, body, #map { height: 100%; margin: 0; background: #18212b; }
.aircraft { color: #f5c542; font-size: 25px; text-shadow: 0 0 3px #000; }
</style></head><body><div id="map"></div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
const map = L.map('map', {zoomControl: true}).setView([20, 0], 2);
L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 19,
  attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
}).addTo(map);
const track = L.polyline([], {color: '#f5c542', weight: 3}).addTo(map);
const aircraftIcon = L.divIcon({
  className: 'aircraft', html: '&#9992;', iconSize: [28, 28], iconAnchor: [14, 14]
});
let aircraft = null;
let receivedFirstFix = false;
window.updateVehicle = function(lat, lon, heading) {
  const position = [lat, lon];
  if (aircraft === null) {
    aircraft = L.marker(position, {icon: aircraftIcon}).addTo(map)
      .bindTooltip('', {direction: 'top'});
  } else {
    aircraft.setLatLng(position);
  }
  aircraft.setTooltipContent(
    lat.toFixed(6) + ', ' + lon.toFixed(6) + ' · ' + heading.toFixed(1) + '°');
  track.addLatLng(position);
  if (!receivedFirstFix) {
    map.setView(position, 16);
    receivedFirstFix = true;
  } else if (!map.getBounds().pad(-0.15).contains(position)) {
    map.panTo(position);
  }
};
</script></body></html>"""


class AttitudeIndicator(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.roll = 0.0
        self.pitch = 0.0
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.setMinimumSize(300, 260)
        self.animation_timer = QTimer(self)
        self.animation_timer.timeout.connect(self._animate)
        self.animation_timer.start(16)

    def set_attitude(self, roll: float, pitch: float) -> None:
        self.target_roll = 0.0 if math.isnan(roll) else roll
        self.target_pitch = 0.0 if math.isnan(pitch) else pitch

    def _animate(self) -> None:
        """Ease the horizon toward each new roll and pitch sample."""
        smoothing = 0.14
        self.roll += (self.target_roll - self.roll) * smoothing
        self.pitch += (self.target_pitch - self.pitch) * smoothing
        if abs(self.target_roll - self.roll) < 0.01:
            self.roll = self.target_roll
        if abs(self.target_pitch - self.pitch) < 0.01:
            self.pitch = self.target_pitch
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt API name
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        center = QPointF(self.width() / 2.0, self.height() / 2.0)
        radius = min(self.width(), self.height()) * 0.43
        painter.setClipRect(self.rect())
        painter.translate(center)
        painter.rotate(-self.roll)
        pitch_offset = max(-90.0, min(90.0, self.pitch)) * radius / 45.0
        painter.translate(0.0, pitch_offset)
        painter.fillRect(-self.width(), -self.height() * 2, self.width() * 2,
                         self.height() * 2, QColor("#2b78c5"))
        painter.fillRect(-self.width(), 0, self.width() * 2,
                         self.height() * 2, QColor("#80552f"))
        painter.setPen(QPen(Qt.white, 2))
        painter.drawLine(-self.width(), 0, self.width(), 0)
        for angle in range(-60, 61, 10):
            if angle == 0:
                continue
            y = -angle * radius / 45.0
            half = radius * (0.28 if angle % 20 else 0.42)
            painter.drawLine(QPointF(-half, y), QPointF(half, y))
            if angle % 20 == 0:
                painter.drawText(QPointF(-half - 28, y + 5), str(abs(angle)))
                painter.drawText(QPointF(half + 8, y + 5), str(abs(angle)))
        painter.resetTransform()
        painter.translate(center)
        painter.setClipping(False)
        painter.setPen(QPen(QColor("#d9e2ec"), 2))
        for angle in range(-60, 61, 10):
            radians = math.radians(angle - 90)
            outer = QPointF(radius * math.cos(radians),
                            radius * math.sin(radians))
            tick_length = 14 if angle % 30 == 0 else 8
            inner_radius = radius - tick_length
            inner = QPointF(inner_radius * math.cos(radians),
                            inner_radius * math.sin(radians))
            painter.drawLine(inner, outer)
            if angle % 30 == 0:
                label_radius = radius + 18
                label_position = QPointF(
                    label_radius * math.cos(radians) - 10,
                    label_radius * math.sin(radians) + 5)
                label = "0" if angle == 0 else f"{angle:+d}"
                painter.drawText(label_position, label)
        painter.setPen(QPen(QColor("#f5c542"), 4))
        painter.drawLine(QPointF(-radius * .45, 0), QPointF(-radius * .12, 0))
        painter.drawLine(QPointF(radius * .12, 0), QPointF(radius * .45, 0))
        painter.drawLine(QPointF(0, -8), QPointF(0, 10))
        painter.setPen(QPen(QColor("#d9e2ec"), 3))
        painter.drawEllipse(QPointF(0, 0), radius, radius)


class StatusLamps(QWidget):
    COLORS = {
        "green": ("#29d66f", "#173b28"),
        "yellow": ("#ffd43b", "#403919"),
        "red": ("#ff4d5a", "#411d22"),
    }

    def __init__(self) -> None:
        super().__init__()
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 0, 8, 0)
        layout.setSpacing(8)
        self.lamps: dict[str, QLabel] = {}
        symbols = {"green": "✓", "yellow": "!", "red": "✕"}
        for name in ("green", "yellow", "red"):
            lamp = QLabel(symbols[name])
            lamp.setAlignment(Qt.AlignCenter)
            lamp.setFixedSize(34, 34)
            lamp.setToolTip(name.capitalize())
            self.lamps[name] = lamp
            layout.addWidget(lamp)
        self.set_level("red")

    def set_level(self, active: str) -> None:
        for name, lamp in self.lamps.items():
            bright, dim = self.COLORS[name]
            color = bright if name == active else dim
            text_color = "#07120b" if name == active else "#82909c"
            lamp.setStyleSheet(
                f"background: {color}; color: {text_color}; "
                "border: 2px solid #667582; border-radius: 17px; "
                "font-size: 18px; font-weight: bold;")


class HistoryPlot(pg.PlotWidget):
    COLORS = ("#3da5ff", "#ff7066", "#60d394")

    def __init__(self, title: str, labels: tuple[str, ...], unit: str) -> None:
        super().__init__()
        self.setTitle(title)
        self.setLabel("left", unit)
        self.setLabel("bottom", "Time", "s")
        self.showGrid(x=True, y=True, alpha=0.2)
        self.addLegend()
        self.times: deque[float] = deque(maxlen=600)
        self.values = [deque(maxlen=600) for _ in labels]
        self.curves = [
            self.plot(name=label, pen=pg.mkPen(self.COLORS[i], width=2))
            for i, label in enumerate(labels)
        ]

    def append(self, timestamp: float, values: list[float]) -> None:
        self.times.append(timestamp)
        for history, value in zip(self.values, values):
            history.append(value)
        if not self.times:
            return
        origin = self.times[-1]
        x = [sample - origin for sample in self.times]
        for curve, history in zip(self.curves, self.values):
            curve.setData(x, list(history))


class CalibrationAssistant(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.mode: str | None = None
        self.deadline = 0.0
        self.samples: list[dict[str, list[float] | float]] = []
        self.results: dict[str, object] = {}
        layout = QVBoxLayout(self)
        instructions = QLabel(
            "Stationary calibration: keep the unit level and motionless for 5 seconds.\n"
            "Magnetometer calibration: rotate the unit slowly through every orientation, "
            "then press Finish. Results are saved as a report; they are not written "
            "to the flight controller automatically.")
        instructions.setWordWrap(True)
        layout.addWidget(instructions)
        buttons = QHBoxLayout()
        stationary = QPushButton("Start stationary calibration (5 s)")
        stationary.clicked.connect(self.start_stationary)
        self.mag_button = QPushButton("Start magnetometer calibration")
        self.mag_button.clicked.connect(self.toggle_magnetometer)
        save = QPushButton("Save calibration report")
        save.clicked.connect(self.save_report)
        buttons.addWidget(stationary); buttons.addWidget(self.mag_button)
        buttons.addWidget(save)
        layout.addLayout(buttons)
        self.progress = QLabel("Ready")
        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        layout.addWidget(self.progress)
        layout.addWidget(self.output, 1)

    def start_stationary(self) -> None:
        self.mode = "stationary"
        self.deadline = time.monotonic() + 5.0
        self.samples.clear()
        self.progress.setText("Collecting stationary samples…")

    def toggle_magnetometer(self) -> None:
        if self.mode == "magnetometer":
            self._finish_magnetometer()
            return
        self.mode = "magnetometer"
        self.samples.clear()
        self.mag_button.setText("Finish magnetometer calibration")
        self.progress.setText("Rotate through all orientations…")

    def sample(self, state: TelemetryState) -> None:
        if self.mode is None:
            return
        if any(math.isnan(value) for value in state.acceleration_mps2 +
               state.angular_rate_rps + state.magnetic_field_gauss):
            self.progress.setText("Waiting for valid IMU telemetry…")
            return
        self.samples.append({
            "acceleration": list(state.acceleration_mps2),
            "gyro": list(state.angular_rate_rps),
            "magnetic": list(state.magnetic_field_gauss),
            "differential_pressure_hpa": state.differential_pressure_hpa,
        })
        if self.mode == "stationary":
            remaining = max(0.0, self.deadline - time.monotonic())
            self.progress.setText(f"Collecting… {remaining:.1f} s remaining")
            if remaining <= 0.0:
                self._finish_stationary()
        else:
            self.progress.setText(
                f"Rotate through all orientations… {len(self.samples)} samples")

    def _finish_stationary(self) -> None:
        if not self.samples:
            self.progress.setText("No valid samples collected")
            self.mode = None
            return
        self.results["stationary"] = calculate_stationary_calibration(
            self.samples)
        self.mode = None
        self.progress.setText("Stationary calibration complete")
        self._show_results()

    def _finish_magnetometer(self) -> None:
        self.mode = None
        self.mag_button.setText("Start magnetometer calibration")
        if len(self.samples) < 20:
            self.progress.setText("Not enough magnetometer samples")
            return
        self.results["magnetometer"] = calculate_magnetometer_calibration(
            self.samples)
        self.progress.setText("Magnetometer calibration complete")
        self._show_results()

    def _show_results(self) -> None:
        self.output.setPlainText(json.dumps(self.results, indent=2))

    def save_report(self) -> None:
        if not self.results:
            QMessageBox.information(self, "Calibration", "No calibration results yet.")
            return
        default = f"calibration-{datetime.now():%Y%m%d-%H%M%S}.json"
        path, _ = QFileDialog.getSaveFileName(
            self, "Save calibration report", default, "JSON files (*.json)")
        if path:
            with open(path, "w", encoding="utf-8") as report:
                json.dump(self.results, report, indent=2)


class Dashboard(QMainWindow):
    def __init__(self, initial_endpoint: str, initial_baud: int) -> None:
        super().__init__()
        self.state = TelemetryState()
        self.recorder = CsvRecorder()
        self.flight_statistics = FlightStatistics()
        self.receiver: MavlinkReceiver | None = None
        self.started_at = 0.0
        self.last_plotted_message = 0
        self.last_gps_position: tuple[float, float] | None = None
        self.setWindowTitle("FlyByWire MAVLink Sensor Station")
        self.resize(1280, 820)

        root = QWidget()
        layout = QVBoxLayout(root)
        layout.addLayout(self._connection_bar(initial_endpoint, initial_baud))
        self.tabs = QTabWidget()
        self.tabs.addTab(self._overview_tab(), "Flight instruments")
        self.tabs.addTab(self._imu_tab(), "IMU graphs")
        self.tabs.addTab(self._environment_tab(), "Air data")
        self.tabs.addTab(self._gps_tab(), "GPS track")
        self.tabs.addTab(self._statistics_tab(), "Flight statistics")
        self.calibration = CalibrationAssistant()
        self.tabs.addTab(self.calibration, "Calibration")
        self.tabs.addTab(self._status_tab(), "Link status")
        layout.addWidget(self.tabs)
        self.setCentralWidget(root)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(50)

    def _connection_bar(self, endpoint: str, baud: int) -> QHBoxLayout:
        layout = QHBoxLayout()
        self.endpoint = QLineEdit(endpoint)
        self.endpoint.setPlaceholderText("COM4, /dev/ttyUSB0, or udpin:0.0.0.0:14550")
        self.baud = QComboBox()
        self.baud.addItems(["57600", "115200", "230400", "460800", "921600"])
        self.baud.setCurrentText(str(baud))
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        self.record_button = QPushButton("Start recording")
        self.record_button.clicked.connect(self.toggle_recording)
        self.link_label = QLabel("DISCONNECTED")
        self.link_label.setStyleSheet("color: #ff7066; font-weight: bold")
        self.status_lamps = StatusLamps()
        layout.addWidget(QLabel("Endpoint"))
        layout.addWidget(self.endpoint, 1)
        layout.addWidget(QLabel("Baud"))
        layout.addWidget(self.baud)
        layout.addWidget(self.connect_button)
        layout.addWidget(self.record_button)
        layout.addWidget(self.link_label)
        layout.addWidget(self.status_lamps)
        return layout

    def _overview_tab(self) -> QWidget:
        tab = QWidget(); layout = QGridLayout(tab)
        self.attitude = AttitudeIndicator()
        layout.addWidget(self.attitude, 0, 0, 3, 1)
        self.heading = self._value_box("Heading", "--- °")
        self.roll_value = self._value_box("Roll", "--- °")
        self.pitch_value = self._value_box("Pitch", "--- °")
        self.airspeed = self._value_box("Airspeed", "--- m/s")
        self.ground_speed = self._value_box("Ground speed", "--- m/s")
        self.pressure_altitude = self._value_box("Pressure altitude", "--- m")
        self.gps_altitude = self._value_box("GPS altitude", "--- m")
        self.ambient_temperature = self._value_box(
            "Ambient temperature (barometer)", "--- °C")
        layout.addWidget(self.pitch_value, 0, 1)
        layout.addWidget(self.roll_value, 0, 2)
        layout.addWidget(self.heading, 0, 3)
        layout.addWidget(self.airspeed, 1, 1)
        layout.addWidget(self.ground_speed, 1, 2)
        layout.addWidget(self.pressure_altitude, 2, 1)
        layout.addWidget(self.gps_altitude, 2, 2)
        self.fault_messages = QPlainTextEdit()
        self.fault_messages.setReadOnly(True)
        self.fault_messages.setMaximumHeight(105)
        self.fault_messages.setPlaceholderText(
            "No fault or custom status messages received")
        fault_box = QGroupBox("Faults and status messages")
        fault_layout = QVBoxLayout(fault_box)
        fault_layout.addWidget(self.fault_messages)
        layout.addWidget(fault_box, 1, 3, 2, 1)
        layout.addWidget(self.ambient_temperature, 3, 1, 1, 3)
        return tab

    @staticmethod
    def _value_box(title: str, value: str) -> QGroupBox:
        box = QGroupBox(title); layout = QVBoxLayout(box)
        label = QLabel(value); label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 26px; font-weight: 600")
        label.setObjectName("value")
        layout.addWidget(label)
        return box

    def _imu_tab(self) -> QWidget:
        tab = QWidget(); layout = QVBoxLayout(tab)
        self.accel_plot = HistoryPlot("Acceleration", ("X", "Y", "Z"), "m/s²")
        self.gyro_plot = HistoryPlot("Angular rate", ("X", "Y", "Z"), "rad/s")
        self.mag_plot = HistoryPlot("Magnetic field", ("X", "Y", "Z"), "gauss")
        layout.addWidget(self.accel_plot); layout.addWidget(self.gyro_plot)
        layout.addWidget(self.mag_plot)
        return tab

    def _environment_tab(self) -> QWidget:
        tab = QWidget(); layout = QVBoxLayout(tab)
        self.pressure_plot = HistoryPlot("Pressure", ("Absolute", "Differential"), "hPa")
        self.air_plot = HistoryPlot("Altitude and airspeed", ("Altitude", "Airspeed"), "m / m/s")
        self.temperature_plot = HistoryPlot("Temperature", ("IMU", "Barometer"), "°C")
        layout.addWidget(self.pressure_plot); layout.addWidget(self.air_plot)
        layout.addWidget(self.temperature_plot)
        return tab

    def _gps_tab(self) -> QWidget:
        tab = QWidget(); layout = QVBoxLayout(tab)
        self.gps_summary = QLabel("No GPS data")
        self.map_view = None
        if QWebEngineView is not None:
            self.map_view = QWebEngineView()
            self.map_view.setMinimumHeight(350)
            self.map_view.settings().setAttribute(
                QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls,
                True)
            self.map_view.setHtml(MAP_HTML, QUrl("https://flybywire.local/"))
            layout.addWidget(self.map_view, 2)
        else:
            map_notice = QLabel(
                "Interactive map unavailable; install the full PySide6 package."
            )
            map_notice.setAlignment(Qt.AlignCenter)
            layout.addWidget(map_notice)
        self.track_plot = pg.PlotWidget(title="GPS ground track")
        self.track_plot.setLabel("left", "Latitude", "deg")
        self.track_plot.setLabel("bottom", "Longitude", "deg")
        self.track_plot.showGrid(x=True, y=True, alpha=.2)
        self.track_curve = self.track_plot.plot(pen=pg.mkPen("#f5c542", width=2), symbol="o")
        self.track_lat: deque[float] = deque(maxlen=2000)
        self.track_lon: deque[float] = deque(maxlen=2000)
        layout.addWidget(self.gps_summary); layout.addWidget(self.track_plot, 1)
        return tab

    def _status_tab(self) -> QWidget:
        tab = QWidget(); form = QFormLayout(tab)
        self.status_values = {name: QLabel("---") for name in (
            "System / component", "Messages received", "Heartbeat status",
            "Message rates", "Last error")}
        for name, label in self.status_values.items(): form.addRow(name, label)
        return tab

    def _statistics_tab(self) -> QWidget:
        tab = QWidget(); layout = QVBoxLayout(tab)
        form_box = QGroupBox("Current flight session")
        form = QFormLayout(form_box)
        names = (
            "Duration", "Distance travelled", "Maximum airspeed",
            "Maximum ground speed", "Minimum pressure altitude",
            "Maximum pressure altitude", "Minimum ambient temperature",
            "Maximum ambient temperature", "Recorded samples",
        )
        self.statistics_values = {name: QLabel("---") for name in names}
        for name, label in self.statistics_values.items():
            label.setStyleSheet("font-size: 18px; font-weight: 600")
            form.addRow(name, label)
        reset = QPushButton("Reset flight statistics")
        reset.clicked.connect(self.flight_statistics.reset)
        layout.addWidget(form_box); layout.addWidget(reset)
        layout.addStretch(1)
        return tab

    def toggle_recording(self) -> None:
        if self.recorder.active:
            saved_path = self.recorder.stop()
            self.record_button.setText("Start recording")
            self.record_button.setStyleSheet("")
            if saved_path is not None:
                QMessageBox.information(self, "Recording saved", str(saved_path))
            return
        default = f"flight-{datetime.now():%Y%m%d-%H%M%S}.csv"
        path, _ = QFileDialog.getSaveFileName(
            self, "Start CSV recording", default, "CSV files (*.csv)")
        if not path:
            return
        try:
            self.recorder.start(path)
        except OSError as error:
            QMessageBox.critical(self, "Recording error", str(error))
            return
        self.record_button.setText("Stop recording")
        self.record_button.setStyleSheet(
            "background: #b62f3a; color: white; font-weight: bold")

    def toggle_connection(self) -> None:
        if self.receiver is not None:
            self.receiver.stop(); self.receiver = None
            self.connect_button.setText("Connect")
            self.state.connected = False
            return
        self.receiver = MavlinkReceiver(self.endpoint.text().strip(), int(self.baud.currentText()))
        self.receiver.message_received.connect(self.state.ingest)
        self.receiver.connection_error.connect(self.on_error)
        self.receiver.finished.connect(self.on_receiver_finished)
        self.receiver.start()
        self.connect_button.setText("Disconnect")

    def on_error(self, text: str) -> None:
        from datetime import datetime

        self.status_values["Last error"].setText(text)
        local_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        entry = f"[{local_time}] [CONNECTION] {text}"
        if not self.state.status_messages or self.state.status_messages[-1] != entry:
            self.state.status_messages.append(entry)
            del self.state.status_messages[:-20]

    def on_receiver_finished(self) -> None:
        self.receiver = None; self.connect_button.setText("Connect")
        self.state.connected = False

    def refresh(self) -> None:
        import time
        self.state.update_connection()
        connected = self.state.connected
        self.link_label.setText("LIVE" if connected else "NO TELEMETRY")
        self.link_label.setStyleSheet(
            f"color: {'#60d394' if connected else '#ff7066'}; font-weight: bold")
        self.status_lamps.set_level(
            self.state.alert_level() if connected else "red")
        self.attitude.set_attitude(self.state.roll_deg, self.state.pitch_deg)
        self._set_box(self.roll_value, self.state.roll_deg, "°")
        self._set_box(self.pitch_value, self.state.pitch_deg, "°")
        self._set_box(self.heading, self.state.yaw_deg, "°")
        self._set_box(self.airspeed, self.state.airspeed_mps, "m/s")
        self._set_box(self.ground_speed, self.state.ground_speed_mps, "m/s")
        self._set_box(self.pressure_altitude, self.state.altitude_m, "m")
        self._set_box(self.gps_altitude, self.state.gps_altitude_m, "m")
        self._set_box(self.ambient_temperature,
                      self.state.barometer_temperature_c, "°C")
        messages = "\n".join(self.state.status_messages)
        if self.fault_messages.toPlainText() != messages:
            self.fault_messages.setPlainText(messages)
            self.fault_messages.verticalScrollBar().setValue(
                self.fault_messages.verticalScrollBar().maximum())
        self.status_values["System / component"].setText(
            f"{self.state.system_id} / {self.state.component_id}")
        self.status_values["Messages received"].setText(str(self.state.message_count))
        self.status_values["Heartbeat status"].setText(str(self.state.system_status))
        self.status_values["Message rates"].setText("  ".join(
            f"{name}: {rate:.1f} Hz" for name, rate in sorted(self.state.message_rates.items())))
        self._refresh_statistics_display()

        if self.state.message_count == self.last_plotted_message:
            return
        self.last_plotted_message = self.state.message_count
        self.flight_statistics.update(self.state)
        self.calibration.sample(self.state)
        if self.recorder.active:
            try:
                self.recorder.append(self.state)
            except OSError as error:
                self.recorder.stop()
                self.record_button.setText("Start recording")
                QMessageBox.critical(self, "Recording stopped", str(error))
        self._refresh_statistics_display()
        timestamp = time.monotonic()
        self.accel_plot.append(timestamp, self.state.acceleration_mps2)
        self.gyro_plot.append(timestamp, self.state.angular_rate_rps)
        self.mag_plot.append(timestamp, self.state.magnetic_field_gauss)
        self.pressure_plot.append(timestamp, [self.state.pressure_hpa,
                                              self.state.differential_pressure_hpa])
        self.air_plot.append(timestamp, [self.state.altitude_m, self.state.airspeed_mps])
        self.temperature_plot.append(timestamp, [self.state.imu_temperature_c,
                                                  self.state.barometer_temperature_c])
        if self.state.gps_fix_type >= 2 and not math.isnan(self.state.latitude_deg):
            position = (self.state.latitude_deg, self.state.longitude_deg)
            if position != self.last_gps_position:
                self.last_gps_position = position
                self.track_lat.append(position[0])
                self.track_lon.append(position[1])
                self.track_curve.setData(list(self.track_lon),
                                         list(self.track_lat))
                if self.map_view is not None:
                    heading = (self.state.course_deg
                               if not math.isnan(self.state.course_deg)
                               else 0.0)
                    self.map_view.page().runJavaScript(
                        "if (window.updateVehicle) "
                        f"updateVehicle({position[0]:.8f}, "
                        f"{position[1]:.8f}, {heading:.2f});"
                    )
        self.gps_summary.setText(
            f"Lat {self._number(self.state.latitude_deg, 7)}°   "
            f"Lon {self._number(self.state.longitude_deg, 7)}°   "
            f"GPS alt {self._number(self.state.gps_altitude_m, 1)} m   "
            f"Speed {self._number(self.state.ground_speed_mps, 2)} m/s   "
            f"HDOP {self._number(self.state.hdop, 2)}")

    @staticmethod
    def _number(value: float, decimals: int = 1) -> str:
        return "---" if math.isnan(value) else f"{value:.{decimals}f}"

    def _set_box(self, box: QGroupBox, value: float, unit: str) -> None:
        box.findChild(QLabel, "value").setText(f"{self._number(value)} {unit}")

    def _refresh_statistics_display(self) -> None:
        statistics = self.flight_statistics
        duration = int(statistics.duration_s)
        self.statistics_values["Duration"].setText(
            f"{duration // 3600:02d}:{(duration // 60) % 60:02d}:{duration % 60:02d}")
        self.statistics_values["Distance travelled"].setText(
            f"{statistics.distance_m:.1f} m")
        values = {
            "Maximum airspeed": (statistics.maximum_airspeed_mps, "m/s"),
            "Maximum ground speed": (statistics.maximum_ground_speed_mps, "m/s"),
            "Minimum pressure altitude":
                (statistics.minimum_pressure_altitude_m, "m"),
            "Maximum pressure altitude":
                (statistics.maximum_pressure_altitude_m, "m"),
            "Minimum ambient temperature":
                (statistics.minimum_temperature_c, "°C"),
            "Maximum ambient temperature":
                (statistics.maximum_temperature_c, "°C"),
        }
        for name, (value, unit) in values.items():
            self.statistics_values[name].setText(
                f"{self._number(value, 2)} {unit}")
        self.statistics_values["Recorded samples"].setText(
            str(statistics.sample_count))

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API name
        if self.receiver is not None: self.receiver.stop()
        self.recorder.stop()
        event.accept()
