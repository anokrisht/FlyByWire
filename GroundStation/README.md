# MAVLink sensor ground station

This desktop application displays telemetry from any MAVLink-compatible flight
controller over a serial port or UDP. It is not tied to the STM32 firmware.

## Screenshots

### Flight instruments

![FlyByWire flight instruments](<../.images/Dashboard - flight instruments .png>)

### GPS map

![FlyByWire GPS map](<../.images/Dashboard - GPS maps.png>)

## Install and run

Python 3.11 or 3.12 is recommended. From the repository root on Windows:

```powershell
py -m venv GroundStation/.venv
GroundStation/.venv/Scripts/Activate.ps1
python -m pip install -r GroundStation/requirements.txt
python GroundStation/main.py --endpoint COM4 --baud 115200
```

Linux serial example:

```sh
python GroundStation/main.py --endpoint /dev/ttyUSB0 --baud 115200
```

The endpoint can also be a pymavlink network connection such as
`udpin:0.0.0.0:14550`. The toolbar can change and reconnect the endpoint while
the application is running.

## Test without a flight controller

Start the dashboard with its default UDP endpoint, then run the simulator in a
second terminal:

```powershell
python GroundStation/main.py
python GroundStation/simulator.py
```

The simulator traces a moving attitude, air data, 9-axis IMU values, and a
circular GPS ground track.

## Recording and analysis

Use **Start recording** in the header to choose a CSV file. The button turns
red while recording; **Stop recording** closes and saves the file. Each row
contains computer UTC, link/alert state, attitude, air data, both temperatures,
9-axis IMU values, and GPS values. The file is flushed continuously so most
data remains usable if the program closes unexpectedly.

The **Flight statistics** tab tracks session duration, GPS distance travelled,
maximum airspeed and ground speed, pressure-altitude range, ambient-temperature
range, and sample count. Statistics can be reset independently of recording.

The **Calibration** tab provides a five-second stationary accelerometer, gyro,
and differential-pressure-zero measurement plus a rotate-through-all-axes
magnetometer calibration. Results can be saved as JSON. This assistant computes
calibration values but does not yet write them back to the microcontroller.

## MAVLink contract for a microcontroller

Transmit MAVLink 2 using the standard `common.xml` dialect. No custom messages
are required. The dashboard consumes:


| Message | Recommended rate | Sensor data |
| --- | ---: | --- |
| `HEARTBEAT` | 1 Hz | Link, vehicle, and system state |
| `HIGHRES_IMU` | 20–100 Hz | Acceleration, rates, magnetic field, pressure, differential pressure, temperature |
| `ATTITUDE` | 20–50 Hz | Roll, pitch, yaw and angular rates |
| `SCALED_PRESSURE` | 10–25 Hz | Barometer, differential pressure and temperature |
| `GPS_RAW_INT` | GPS update rate | Raw fix, coordinates, altitude, speed, course, DOP and satellites |
| `VFR_HUD` | 10–20 Hz | Airspeed, ground speed, heading and pressure altitude |
| `STATUSTEXT` | On change; repeat active faults every 5 s | Sensor faults, warnings, recoveries and custom status |

The dashboard header has three health lamps: green means the link is live with
no active fault, yellow means at least one warning is active, and red means a
critical/error report or loss of telemetry. An informational recovery message
from the same sensor clears its active warning or fault.

After the first valid GPS date/time, firmware messages use ISO UTC timestamps,
for example `2026-08-19T14:32:08.514Z`. The STM32 advances this clock using its
monotonic tick if GPS is lost and resynchronizes when new GPS time arrives.
Before the first GPS synchronization it falls back to `T+HH:MM:SS.mmm` uptime.
UTC does not change with geographic time zones; local time and daylight-saving
conversion belong on the computer. Computer-side connection errors use the
computer's local time.

Use the units defined by MAVLink exactly. In particular, `HIGHRES_IMU` uses
SI acceleration and angular-rate units, `SCALED_PRESSURE` uses hPa and
centidegrees Celsius, and `GPS_RAW_INT` uses scaled integers. Every packet must
use a stable system/component ID and sequence number. Send `HEARTBEAT` even if a
sensor is unavailable, and omit or mark invalid only the affected measurement.

QGroundControl can consume the same stream. A serial port normally has only one
reader, so use a MAVLink router when QGroundControl and this station must run at
the same time.
