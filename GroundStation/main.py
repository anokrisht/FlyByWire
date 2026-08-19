"""FlyByWire MAVLink sensor ground station entry point."""

from __future__ import annotations

import argparse
import sys

from PySide6.QtWidgets import QApplication

from dashboard import Dashboard


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="udpin:0.0.0.0:14550",
                        help="Serial device or pymavlink network endpoint")
    parser.add_argument("--baud", default=115200, type=int,
                        help="Serial baud rate (ignored for UDP)")
    args = parser.parse_args()
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = Dashboard(args.endpoint, args.baud)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
