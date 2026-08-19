"""MAVLink receiver worker; UI code never blocks on a serial port."""

from __future__ import annotations

from PySide6.QtCore import QThread, Signal


class MavlinkReceiver(QThread):
    message_received = Signal(object)
    connection_error = Signal(str)
    link_opened = Signal()

    def __init__(self, endpoint: str, baud: int) -> None:
        super().__init__()
        self.endpoint = endpoint
        self.baud = baud
        self._link = None

    def run(self) -> None:
        try:
            from pymavlink import mavutil

            self._link = mavutil.mavlink_connection(
                self.endpoint,
                baud=self.baud,
                dialect="common",
                autoreconnect=True,
                robust_parsing=True,
            )
            self.link_opened.emit()
            while not self.isInterruptionRequested():
                message = self._link.recv_match(blocking=True, timeout=0.25)
                if message is not None:
                    self.message_received.emit(message)
        except Exception as error:  # Report serial/network failures in the UI.
            self.connection_error.emit(str(error))
        finally:
            if self._link is not None:
                self._link.close()
                self._link = None

    def stop(self) -> None:
        self.requestInterruption()
        self.wait(1500)
