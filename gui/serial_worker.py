import queue
import time

import serial
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot


class SerialWorker(QObject):
    connected = pyqtSignal(str)
    disconnected = pyqtSignal()
    line_received = pyqtSignal(str)
    error = pyqtSignal(str)

    def __init__(self, port: str, baudrate: int, timeout_s: float = 0.05):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self._serial = None
        self._running = False
        self._tx_queue = queue.Queue()

    @pyqtSlot()
    def run(self):
        try:
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout_s,
                write_timeout=0.1,
            )
            self._running = True
            self.connected.emit(self.port)

            while self._running:
                self._flush_tx_queue()
                self._read_line_if_available()
                time.sleep(0.001)

        except Exception as exc:
            self.error.emit(str(exc))
        finally:
            self._close_serial()
            self.disconnected.emit()

    def _flush_tx_queue(self):
        while not self._tx_queue.empty() and self._serial is not None:
            command = self._tx_queue.get_nowait()
            payload = (command.rstrip("\r\n") + "\n").encode("utf-8")
            self._serial.write(payload)

    def _read_line_if_available(self):
        if self._serial is None or self._serial.in_waiting <= 0:
            return

        raw = self._serial.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                self.line_received.emit(line)

    @pyqtSlot(str)
    def send_command(self, command: str):
        if self._running:
            self._tx_queue.put(command)
        else:
            self.error.emit("Serial port is not connected")

    @pyqtSlot()
    def stop(self):
        self._running = False

    def _close_serial(self):
        if self._serial is not None:
            try:
                if self._serial.is_open:
                    self._serial.close()
            finally:
                self._serial = None
