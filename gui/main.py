import sys
from pathlib import Path

import yaml
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QVBoxLayout,
    QWidget,
)

from serial_worker import SerialWorker
from udp_server import UDPServer


ROOT_DIR = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT_DIR / "config" / "gripper_config.yaml"


class GripperWindow(QMainWindow):
    udp_command_received = pyqtSignal(str, object)

    def __init__(self):
        super().__init__()
        self.config = self._load_config()
        self.serial_thread = None
        self.serial_worker = None
        self.udp_server = None

        self.setWindowTitle("Hybrid Gripper Control")
        self.resize(720, 520)
        self._build_ui()
        self._set_connected(False)
        self.udp_command_received.connect(self._handle_udp_command)

        if self.config.get("udp", {}).get("enable", False):
            self._start_udp()

    def _load_config(self):
        with CONFIG_PATH.open("r", encoding="utf-8") as file:
            return yaml.safe_load(file)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        connection_box = QGroupBox("Connection")
        connection_layout = QGridLayout(connection_box)

        serial_cfg = self.config["serial"]
        self.connection_label = QLabel("Disconnected")
        self.serial_info_label = QLabel(
            f"{serial_cfg['port']} @ {serial_cfg['baudrate']} baud"
        )
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)

        connection_layout.addWidget(QLabel("Serial:"), 0, 0)
        connection_layout.addWidget(self.serial_info_label, 0, 1)
        connection_layout.addWidget(QLabel("Status:"), 1, 0)
        connection_layout.addWidget(self.connection_label, 1, 1)
        connection_layout.addWidget(self.connect_button, 0, 2, 2, 1)
        root.addWidget(connection_box)

        telemetry_box = QGroupBox("Telemetry")
        telemetry_layout = QGridLayout(telemetry_box)
        self.width_label = QLabel("-- mm")
        self.position_label = QLabel("--")
        self.velocity_label = QLabel("--")
        self.torque_label = QLabel("--")
        self.temperature_label = QLabel("--")
        self.state_label = QLabel("UNKNOWN")

        labels = [
            ("Gripper width", self.width_label),
            ("Motor position", self.position_label),
            ("Motor velocity", self.velocity_label),
            ("Motor torque", self.torque_label),
            ("Temperature", self.temperature_label),
            ("State", self.state_label),
        ]
        for row, (name, value) in enumerate(labels):
            telemetry_layout.addWidget(QLabel(name + ":"), row, 0)
            telemetry_layout.addWidget(value, row, 1)
        root.addWidget(telemetry_box)

        control_box = QGroupBox("Gripper control")
        control_layout = QVBoxLayout(control_box)

        button_row = QHBoxLayout()
        self.enable_button = QPushButton("ENABLE")
        self.disable_button = QPushButton("DISABLE")
        self.open_button = QPushButton("OPEN")
        self.close_button = QPushButton("CLOSE")
        self.stop_button = QPushButton("STOP")

        self.enable_button.clicked.connect(lambda: self.send_command("ENABLE"))
        self.disable_button.clicked.connect(lambda: self.send_command("DISABLE"))
        self.open_button.clicked.connect(lambda: self.send_command("OPEN"))
        self.close_button.clicked.connect(lambda: self.send_command("CLOSE"))
        self.stop_button.clicked.connect(lambda: self.send_command("STOP"))

        for button in (
            self.enable_button,
            self.disable_button,
            self.open_button,
            self.close_button,
            self.stop_button,
        ):
            button_row.addWidget(button)
        control_layout.addLayout(button_row)

        width_row = QHBoxLayout()
        width_row.addWidget(QLabel("Target width:"))
        self.width_spin = QDoubleSpinBox()
        gripper_cfg = self.config["gripper"]
        self.width_spin.setRange(
            gripper_cfg["width_min_mm"], gripper_cfg["width_max_mm"]
        )
        self.width_spin.setValue(gripper_cfg["default_width_mm"])
        self.width_spin.setSuffix(" mm")
        self.width_spin.setDecimals(1)
        self.set_width_button = QPushButton("SET WIDTH")
        self.set_width_button.clicked.connect(self.set_target_width)
        width_row.addWidget(self.width_spin)
        width_row.addWidget(self.set_width_button)
        control_layout.addLayout(width_row)
        root.addWidget(control_box)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        root.addWidget(self.log)

    def toggle_connection(self):
        if self.serial_worker is None:
            self._connect_serial()
        else:
            self._disconnect_serial()

    def _connect_serial(self):
        cfg = self.config["serial"]
        self.serial_thread = QThread(self)
        self.serial_worker = SerialWorker(
            cfg["port"], int(cfg["baudrate"]), float(cfg["timeout_s"])
        )
        self.serial_worker.moveToThread(self.serial_thread)
        self.serial_thread.started.connect(self.serial_worker.run)
        self.serial_worker.connected.connect(self._on_connected)
        self.serial_worker.disconnected.connect(self._on_disconnected)
        self.serial_worker.line_received.connect(self._handle_serial_line)
        self.serial_worker.error.connect(self._on_serial_error)
        self.serial_thread.start()

    def _disconnect_serial(self):
        if self.serial_worker is not None:
            self.serial_worker.stop()

    def _on_connected(self, port):
        self._set_connected(True)
        self._append_log(f"Connected to {port}")

    def _on_disconnected(self):
        self._append_log("Serial disconnected")
        self._set_connected(False)
        if self.serial_thread is not None:
            self.serial_thread.quit()
            self.serial_thread.wait(500)
            self.serial_thread.deleteLater()
        self.serial_thread = None
        self.serial_worker = None

    def _on_serial_error(self, message):
        self._append_log(f"SERIAL ERROR: {message}")

    def _set_connected(self, connected):
        self.connection_label.setText("Connected" if connected else "Disconnected")
        self.connect_button.setText("Disconnect" if connected else "Connect")

        for button in (
            self.enable_button,
            self.disable_button,
            self.open_button,
            self.close_button,
            self.stop_button,
            self.set_width_button,
        ):
            button.setEnabled(connected)

    def set_target_width(self):
        width_mm = self.width_spin.value()
        self.send_command(f"SET_WIDTH {width_mm:.1f}")

    def send_command(self, command):
        if self.serial_worker is None:
            self._append_log("Not connected: " + command)
            return
        self.serial_worker.send_command(command)
        self._append_log("TX: " + command)

    def _handle_serial_line(self, line):
        self._append_log("RX: " + line)

        # Expected examples:
        # WIDTH,42.5
        # POS,1.234
        # VEL,0.25
        # TORQUE,2.1
        # TEMP,38.0
        # STATE,READY
        if "," not in line:
            return

        key, value = line.split(",", 1)
        key = key.strip().upper()
        value = value.strip()

        mapping = {
            "WIDTH": (self.width_label, " mm"),
            "POS": (self.position_label, ""),
            "VEL": (self.velocity_label, ""),
            "TORQUE": (self.torque_label, " Nm"),
            "TEMP": (self.temperature_label, " °C"),
            "STATE": (self.state_label, ""),
        }
        if key in mapping:
            widget, suffix = mapping[key]
            widget.setText(value + suffix)

    def _start_udp(self):
        cfg = self.config["udp"]
        self.udp_server = UDPServer(
            cfg["bind_ip"], int(cfg["port"]), self._on_udp_datagram
        )
        try:
            self.udp_server.start()
            self._append_log(
                f"UDP listening on {cfg['bind_ip']}:{cfg['port']}"
            )
        except OSError as exc:
            self._append_log(f"UDP ERROR: {exc}")

    def _on_udp_datagram(self, command, address):
        # This callback runs in the UDP worker thread. Forward work to Qt thread.
        self.udp_command_received.emit(command, address)

    def _handle_udp_command(self, command, address):
        # UDP is disabled by default. Command format will be finalized later.
        self._append_log(f"UDP {address}: {command}")
        self.send_command(command)

    def _append_log(self, text):
        self.log.appendPlainText(text)

    def closeEvent(self, event):
        if self.udp_server is not None:
            self.udp_server.stop()
        if self.serial_worker is not None:
            self.serial_worker.stop()
        if self.serial_thread is not None:
            self.serial_thread.quit()
            self.serial_thread.wait(1000)
        event.accept()


def main():
    app = QApplication(sys.argv)
    try:
        window = GripperWindow()
    except Exception as exc:
        QMessageBox.critical(None, "Startup error", str(exc))
        return 1

    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
