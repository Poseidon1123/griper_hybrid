from __future__ import annotations

import queue
import sys
import time
from copy import deepcopy
from pathlib import Path

import yaml
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
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

from controller import GripperController
from drivers import available_ports

ROOT_DIR = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT_DIR / "config" / "gripper_config.yaml"


class ControlWorker(QObject):
    telemetry = pyqtSignal(dict)
    log = pyqtSignal(str)
    connection = pyqtSignal(bool, str, str)
    finished = pyqtSignal()

    def __init__(self, config: dict):
        super().__init__()
        self.config = deepcopy(config)
        self.controller = GripperController(self.config)
        self.commands = queue.Queue()
        self.running = True

    def enqueue(self, name: str, payload=None):
        self.commands.put((name, payload))

    def run(self):
        try:
            while self.running:
                loop_t = time.monotonic()
                self._process_commands()

                if self.controller.connected:
                    try:
                        data = self.controller.step()
                        if data:
                            self.telemetry.emit(data)
                    except Exception as exc:
                        self.log.emit(f"CONTROL ERROR: {exc}")
                        try:
                            self.controller.stop()
                        except Exception:
                            pass
                        self.telemetry.emit(self.controller.telemetry())

                hz = float(self.config.get("controller", {}).get("loop_hz", 20.0))
                period = 1.0 / max(1.0, hz)
                dt = time.monotonic() - loop_t
                if dt < period:
                    time.sleep(period - dt)
        finally:
            self.controller.disconnect()
            self.finished.emit()

    def _process_commands(self):
        while True:
            try:
                name, payload = self.commands.get_nowait()
            except queue.Empty:
                return

            try:
                if name == "connect":
                    info = self.controller.connect(str(payload))
                    self.connection.emit(True, str(payload), f"0x{info['motor_id']:03X}")
                    self.log.emit(
                        f"Connected {payload} | motor ID=0x{info['motor_id']:03X} | "
                        f"position={info['position_deg']:.3f} deg | Q&A={info['qna_ok']}"
                    )
                elif name == "disconnect":
                    self.controller.disconnect()
                    self.connection.emit(False, "", "")
                    self.log.emit("Disconnected")
                elif name == "grip":
                    self.controller.start_grip()
                    self.log.emit("GRIP: closing until object contact")
                elif name == "open":
                    self.controller.open_release()
                    self.log.emit("OPEN/RELEASE command")
                elif name == "stop":
                    self.controller.stop()
                    self.log.emit("STOP: command stream stopped")
                elif name == "move_abs":
                    self.controller.move_absolute(float(payload))
                    self.log.emit(f"Move absolute -> {float(payload):.3f} deg")
                elif name == "move_rel":
                    self.controller.move_relative(float(payload))
                    self.log.emit(f"Move relative -> {float(payload):+.3f} deg")
                elif name == "config":
                    self.config = deepcopy(payload)
                    self.controller.update_config(self.config)
                    self.log.emit("Runtime configuration updated")
                elif name == "shutdown":
                    self.running = False
                    return
            except Exception as exc:
                self.log.emit(f"COMMAND ERROR [{name}]: {exc}")
                if name == "connect":
                    self.connection.emit(False, "", "")
                    self.controller.disconnect()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.config = self.load_config()
        self.last_state = "DISCONNECTED"

        self.setWindowTitle("EC-A4310 Hybrid Gripper HMI")
        self.resize(940, 720)
        self.build_ui()

        self.thread = QThread(self)
        self.worker = ControlWorker(self.config)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.telemetry.connect(self.on_telemetry)
        self.worker.log.connect(self.append_log)
        self.worker.connection.connect(self.on_connection)
        self.worker.finished.connect(self.thread.quit)
        self.thread.start()

        self.refresh_ports()
        self.set_controls_connected(False)

    @staticmethod
    def load_config():
        with CONFIG_PATH.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    def save_config(self):
        with CONFIG_PATH.open("w", encoding="utf-8") as f:
            yaml.safe_dump(self.config, f, sort_keys=False, allow_unicode=True)

    def build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        conn = QGroupBox("USB-CAN-A connection")
        layout = QGridLayout(conn)
        self.port_combo = QComboBox()
        self.refresh_btn = QPushButton("Refresh COM")
        self.connect_btn = QPushButton("Connect")
        self.connection_label = QLabel("DISCONNECTED")
        self.motor_id_label = QLabel("--")

        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.toggle_connection)

        layout.addWidget(QLabel("COM port"), 0, 0)
        layout.addWidget(self.port_combo, 0, 1)
        layout.addWidget(self.refresh_btn, 0, 2)
        layout.addWidget(self.connect_btn, 0, 3)
        layout.addWidget(QLabel("Connection"), 1, 0)
        layout.addWidget(self.connection_label, 1, 1)
        layout.addWidget(QLabel("Motor ID"), 1, 2)
        layout.addWidget(self.motor_id_label, 1, 3)
        root.addWidget(conn)

        tele = QGroupBox("Motor feedback")
        tgrid = QGridLayout(tele)
        self.state_value = QLabel("DISCONNECTED")
        self.position_value = QLabel("-- deg")
        self.target_value = QLabel("-- deg")
        self.current_value = QLabel("-- A")
        self.temp_value = QLabel("-- °C")
        self.error_value = QLabel("--")
        values = [
            ("State", self.state_value),
            ("Actual position", self.position_value),
            ("Target position", self.target_value),
            ("Actual current", self.current_value),
            ("Motor temperature", self.temp_value),
            ("Motor fault", self.error_value),
        ]
        for i, (name, widget) in enumerate(values):
            r, c = divmod(i, 2)
            tgrid.addWidget(QLabel(name + ":"), r, c * 2)
            tgrid.addWidget(widget, r, c * 2 + 1)
        root.addWidget(tele)

        control = QGroupBox("Gripper control")
        croot = QVBoxLayout(control)
        row = QHBoxLayout()
        self.grip_btn = QPushButton("GRIP / AUTO HOLD")
        self.open_btn = QPushButton("OPEN / RELEASE")
        self.stop_btn = QPushButton("STOP")
        self.grip_btn.clicked.connect(lambda: self.worker.enqueue("grip"))
        self.open_btn.clicked.connect(lambda: self.worker.enqueue("open"))
        self.stop_btn.clicked.connect(self.stop_clicked)
        row.addWidget(self.grip_btn)
        row.addWidget(self.open_btn)
        row.addWidget(self.stop_btn)
        croot.addLayout(row)

        manual = QHBoxLayout()
        self.jog_spin = QDoubleSpinBox()
        self.jog_spin.setRange(0.1, 90.0)
        self.jog_spin.setValue(2.0)
        self.jog_spin.setSuffix(" deg")
        self.jog_minus_btn = QPushButton("Jog -")
        self.jog_plus_btn = QPushButton("Jog +")
        self.jog_minus_btn.clicked.connect(
            lambda: self.worker.enqueue("move_rel", -self.jog_spin.value())
        )
        self.jog_plus_btn.clicked.connect(
            lambda: self.worker.enqueue("move_rel", self.jog_spin.value())
        )
        manual.addWidget(QLabel("Jog"))
        manual.addWidget(self.jog_spin)
        manual.addWidget(self.jog_minus_btn)
        manual.addWidget(self.jog_plus_btn)

        self.abs_spin = QDoubleSpinBox()
        self.abs_spin.setRange(-10000.0, 10000.0)
        self.abs_spin.setDecimals(3)
        self.abs_spin.setSuffix(" deg")
        self.abs_btn = QPushButton("Go absolute")
        self.abs_btn.clicked.connect(
            lambda: self.worker.enqueue("move_abs", self.abs_spin.value())
        )
        manual.addSpacing(20)
        manual.addWidget(QLabel("Absolute"))
        manual.addWidget(self.abs_spin)
        manual.addWidget(self.abs_btn)
        croot.addLayout(manual)
        root.addWidget(control)

        params = QGroupBox("Grip parameters")
        form = QFormLayout(params)

        self.close_direction = QComboBox()
        self.close_direction.addItem("+1 (increase angle = close)", 1)
        self.close_direction.addItem("-1 (decrease angle = close)", -1)
        idx = self.close_direction.findData(int(self.config["gripper"]["close_direction"]))
        if idx >= 0:
            self.close_direction.setCurrentIndex(idx)

        self.i_close = self.make_spin(0.0, 3.0, self.config["grip"]["i_close_a"], " A", 2)
        self.i_hold = self.make_spin(0.0, 3.0, self.config["grip"]["i_hold_a"], " A", 2)
        self.i_contact = self.make_spin(0.0, 3.0, self.config["grip"]["contact_current_a"], " A", 2)
        self.close_speed = self.make_spin(0.1, 20.0, self.config["grip"]["close_speed_rpm"], " rpm", 2)
        self.preload = self.make_spin(0.0, 20.0, self.config["grip"]["preload_deg"], " deg", 2)
        self.max_travel = self.make_spin(1.0, 360.0, self.config["gripper"]["max_close_travel_deg"], " deg", 1)
        self.open_delta = self.make_spin(0.5, 180.0, self.config["gripper"]["open_release_delta_deg"], " deg", 1)
        self.motion_current = self.make_spin(0.0, 3.0, self.config["motion"]["current_limit_a"], " A", 2)
        self.motion_speed = self.make_spin(0.1, 20.0, self.config["motion"]["speed_rpm"], " rpm", 2)
        self.apply_btn = QPushButton("Apply + Save YAML")
        self.apply_btn.clicked.connect(self.apply_parameters)

        form.addRow("Close direction", self.close_direction)
        form.addRow("I close", self.i_close)
        form.addRow("I hold", self.i_hold)
        form.addRow("Contact current", self.i_contact)
        form.addRow("Close speed", self.close_speed)
        form.addRow("Preload after contact", self.preload)
        form.addRow("Max close travel", self.max_travel)
        form.addRow("Open/release delta", self.open_delta)
        form.addRow("Manual move current", self.motion_current)
        form.addRow("Manual move speed", self.motion_speed)
        form.addRow(self.apply_btn)
        root.addWidget(params)

        self.log_box = QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(1000)
        root.addWidget(self.log_box, 1)

    @staticmethod
    def make_spin(minimum, maximum, value, suffix, decimals):
        w = QDoubleSpinBox()
        w.setRange(minimum, maximum)
        w.setValue(float(value))
        w.setDecimals(decimals)
        w.setSuffix(suffix)
        return w

    def refresh_ports(self):
        current = self.port_combo.currentData()
        self.port_combo.clear()
        ports = available_ports()
        for p in ports:
            self.port_combo.addItem(f"{p.device} — {p.description}", p.device)
        if current:
            idx = self.port_combo.findData(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)
        self.append_log(f"Found {len(ports)} serial port(s)")

    def toggle_connection(self):
        if self.connect_btn.text() == "Connect":
            port = self.port_combo.currentData()
            if not port:
                QMessageBox.warning(self, "No COM", "Không tìm thấy cổng COM để kết nối.")
                return
            self.connect_btn.setEnabled(False)
            self.worker.enqueue("connect", port)
        else:
            self.worker.enqueue("disconnect")

    def on_connection(self, connected: bool, port: str, motor_id: str):
        self.connect_btn.setEnabled(True)
        self.connect_btn.setText("Disconnect" if connected else "Connect")
        self.connection_label.setText(f"CONNECTED {port}" if connected else "DISCONNECTED")
        self.set_controls_connected(connected)
        self.motor_id_label.setText(motor_id if connected else "--")
        if not connected:
            self.state_value.setText("DISCONNECTED")

    def set_controls_connected(self, enabled: bool):
        for w in (
            self.grip_btn,
            self.open_btn,
            self.stop_btn,
            self.jog_minus_btn,
            self.jog_plus_btn,
            self.abs_btn,
        ):
            w.setEnabled(enabled)

    def on_telemetry(self, data: dict):
        state = data.get("state", "--")
        self.last_state = state
        self.state_value.setText(state)

        pos = data.get("position_deg")
        self.position_value.setText("-- deg" if pos is None else f"{pos:.3f} deg")
        target = data.get("target_deg")
        self.target_value.setText("-- deg" if target is None else f"{target:.3f} deg")
        current = data.get("current_a")
        self.current_value.setText("-- A" if current is None else f"{current:+.2f} A")
        temp = data.get("temperature_c")
        self.temp_value.setText("-- °C" if temp is None else f"{temp:.1f} °C")
        err = int(data.get("error_code", 0))
        self.error_value.setText(f"{err}: {data.get('error_text', 'OK')}")

        if state == "HOLD":
            self.grip_btn.setText("HOLDING OBJECT")
        else:
            self.grip_btn.setText("GRIP / AUTO HOLD")

    def stop_clicked(self):
        if self.last_state == "HOLD":
            answer = QMessageBox.warning(
                self,
                "Release hold?",
                "STOP sẽ ngừng gửi heartbeat giữ vật; vật có thể rơi. Tiếp tục?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
        self.worker.enqueue("stop")

    def apply_parameters(self):
        cfg = deepcopy(self.config)
        cfg["gripper"]["close_direction"] = int(self.close_direction.currentData())
        cfg["gripper"]["max_close_travel_deg"] = self.max_travel.value()
        cfg["gripper"]["open_release_delta_deg"] = self.open_delta.value()
        cfg["grip"]["i_close_a"] = self.i_close.value()
        cfg["grip"]["i_hold_a"] = self.i_hold.value()
        cfg["grip"]["contact_current_a"] = self.i_contact.value()
        cfg["grip"]["close_speed_rpm"] = self.close_speed.value()
        cfg["grip"]["preload_deg"] = self.preload.value()
        cfg["motion"]["current_limit_a"] = self.motion_current.value()
        cfg["motion"]["speed_rpm"] = self.motion_speed.value()

        absolute_limit = float(cfg["safety"]["absolute_current_limit_a"])
        for name, value in (
            ("I close", cfg["grip"]["i_close_a"]),
            ("I hold", cfg["grip"]["i_hold_a"]),
            ("Contact current", cfg["grip"]["contact_current_a"]),
            ("Manual current", cfg["motion"]["current_limit_a"]),
        ):
            if value > absolute_limit:
                QMessageBox.critical(
                    self,
                    "Current limit",
                    f"{name}={value:.2f} A vượt safety limit {absolute_limit:.2f} A",
                )
                return

        self.config = cfg
        self.save_config()
        self.worker.enqueue("config", deepcopy(cfg))
        self.append_log("Saved config/gripper_config.yaml")

    def append_log(self, text: str):
        stamp = time.strftime("%H:%M:%S")
        self.log_box.appendPlainText(f"[{stamp}] {text}")

    def closeEvent(self, event):
        if self.last_state == "HOLD":
            answer = QMessageBox.warning(
                self,
                "Close HMI?",
                "HMI đang HOLD. Thoát sẽ ngừng giữ vật. Tiếp tục?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                event.ignore()
                return
        self.worker.enqueue("shutdown")
        self.thread.quit()
        self.thread.wait(1500)
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
