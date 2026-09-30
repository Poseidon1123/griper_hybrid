from __future__ import annotations

import time
from collections import deque
from copy import deepcopy

from drivers.ec_a4310 import ECA4310
from drivers.waveshare_usb_can import WaveshareUsbCanA


class GripperController:
    """Non-GUI state machine for EC-A4310 gripper control."""

    DISCONNECTED = "DISCONNECTED"
    IDLE = "IDLE"
    MOVING = "MOVING"
    CLOSING = "CLOSING"
    HOLD = "HOLD"
    FAULT = "FAULT"

    def __init__(self, config: dict):
        self.cfg = deepcopy(config)
        self.can = None
        self.motor = None
        self.state = self.DISCONNECTED
        self.target_deg = None
        self.last_feedback = None
        self.last_idle_poll = 0.0
        self.history = deque()
        self.contact_confirm_count = 0
        self.close_start_deg = None
        self.close_start_time = None
        self.move_good_count = 0

    @property
    def connected(self) -> bool:
        return self.motor is not None and self.can is not None

    def update_config(self, config: dict) -> None:
        self.cfg = deepcopy(config)

    def connect(self, port: str) -> dict:
        self.disconnect()
        timeout_s = float(self.cfg["usb_can"].get("serial_timeout_s", 0.03))
        self.can = WaveshareUsbCanA(port, timeout=timeout_s)
        self.can.configure_1m_standard_normal()

        configured_id = self.cfg.get("motor", {}).get("can_id")
        motor_id = int(configured_id) if configured_id not in (None, "", 0) else None
        self.motor = ECA4310(self.can, motor_id=motor_id)

        if self.motor.motor_id is None:
            self.motor.discover_id()
        else:
            try:
                self.motor.query_position(timeout=0.35)
            except Exception:
                self.motor.motor_id = None
                self.motor.discover_id()

        qna_ok = self.motor.set_qna_mode()
        pos = self.motor.query_position(timeout=0.6)
        self.state = self.IDLE
        self.last_feedback = {
            "position_deg": pos,
            "current_a": 0.0,
            "temperature_c": None,
            "error_code": 0,
            "error_text": "OK",
        }
        return {
            "motor_id": int(self.motor.motor_id),
            "qna_ok": qna_ok,
            "position_deg": pos,
        }

    def disconnect(self) -> None:
        if self.can is not None:
            try:
                self.can.close()
            except Exception:
                pass
        self.can = None
        self.motor = None
        self.state = self.DISCONNECTED
        self.target_deg = None
        self.history.clear()
        self.contact_confirm_count = 0

    def stop(self) -> None:
        """Stop streaming commands; driver heartbeat stops motion/holding shortly after."""
        if self.connected:
            self.state = self.IDLE
            self.target_deg = None
            self.history.clear()
            self.contact_confirm_count = 0

    def move_absolute(self, target_deg: float) -> None:
        self._require_connected()
        self.target_deg = float(target_deg)
        self.state = self.MOVING
        self.move_good_count = 0

    def move_relative(self, delta_deg: float) -> None:
        self._require_connected()
        pos = self.current_position()
        self.move_absolute(pos + float(delta_deg))

    def open_release(self) -> None:
        direction = int(self.cfg["gripper"]["close_direction"])
        delta = float(self.cfg["gripper"]["open_release_delta_deg"])
        self.move_relative(-direction * abs(delta))

    def start_grip(self) -> None:
        self._require_connected()
        pos = self.current_position()
        direction = int(self.cfg["gripper"]["close_direction"])
        max_travel = float(self.cfg["gripper"]["max_close_travel_deg"])
        self.close_start_deg = pos
        self.close_start_time = time.monotonic()
        self.target_deg = pos + direction * abs(max_travel)
        self.history.clear()
        self.contact_confirm_count = 0
        self.state = self.CLOSING

    def current_position(self) -> float:
        if self.last_feedback and self.last_feedback.get("position_deg") is not None:
            return float(self.last_feedback["position_deg"])
        self._require_connected()
        return float(self.motor.query_position(timeout=0.5))

    def step(self):
        """Run one controller iteration from one worker thread."""
        if not self.connected:
            return None

        now = time.monotonic()

        if self.state == self.IDLE:
            if now - self.last_idle_poll >= 0.5:
                try:
                    pos = self.motor.query_position(timeout=0.15)
                    if self.last_feedback is None:
                        self.last_feedback = {}
                    self.last_feedback["position_deg"] = pos
                    self.last_feedback.setdefault("current_a", 0.0)
                    self.last_feedback.setdefault("temperature_c", None)
                    self.last_feedback.setdefault("error_code", 0)
                    self.last_feedback.setdefault("error_text", "OK")
                    self.last_idle_poll = now
                except TimeoutError:
                    pass
            return self.telemetry()

        if self.state == self.FAULT:
            return self.telemetry()

        if self.target_deg is None:
            self.state = self.IDLE
            return self.telemetry()

        if self.state == self.HOLD:
            current_limit = float(self.cfg["grip"]["i_hold_a"])
            speed_rpm = float(self.cfg["grip"]["close_speed_rpm"])
        elif self.state == self.CLOSING:
            current_limit = float(self.cfg["grip"]["i_close_a"])
            speed_rpm = float(self.cfg["grip"]["close_speed_rpm"])
        else:
            current_limit = float(self.cfg["motion"]["current_limit_a"])
            speed_rpm = float(self.cfg["motion"]["speed_rpm"])

        self._validate_current(current_limit)
        self.motor.send_position(
            target_deg=self.target_deg,
            speed_rpm=speed_rpm,
            current_limit_a=current_limit,
            ack=2,
        )
        fb = self.motor.read_feedback_type2(timeout=0.03)

        if fb is not None:
            self.last_feedback = fb
            self._check_feedback_safety(fb)

            if self.state == self.CLOSING:
                self._update_contact_detection(now, fb)
            elif self.state == self.MOVING:
                tol = float(self.cfg["motion"]["position_tolerance_deg"])
                if abs(self.target_deg - fb["position_deg"]) <= tol:
                    self.move_good_count += 1
                else:
                    self.move_good_count = 0
                if self.move_good_count >= 3:
                    self.state = self.IDLE
                    self.target_deg = None

        if self.state == self.CLOSING:
            timeout_s = float(self.cfg["safety"]["close_timeout_s"])
            if self.close_start_time and now - self.close_start_time > timeout_s:
                self.state = self.FAULT
                raise TimeoutError(f"Grip close timeout > {timeout_s:.1f} s")

        return self.telemetry()

    def telemetry(self) -> dict:
        fb = self.last_feedback or {}
        return {
            "state": self.state,
            "target_deg": self.target_deg,
            "position_deg": fb.get("position_deg"),
            "current_a": fb.get("current_a"),
            "temperature_c": fb.get("temperature_c"),
            "error_code": fb.get("error_code", 0),
            "error_text": fb.get("error_text", "OK"),
        }

    def _update_contact_detection(self, now: float, fb: dict) -> None:
        pos = float(fb["position_deg"])
        current_a = abs(float(fb["current_a"]))
        self.history.append((now, pos))

        window_s = float(self.cfg["grip"]["stall_window_s"])
        while self.history and now - self.history[0][0] > window_s:
            self.history.popleft()

        stalled = False
        if len(self.history) >= 2:
            dt = self.history[-1][0] - self.history[0][0]
            move = abs(self.history[-1][1] - self.history[0][1])
            stalled = (
                dt >= window_s * 0.70
                and move <= float(self.cfg["grip"]["stall_move_deg"])
            )

        traveled = abs(pos - float(self.close_start_deg))
        elapsed = now - float(self.close_start_time)
        candidate = (
            current_a >= float(self.cfg["grip"]["contact_current_a"])
            and stalled
            and traveled >= float(self.cfg["grip"]["min_travel_before_contact_deg"])
            and elapsed >= float(self.cfg["grip"]["min_time_before_contact_s"])
        )

        if candidate:
            self.contact_confirm_count += 1
        else:
            self.contact_confirm_count = 0

        required = int(self.cfg["grip"]["contact_confirm_count"])
        if self.contact_confirm_count >= required:
            direction = int(self.cfg["gripper"]["close_direction"])
            preload = abs(float(self.cfg["grip"]["preload_deg"]))
            self.target_deg = pos + direction * preload
            self.state = self.HOLD
            self.history.clear()
            self.contact_confirm_count = 0

    def _check_feedback_safety(self, fb: dict) -> None:
        if int(fb.get("error_code", 0)) != 0:
            self.state = self.FAULT
            raise RuntimeError(
                f"Motor fault {fb['error_code']}: {fb.get('error_text', 'Unknown')}"
            )
        temp = fb.get("temperature_c")
        if temp is not None and temp >= float(self.cfg["safety"]["max_motor_temp_c"]):
            self.state = self.FAULT
            raise RuntimeError(f"Motor over temperature: {temp:.1f} C")

    def _validate_current(self, value: float) -> None:
        maximum = float(self.cfg["safety"]["absolute_current_limit_a"])
        if value < 0 or value > maximum:
            self.state = self.FAULT
            raise ValueError(
                f"Current limit {value:.2f} A exceeds configured safety limit {maximum:.2f} A"
            )

    def _require_connected(self) -> None:
        if not self.connected:
            raise RuntimeError("USB-CAN / motor is not connected")
