from __future__ import annotations

import struct
import time
from typing import Optional

from .waveshare_usb_can import CanFrame, WaveshareUsbCanA

ERROR_TEXT = {
    0: "OK",
    1: "Motor over-temperature",
    2: "Motor over-current",
    3: "Motor under-voltage",
    4: "Encoder error",
    6: "Brake voltage too high",
    7: "DRV driver error",
}


class ECA4310:
    def __init__(self, can: WaveshareUsbCanA, motor_id: Optional[int] = None):
        self.can = can
        self.motor_id = motor_id

    def discover_id(self, timeout: float = 1.0) -> int:
        self.can.ser.reset_input_buffer()
        self.can.send_can(0x7FF, [0xFF, 0xFF, 0x00, 0x82])
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            fr = self.can.recv_can(0.10)
            if fr is None or fr.can_id != 0x7FF:
                continue
            d = fr.data
            if len(d) >= 5 and d[0] == 0xFF and d[1] == 0xFF and d[2] == 0x01:
                motor_id = (d[3] << 8) | d[4]
                if 1 <= motor_id <= 0x7FE:
                    self.motor_id = motor_id
                    return motor_id
        raise TimeoutError("No valid EC-A4310 motor-ID response")

    def set_qna_mode(self) -> bool:
        self._require_id()
        mid = int(self.motor_id)
        payload = [(mid >> 8) & 0xFF, mid & 0xFF, 0x00, 0x02]
        self.can.ser.reset_input_buffer()
        self.can.send_can(0x7FF, payload)
        fr = self.can.wait_for_id(0x7FF, 0.4)
        time.sleep(0.55)
        return bool(
            fr is not None
            and len(fr.data) >= 4
            and fr.data[2] == 0x01
            and fr.data[3] == 0x02
        )

    def query_position(self, timeout: float = 0.5) -> float:
        self._require_id()
        mid = int(self.motor_id)
        self.can.ser.reset_input_buffer()
        self.can.send_can(mid, [0xE0, 0x01])
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            fr = self.can.recv_can(0.10)
            if fr is None or fr.can_id != mid or len(fr.data) < 2:
                continue
            d = fr.data
            msg_type = (d[0] >> 5) & 0x07
            err = d[0] & 0x1F
            if err:
                raise RuntimeError(f"Motor error {err}: {ERROR_TEXT.get(err, 'Unknown')}")
            if msg_type == 0x05 and d[1] == 0x01 and len(d) >= 6:
                return struct.unpack(">f", d[2:6])[0]
        raise TimeoutError("No position-query response")

    @staticmethod
    def pack_servo_position(
        position_deg: float,
        speed_rpm: float,
        current_limit_a: float,
        ack: int = 2,
    ) -> bytes:
        if not 0.0 <= speed_rpm <= 3276.7:
            raise ValueError("speed_rpm out of protocol range")
        if not 0.0 <= current_limit_a <= 409.5:
            raise ValueError("current_limit_a out of protocol range")
        if ack not in (0, 1, 2, 3):
            raise ValueError("ack must be 0..3")

        speed_u15 = int(round(speed_rpm * 10.0))
        current_u12 = int(round(current_limit_a * 10.0))
        pos_bits = int.from_bytes(struct.pack(">f", float(position_deg)), "big")

        raw64 = (
            (0x01 << 61)
            | (pos_bits << 29)
            | ((speed_u15 & 0x7FFF) << 14)
            | ((current_u12 & 0x0FFF) << 2)
            | (ack & 0x03)
        )
        return raw64.to_bytes(8, "big")

    def send_position(
        self,
        target_deg: float,
        speed_rpm: float,
        current_limit_a: float,
        ack: int = 2,
    ) -> None:
        self._require_id()
        payload = self.pack_servo_position(
            target_deg,
            speed_rpm,
            current_limit_a,
            ack,
        )
        self.can.send_can(int(self.motor_id), payload)

    def read_feedback_type2(self, timeout: float = 0.08):
        self._require_id()
        deadline = time.monotonic() + timeout
        mid = int(self.motor_id)
        while time.monotonic() < deadline:
            fr = self.can.recv_can(
                timeout=min(0.02, max(0.001, deadline - time.monotonic()))
            )
            if fr is not None and fr.can_id == mid:
                fb = self.parse_feedback_type2(fr)
                if fb is not None:
                    return fb
        return None

    @staticmethod
    def parse_feedback_type2(frame: CanFrame):
        d = frame.data
        if len(d) < 8:
            return None
        msg_type = (d[0] >> 5) & 0x07
        err = d[0] & 0x1F
        if msg_type != 0x02:
            return None

        return {
            "position_deg": struct.unpack(">f", d[1:5])[0],
            "current_a": int.from_bytes(d[5:7], "big", signed=True) / 100.0,
            "temperature_c": (d[7] - 50) / 2.0,
            "error_code": err,
            "error_text": ERROR_TEXT.get(err, "Unknown"),
        }

    def _require_id(self) -> None:
        if self.motor_id is None:
            raise RuntimeError("Motor ID unknown")
