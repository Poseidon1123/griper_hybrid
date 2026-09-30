from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import serial
from serial.tools import list_ports

SERIAL_BAUD = 2_000_000
CAN_BAUD_CODE_1M = 0x01
CAN_STANDARD = 0x01
CAN_MODE_NORMAL = 0x00


@dataclass
class CanFrame:
    can_id: int
    data: bytes


def available_ports():
    return list(list_ports.comports())


class WaveshareUsbCanA:
    """Waveshare USB-CAN-A using the variable-length serial protocol."""

    def __init__(self, port: str, timeout: float = 0.03):
        self.port = port
        self.ser = serial.Serial(
            port=port,
            baudrate=SERIAL_BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=0.5,
        )

    def close(self) -> None:
        if self.ser.is_open:
            self.ser.close()

    def configure_1m_standard_normal(self) -> None:
        cmd = bytearray([
            0xAA, 0x55,
            0x12,
            CAN_BAUD_CODE_1M,
            CAN_STANDARD,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            CAN_MODE_NORMAL,
            0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00,
        ])
        cmd[19] = sum(cmd[2:19]) & 0xFF
        self.ser.reset_input_buffer()
        self.ser.write(cmd)
        self.ser.flush()
        time.sleep(0.10)
        self.ser.reset_input_buffer()

    def send_can(self, can_id: int, data) -> None:
        if not 0 <= can_id <= 0x7FF:
            raise ValueError("Standard CAN ID must be in range 0x000..0x7FF")
        data = bytes(data)
        if len(data) > 8:
            raise ValueError("Classic CAN payload max = 8 bytes")

        frame = bytearray([
            0xAA,
            0xC0 | len(data),
            can_id & 0xFF,
            (can_id >> 8) & 0x07,
        ])
        frame.extend(data)
        frame.append(0x55)
        self.ser.write(frame)
        self.ser.flush()

    def recv_can(self, timeout: float = 0.10) -> Optional[CanFrame]:
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            if b[0] != 0xAA:
                continue

            type_b = self.ser.read(1)
            if not type_b:
                continue
            t = type_b[0]

            if t == 0x55:
                self.ser.read(18)
                continue

            if (t & 0xC0) != 0xC0:
                continue

            is_extended = bool(t & 0x20)
            is_remote = bool(t & 0x10)
            dlc = t & 0x0F
            if dlc > 8:
                continue

            id_len = 4 if is_extended else 2
            id_bytes = self.ser.read(id_len)
            if len(id_bytes) != id_len:
                continue

            data = self.ser.read(0 if is_remote else dlc)
            if len(data) != (0 if is_remote else dlc):
                continue

            tail = self.ser.read(1)
            if tail != b"\x55":
                continue

            return CanFrame(
                can_id=int.from_bytes(id_bytes, byteorder="little"),
                data=data,
            )

        return None

    def wait_for_id(self, can_id: int, timeout: float = 0.30) -> Optional[CanFrame]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.recv_can(
                timeout=min(0.03, max(0.001, deadline - time.monotonic()))
            )
            if frame is not None and frame.can_id == can_id:
                return frame
        return None
