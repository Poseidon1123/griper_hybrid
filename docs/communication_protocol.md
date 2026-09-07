# Communication protocol

This document defines the PC <-> STM32 text protocol used during development.
The robot-controller UDP protocol can later reuse or wrap these commands.

## Transport

- PC <-> STM32: USB-UART, default 115200 baud, 8N1.
- Line ending: `\n`.
- Encoding: UTF-8 / ASCII-compatible text.
- One command per line.

## Commands from PC to STM32

```text
PING
ENABLE
DISABLE
OPEN
CLOSE
STOP
SET_WIDTH <width_mm>
SET_VELOCITY <velocity_mm_s>
SET_FORCE <force_n>
GET_STATUS
```

Examples:

```text
PING
SET_WIDTH 40.0
SET_VELOCITY 15.0
STOP
```

## Telemetry from STM32 to PC

```text
PONG
WIDTH,<mm>
POS,<motor_position>
VEL,<motor_velocity>
TORQUE,<Nm>
TEMP,<degC>
STATE,<state_name>
ERROR,<error_code>,<message>
```

Example:

```text
WIDTH,42.5
POS,1.234
VEL,0.250
TORQUE,2.10
TEMP,38.0
STATE,READY
```

## Safety behavior

- `STOP` must have priority over normal motion commands.
- Invalid numeric values must be rejected.
- Width requests must be clamped/rejected outside configured mechanical limits.
- Communication watchdog behavior must be implemented on STM32, not only in the GUI.
- Motion commands to EC-A4310 must remain disabled until the official CAN protocol is verified.

## EC-A4310 CAN protocol

Not defined here yet. Do **not** invent CAN IDs or byte payloads. The exact CAN frame format, scaling, enable/disable sequence, feedback IDs, and control modes must be taken from the official protocol for the EC-A4310-P2-36 unit being used.
