# griper_hybrid

Hybrid gripper control project using STM32F103 + CAN + ENCOS EC-A4310-P2-36, with a PyQt5 GUI on the PC.

## Architecture

```text
Robot main controller
        |
        | UDP text commands (future integration)
        v
+------------------------+
| PyQt5 GUI on PC        |
| - reads YAML config    |
| - sends serial cmds    |
| - displays telemetry   |
+-----------+------------+
            | USB-UART
            v
+------------------------+
| STM32F103 firmware     |
| - real-time loop       |
| - safety limits        |
| - gripper state        |
| - CAN communication    |
+-----------+------------+
            | CAN
            v
+------------------------+
| EC-A4310-P2-36         |
+-----------+------------+
            |
            v
         Gripper
```

## Project structure

```text
griper_hybrid/
├── config/
│   └── gripper_config.yaml
├── docs/
│   └── communication_protocol.md
├── firmware/
│   ├── README.md
│   └── app/
│       ├── gripper_control.c/.h
│       ├── encos_motor.c/.h
│       ├── safety.c/.h
│       └── serial_protocol.c/.h
├── gui/
│   ├── main.py
│   ├── serial_worker.py
│   └── udp_server.py
├── .gitignore
├── requirements.txt
└── README.md
```

## Run the GUI on Windows

Clone the repository and enter the project directory:

```bash
git clone https://github.com/Poseidon1123/griper_hybrid.git
cd griper_hybrid
```

Create a Python virtual environment:

```bash
python -m venv .venv
.venv\Scripts\activate
```

Install dependencies:

```bash
pip install -r requirements.txt
```

Edit the serial port in:

```text
config/gripper_config.yaml
```

For example:

```yaml
serial:
  port: "COM5"
  baudrate: 115200
```

Run the GUI:

```bash
python gui/main.py
```

The GUI can be opened before STM32 is connected. Motion buttons remain disabled until the serial connection is established.

## STM32 workflow

1. Create an STM32CubeIDE project for the actual STM32F103 board/MCU.
2. Configure UART, CAN1, GPIO safety inputs, and a periodic timer.
3. Copy/integrate the modules under `firmware/app/` into the CubeIDE project.
4. Verify PC <-> STM32 text communication first.
5. Verify raw CAN communication without motion.
6. Only after the official EC-A4310-P2-36 CAN protocol is confirmed, implement the functions in `encos_motor.c`.

## Development milestones

1. Verify PC <-> STM32 serial communication.
2. Configure STM32 CAN and verify raw CAN frames.
3. Confirm the official CAN protocol for EC-A4310-P2-36.
4. Implement motor enable/disable and feedback readout.
5. Implement position control.
6. Map motor position to gripper width (0-80 mm target range).
7. Add software safety limits and communication watchdog.
8. Connect the PyQt5 GUI to STM32.
9. Load/edit parameters from YAML.
10. Add UDP integration with the robot main controller.

## Important safety note

The EC-A4310 CAN frame format must be verified from the correct ENCOS manual/protocol before sending motion commands. Placeholder CAN IDs or payloads must not be used on the real motor.
