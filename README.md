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
│   └── README.md
├── gui/
│   ├── main.py
│   ├── serial_worker.py
│   └── udp_server.py
├── .gitignore
├── requirements.txt
└── README.md
```

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
