# griper_hybrid

Control project for a hybrid two-finger gripper using:

- ENCOS EC-A4310-P2-36 joint motor
- Waveshare USB-CAN-A
- Classic CAN at 1 Mbit/s
- Python + PyQt5 HMI

The current prototype architecture is:

```text
Robot main controller (future UDP)
            |
            v
+---------------------------+
| PyQt5 HMI on Windows PC   |
| - parameters / YAML       |
| - telemetry               |
| - grip state machine      |
+-------------+-------------+
              | USB serial protocol
              v
+---------------------------+
| Waveshare USB-CAN-A       |
+-------------+-------------+
              | CAN 1 Mbps
              v
+---------------------------+
| EC-A4310-P2-36            |
+-------------+-------------+
              |
              v
           Gripper
```

`firmware/` is retained for the later STM32 real-time version. The current HMI controls the motor directly through USB-CAN-A.

## Features

- automatic COM-port discovery
- motor CAN-ID discovery / Q&A mode
- actual motor position feedback
- actual current feedback
- motor temperature and fault display
- relative jog
- absolute position command
- OPEN / RELEASE command
- automatic `CLOSE -> CONTACT -> HOLD`
- adjustable `I_close`, `I_hold`, contact-current threshold, speed and preload
- configuration stored in `config/gripper_config.yaml`
- CAN control runs in a worker thread so the PyQt5 GUI stays responsive

## Automatic grip logic

The controller does not declare contact from current alone. Contact is confirmed when:

1. actual current exceeds `contact_current_a`, and
2. motor position changes by less than `stall_move_deg` over `stall_window_s`, and
3. the condition is confirmed for several consecutive samples.

After contact:

```text
CLOSING
   |
   | contact detected
   v
HOLD
```

The HOLD state keeps a small position preload while reducing the servo-position current threshold to `i_hold_a`.

> `i_hold_a` is a current threshold/limit in servo-position mode. Actual current is not guaranteed to equal exactly `i_hold_a`.

## Install

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

## Run HMI

From the repository root:

```bash
python run_hmi.py
```

Then:

1. Select `USB-SERIAL CH340` (the Waveshare USB-CAN-A COM port).
2. Click **Connect**.
3. Verify position feedback.
4. Use small Jog commands first to confirm the closing direction.
5. Set **Close direction** to `+1` or `-1`.
6. Click **Apply + Save YAML**.
7. Test **GRIP / AUTO HOLD** with a light object first.

## Main project structure

```text
griper_hybrid/
├── config/
│   └── gripper_config.yaml
├── controller/
│   ├── __init__.py
│   └── gripper_controller.py
├── drivers/
│   ├── __init__.py
│   ├── ec_a4310.py
│   └── waveshare_usb_can.py
├── firmware/
│   └── ...
├── gui/
│   ├── __init__.py
│   ├── main.py
│   ├── serial_worker.py       # legacy STM32 path, kept for later
│   └── udp_server.py          # future robot integration
├── requirements.txt
└── run_hmi.py
```

## Important configuration

`config/gripper_config.yaml`:

```yaml
gripper:
  close_direction: -1
  max_close_travel_deg: 90.0
  open_release_delta_deg: 15.0

grip:
  close_speed_rpm: 2.0
  i_close_a: 1.2
  i_hold_a: 0.8
  contact_current_a: 0.6
  preload_deg: 2.0

safety:
  absolute_current_limit_a: 3.0
  max_motor_temp_c: 70.0
```

These are prototype values, not guaranteed values for lifting a 3 kg object. Calibrate current/force against the real mechanism, finger friction, geometry and thermal behavior.

## Safety notes

- Confirm the motor closing direction with a small jog before automatic gripping.
- Set `max_close_travel_deg` to a mechanically safe value.
- Start with light objects and low current.
- STOP ends the command stream; when HOLD is active, stopping may release the object.
- PC + Windows + USB-CAN is not hard real-time. For a final real-time controller, move the fast safety/control loop to STM32 and keep the HMI on the PC.
