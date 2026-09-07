# STM32 firmware

Target controller: STM32F103 series.

The STM32 is responsible for deterministic gripper control and safety. The PC GUI is not part of the real-time loop.

## Recommended peripherals

- CAN1: communication with EC-A4310-P2-36 through a 3.3 V CAN transceiver.
- USART1 or USB-UART: communication with the PC GUI.
- TIM2: periodic scheduler/control tick, initial target 1 kHz.
- GPIO inputs: open/close limit switches and emergency-stop status if used.

## Proposed application modules

```text
Core/
├── Inc/
│   ├── gripper_control.h
│   ├── encos_motor.h
│   ├── serial_protocol.h
│   └── safety.h
└── Src/
    ├── gripper_control.c
    ├── encos_motor.c
    ├── serial_protocol.c
    └── safety.c
```

## Real-time responsibility

A periodic timer should set a control flag or run a short deterministic task. Avoid long blocking work inside interrupts. A common pattern is:

```text
TIM2 tick at 1 kHz
      |
      v
set control_tick flag
      |
      v
main loop executes Gripper_Update()
```

CAN receive should use interrupts/FIFO and copy received data into state variables quickly.

## Important

Do not send guessed EC-A4310 CAN commands to the real actuator. The CAN protocol must be verified first.
