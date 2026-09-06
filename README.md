# ESP32-S3 Water Sampling Firmware

ESP-IDF / FreeRTOS firmware for an ESP32-S3 controller used in a drone-mounted water-sampling system.

The firmware controls nozzle deployment, water pumping, RC/Pixhawk command input, mechanical limit switches, motion safety, and local power/status diagnostics.

---

## Features

Currently implemented:

- Event-driven FreeRTOS state machine
- Continuous-rotation nozzle servo control using MCPWM
- Water pump GPIO control
- Upper and lower mechanical limit switches
- Motion timeout safety monitoring
- RC PWM input using MCPWM capture
- RC signal-loss detection
- INA226 voltage, current, and power monitoring
- SSD1306 OLED diagnostics
- Component-level and integration tests
- Explicit component initialization and cleanup

---

## Architecture

The firmware is divided into independent ESP-IDF components.

```text
             RC Input
                │
         Limit Switches
                │
                ▼
        State-Machine Queue
                │
                ▼
           State Machine
          ┌─────┴─────┐
          ▼           ▼
     Nozzle Servo    Pump


       Motion Timeout
             │
             └──────► FAULT


          INA226
             │
             ▼
        Diagnostics
             │
             ▼
        SSD1306 OLED
```

The state machine owns the logical system state.

Input components generate events rather than directly controlling actuators, while state handlers decide when hardware actions should occur.

---

## State Machine

Current states:

```text
INIT
STOWED
LOWERING
DEPLOYED
PUMPING
RAISING
POSITION_UNKNOWN
FAULT
```

Typical operating sequence:

```text
INIT
  │
  ▼
POSITION_UNKNOWN
  │ upper limit
  ▼
STOWED
  │ extend
  ▼
LOWERING
  │ lower limit
  ▼
DEPLOYED
  │ pump on
  ▼
PUMPING
  │ pump off
  ▼
DEPLOYED
  │ retract
  ▼
RAISING
  │ upper limit
  ▼
STOWED
```

`POSITION_UNKNOWN` is used when nozzle position can no longer be safely determined.

`FAULT` is latched and requires a reset before normal operation can resume.

---

## Safety Behaviour

Current safety mechanisms include:

- Pump defaults to OFF
- Pump only operates while the nozzle is confirmed deployed
- Servo PWM is disabled while stationary
- Upper and lower limits confirm movement completion
- Interrupted movement results in `POSITION_UNKNOWN`
- Motion timeout during lowering or raising generates `FAULT`
- Simultaneous upper and lower limit activation generates `FAULT`
- FAULT disables servo output and turns the pump OFF
- RC signal loss stops active pumping or movement
- State callback failures transition to the registered failure state

---

## RC Control

Current development setup:

```text
FlySky Transmitter
        │
        ▼
FS-iA10B Receiver
        │ PWM
        ▼
ESP32-S3
```

Intended production path:

```text
Transmitter
    │
    ▼
Receiver
    │
    ▼
Pixhawk 6C
    │ AUX PWM
    ▼
ESP32-S3
```

Current mappings:

```text
Pump:
    HIGH → OFF
    LOW  → ON

Nozzle:
    HIGH → EXTEND
    LOW  → RETRACT
```

The ESP32 also detects disappearance of valid PWM as an additional signal-loss safeguard.

---

## Diagnostics

The diagnostic subsystem uses a shared I2C bus:

```text
ESP32-S3
    │
    ├── INA226   0x40
    └── SSD1306  0x3C
```

The OLED displays information including:

- State-machine state
- Battery voltage
- Current
- Power

Diagnostics are currently passive and do not directly affect actuator control.

---

## Testing

Tests are implemented as a separate ESP-IDF component.

Each test initializes and deinitializes its own dependencies so tests can be run independently.

Current test coverage includes:

- Normal and invalid state-machine sequences
- FAULT, RESET, and HALT behaviour
- State callback failure handling
- Servo and pump control
- Limit-switch behaviour
- Motion timeout behaviour
- RC signal-loss handling
- RC pump/nozzle integration
- INA226 measurements
- SSD1306 output
- Combined diagnostics display

Hardware-interactive tests should generally be run individually because they may require physical switch, transmitter, or mechanism input.

---

## Building

Requires ESP-IDF.

```bash
idf.py build
```

Flash and monitor:

```bash
idf.py flash monitor
```

Clean rebuild if required:

```bash
idf.py fullclean
idf.py build
```

---

## Current Status

The main firmware architecture and individual hardware components are implemented and bench-tested.

Current development focus:

- Revalidate RC behaviour with the receiver disconnected
- Revalidate RC signal-loss handling after lifecycle changes
- Test the complete mechanical nozzle/spool assembly
- Calibrate final servo and motion-timeout values
- Validate INA226 measurements on the complete subsystem
- Finalize the production PCB and power architecture

---

## Design Principles

- Events describe what happened
- States describe the current system condition
- Only the state machine owns logical state
- Hardware drivers control hardware, not system policy
- Input components generate events rather than commanding actuators directly
- State entry/exit callbacks own state-specific hardware actions
- Components own and clean up their own resources
- RTOS tasks are only used where concurrency is required
- Tests are isolated and self-contained