# ESP32-S3 Water Sampling Firmware

RTOS-based firmware for an ESP32-S3 controller used to operate a water-sampling mechanism. The system controls a continuous-rotation servo for nozzle deployment/retraction and will also control a water pump and monitor upper/lower limit switches.

The firmware is built with **ESP-IDF** and **FreeRTOS** and is designed around a modular, event-driven state machine rather than a traditional super-loop.

## Current Status

Implemented:

- FreeRTOS-based state machine task and event queue
- Per-state event handlers
- State transition validation
- Fault handling
- Unknown-position handling for the continuous-rotation servo
- MCPWM-based nozzle servo driver
- State-machine integration tests for valid and invalid event sequences

In progress / planned:

- Bench testing of the nozzle servo
- State-entry actuator integration
- Pump GPIO driver
- Upper and lower limit-switch inputs
- Limit-switch debounce and ISR/event handling
- Motion timeouts
- RC/Pixhawk PWM input
- Additional safety and diagnostics

## Architecture

The firmware separates high-level behaviour from hardware control.

```text
                Event Producers
        ┌────────────┼────────────┐
        │            │            │
   RC/Pixhawk   Limit Switches   Debug/UI
        │            │            │
        └────────────┼────────────┘
                     ▼
              FreeRTOS Event Queue
                     │
                     ▼
               State Machine
                     │
          ┌──────────┴──────────┐
          ▼                     ▼
    Nozzle Servo              Pump
       MCPWM                  GPIO
          │                     │
          ▼                     ▼
 Continuous Servo        Pump Driver Circuit
```

The state machine owns the current system state. Other components do not modify the state directly; they post events to the state-machine queue.

## State Machine

Current states:

```text
STATE_MACHINE_INIT
STATE_MACHINE_STOWED
STATE_MACHINE_LOWERING
STATE_MACHINE_DEPLOYED
STATE_MACHINE_PUMPING
STATE_MACHINE_RAISING
STATE_MACHINE_POSITION_UNKNOWN
STATE_MACHINE_FAULT
```

Typical operating sequence:

```text
INIT
  │ SYSTEM_READY
  ▼
POSITION_UNKNOWN
  │ UPPER_LIMIT_ACTIVE
  ▼
STOWED
  │ NOZZLE_EXTEND
  ▼
LOWERING
  │ LOWER_LIMIT_ACTIVE
  ▼
DEPLOYED
  │ PUMP_ON
  ▼
PUMPING
  │ PUMP_OFF
  ▼
DEPLOYED
  │ NOZZLE_RETRACT
  ▼
RAISING
  │ UPPER_LIMIT_ACTIVE
  ▼
STOWED
```

`STATE_MACHINE_POSITION_UNKNOWN` is required because the nozzle uses a continuous-rotation servo and therefore has no inherent position feedback. If motion is stopped between the two limit switches, the firmware cannot assume the physical nozzle position.

`STATE_MACHINE_FAULT` is latched and rejects normal operating events until a reset event is received.

## State Machine Component Layout

The state machine is split into three ESP-IDF components.

```text
components/
├── state_machine/
│   ├── state_machine.c
│   ├── CMakeLists.txt
│   └── include/
│       └── state_machine.h
│
├── state_machine_common/
│   ├── state_machine_common.c
│   ├── CMakeLists.txt
│   └── include/
│       ├── state_machine_event.h
│       └── state_machine_state.h
│
└── state_machine_states/
    ├── state_machine_state_init.c
    ├── state_machine_state_stowed.c
    ├── state_machine_state_lowering.c
    ├── state_machine_state_deployed.c
    ├── state_machine_state_pumping.c
    ├── state_machine_state_raising.c
    ├── state_machine_state_pos_unknown.c
    ├── state_machine_state_fault.c
    └── include/
        └── ...
```

Each state handler receives an event and returns the next requested state. Individual state files do not directly modify the global state.

Example:

```c
StateMachineState_t stateMachineStateLoweringProcessEvent(
    const StateMachineEvent_t *pEvent
)
{
    switch (pEvent->eId) {

        case SM_EVENT_LOWER_LIMIT_ACTIVE:
            return STATE_MACHINE_DEPLOYED;

        case SM_EVENT_STOP_SPOOL:
            return STATE_MACHINE_POSITION_UNKNOWN;

        default:
            return STATE_MACHINE_LOWERING;
    }
}
```

The central state-machine component performs the actual transition.

## Nozzle Servo

The nozzle is driven using an RC-style continuous-rotation servo.

The servo driver uses the ESP32-S3 **MCPWM** peripheral rather than a software-generated PWM signal.

Current nominal PWM values:

```text
PWM frequency:  50 Hz
Period:         20,000 us

Retract:        1200 us
Neutral/Stop:   1500 us
Extend:         1800 us
```

These values may require calibration for the final servo and mechanism.

Public driver API:

```c
esp_err_t nozzleServoInit(void);
esp_err_t nozzleServoExtend(void);
esp_err_t nozzleServoRetract(void);
esp_err_t nozzleServoStop(void);
```

The component intentionally contains no FreeRTOS task. MCPWM generates the servo waveform in hardware.

## ESP-IDF Component Dependencies

Current state-machine dependency structure:

```text
state_machine_common
        ▲
        │
state_machine_states
        ▲
        │
   state_machine
        ▲
        │
       main
```

The `state_machine` component publicly depends on `state_machine_common` because its public API exposes shared event/state types, while `state_machine_states` is an internal implementation dependency.

Example:

```cmake
idf_component_register(
    SRCS "state_machine.c"
    INCLUDE_DIRS "include"
    REQUIRES state_machine_common
    PRIV_REQUIRES state_machine_states
)
```

## Building

This project requires ESP-IDF.

From an ESP-IDF terminal:

```bash
idf.py build
```

To perform a clean rebuild:

```bash
idf.py fullclean
idf.py build
```

## Flashing and Monitoring

Flash the ESP32-S3 and open the serial monitor:

```bash
idf.py flash monitor
```

If the serial port is not detected automatically:

```bash
idf.py -p <PORT> flash monitor
```

Example on Windows:

```bash
idf.py -p COM5 flash monitor
```

Example on macOS:

```bash
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```

## State Machine Testing

The state machine can be tested without connecting any actuators by posting synthetic events from `app_main()`.

Example normal sequence:

```c
stateMachineInit();

stateMachinePostEvent(SM_EVENT_SYSTEM_READY);
stateMachinePostEvent(SM_EVENT_UPPER_LIMIT_ACTIVE);
stateMachinePostEvent(SM_EVENT_NOZZLE_EXTEND);
stateMachinePostEvent(SM_EVENT_LOWER_LIMIT_ACTIVE);
stateMachinePostEvent(SM_EVENT_PUMP_ON);
stateMachinePostEvent(SM_EVENT_PUMP_OFF);
stateMachinePostEvent(SM_EVENT_NOZZLE_RETRACT);
stateMachinePostEvent(SM_EVENT_UPPER_LIMIT_ACTIVE);
```

Negative-path tests have also been used to verify that:

- The pump cannot start while the nozzle is stowed
- The pump cannot start while the nozzle is moving
- The wrong limit switch does not complete a movement
- Stopping movement before an endpoint produces `POSITION_UNKNOWN`
- Faults are latched
- Normal commands are rejected while faulted
- Reset returns the mechanism to `POSITION_UNKNOWN`

## Safety Design

The firmware is being designed around several safety principles:

- Pump OFF on startup
- Servo neutral/stop command on startup
- Pump operation only while the nozzle is confirmed deployed
- Motion completion only after the correct limit switch is reached
- Unknown position after interrupted movement
- Fault state rejects normal operating commands
- RC signal loss will stop the pump
- Motion timeouts will be added so a failed limit switch cannot leave the servo running indefinitely

Hardware-level fail-safe design should also ensure that actuator outputs remain safe while the ESP32 is resetting, booting, or unpowered.

## Development Roadmap

Planned implementation order:

1. Bench-test MCPWM nozzle-servo driver
2. Connect state-entry actions to the nozzle-servo driver
3. Implement pump GPIO driver
4. Integrate pump control with state transitions
5. Implement upper/lower limit-switch inputs
6. Add debounce and ISR-safe event posting
7. Add motion timeouts
8. Implement RC/Pixhawk PWM capture
9. Add diagnostics, watchdogs, and additional fault handling

## Design Philosophy

The firmware follows a few core rules:

- **Events describe what happened**
- **States describe the current system condition**
- **Only the state machine owns the current state**
- **Hardware drivers know how to control hardware, not when**
- **Input components post events rather than directly commanding actuators**
- **RTOS tasks are only added where concurrency is actually required**

This keeps the firmware modular, testable, and scalable as additional sensors and safety features are added.