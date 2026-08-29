# ESP32-S3 Water Sampling Firmware

RTOS-based firmware for an ESP32-S3 controller used to operate a drone-mounted water-sampling mechanism.

The system controls:

- A continuous-rotation servo for nozzle deployment and retraction
- A water pump
- Upper and lower mechanical limit switches
- Motion timeout safety monitoring

The firmware is built with **ESP-IDF** and **FreeRTOS** and uses a modular, event-driven state-machine architecture rather than a traditional super-loop.

## Current Status

Implemented and tested:

- FreeRTOS-based state-machine task and event queue
- Runtime state registration
- Per-state lifecycle callbacks
  - State initialization
  - State deinitialization
  - Event processing
  - Next-state selection
- Global fault-event handling
- Registered failure-state handling for callback failures
- HALT behaviour
- Unknown-position handling for the continuous-rotation servo
- MCPWM-based nozzle-servo driver
- GPIO-based pump driver
- Pump integration with state entry/exit actions
- Upper and lower limit-switch driver
- GPIO interrupt handling
- Mechanical switch debounce
- Limit-switch startup synchronization
- Invalid simultaneous-limit detection
- FreeRTOS software-timer based motion timeout
- Motion timeout integration with lowering and raising states
- State-machine integration and negative-path tests
- Hardware bench testing of servo, pump, limit switches, and timeout behaviour

Planned:

- RC/Pixhawk PWM input
- RC signal-loss detection
- Full mechanical spool/nozzle integration testing
- Final motion-timeout calibration
- Additional diagnostics and fault handling

## Architecture

The firmware separates high-level system behaviour from hardware control.

```text
                      Event Producers
              ┌────────────┼────────────┐
              │            │            │
         RC/Pixhawk   Limit Switches   Debug/Test
              │            │            │
              └────────────┼────────────┘
                           ▼
                    FreeRTOS Event Queue
                           │
                           ▼
                     State Machine
                           │
                 ┌─────────┴─────────┐
                 ▼                   ▼
           Nozzle Servo             Pump
              MCPWM                 GPIO
                 │                   │
                 ▼                   ▼
        Continuous Servo      Pump Driver Circuit


                 Motion Timeout
                       │
                       └──────► FAULT event
```

The state machine owns the current logical system state.

Hardware and input components do not directly change the state. They generate events which are posted to the state-machine queue.

Similarly, hardware drivers know **how** to control their hardware, while state handlers determine **when** those actions should occur.

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

`STATE_MACHINE_POSITION_UNKNOWN` is required because the nozzle uses a continuous-rotation servo and therefore has no inherent position feedback.

If movement is interrupted before either physical endpoint is reached, the firmware cannot safely assume where the nozzle is located.

`STATE_MACHINE_FAULT` is a latched safe state. Normal operating commands are rejected while faulted, and a reset returns the system to `STATE_MACHINE_POSITION_UNKNOWN`.

## State Lifecycle

Each registered state is represented by a `StateMachineState_t` object containing lifecycle callbacks:

```c
typedef struct {
    StateMachineStateId_t eState;
    StateMachineStateInit_t cbInit;
    StateMachineStateDeinit_t cbDeinit;
    StateMachineStateProcess_t cbProcess;
    StateMachineStateNextState_t cbNextState;
} StateMachineState_t;
```

The lifecycle is:

```text
Event received
     │
     ▼
cbProcess()
     │
     ▼
cbNextState()
     │
     ├── same state ──► remain in current state
     │
     ▼
cbDeinit()
     │
     ▼
state transition
     │
     ▼
cbInit()
```

State entry and exit callbacks are used to control physical hardware safely.

For example:

```text
Enter LOWERING
    ├── Start servo extension
    └── Start motion timeout

Leave LOWERING
    ├── Stop servo
    └── Stop motion timeout
```

If a state initialization or deinitialization callback fails, the generic state-machine engine enters the registered failure state.

The failure state is currently:

```text
STATE_MACHINE_FAULT
```

## State Machine Component Layout

The state-machine implementation is split into three ESP-IDF components.

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
│       └── state_machine_common.h
│
└── state_machine_states/
    ├── state_machine_states.c
    ├── state_machine_global_events.c
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

`state_machine_common` contains the shared event, state ID, and state-object definitions.

`state_machine` contains the generic state-machine engine.

`state_machine_states` contains the application-specific states and registers them with the generic state-machine engine during startup.

Individual state files do not directly modify the global state.

## Nozzle Servo

The nozzle is driven using an RC-style continuous-rotation servo.

The driver uses the ESP32-S3 **MCPWM** peripheral rather than software-generated PWM.

Current PWM values:

```text
PWM frequency:  50 Hz
Period:         20,000 us

Retract:        1200 us
Neutral/Stop:   1450 us
Extend:         1800 us
```

These values may require further calibration once the final mechanical assembly is installed.

Public driver API:

```c
esp_err_t nozzleServoInit(void);
esp_err_t nozzleServoExtend(void);
esp_err_t nozzleServoRetract(void);
esp_err_t nozzleServoStop(void);
```

The component contains no FreeRTOS task. MCPWM generates the waveform in hardware.

The servo driver has been bench-tested and integrated with the state-machine lifecycle.

## Pump

The pump is controlled through a GPIO output connected to the external pump driver circuit.

Public API:

```c
esp_err_t pumpInit(void);
esp_err_t pumpOn(void);
esp_err_t pumpOff(void);
```

The pump starts in the safe OFF state.

State-machine integration:

```text
Enter PUMPING
    └── pumpOn()

Leave PUMPING
    └── pumpOff()

Enter FAULT
    └── pumpOff()
```

The driver and state-machine integration have been bench-tested successfully.

## Limit Switches

Two mechanical limit switches provide endpoint feedback:

```text
Upper limit → nozzle fully stowed
Lower limit → nozzle fully deployed
```

The switches are currently configured as active-low inputs using GPIO pull-ups:

```text
Released → HIGH
Active   → LOW
```

Both GPIOs use interrupts on either edge.

The interrupt handlers do not directly post state-machine events. Instead, they notify a dedicated limit-switch task:

```text
GPIO edge
    │
    ▼
GPIO ISR
    │
    ▼
FreeRTOS task notification
    │
    ▼
30 ms debounce delay
    │
    ▼
Read stable GPIO level
    │
    ▼
Post state-machine event
```

Generated events include:

```text
SM_EVENT_UPPER_LIMIT_ACTIVE
SM_EVENT_UPPER_LIMIT_RELEASED
SM_EVENT_LOWER_LIMIT_ACTIVE
SM_EVENT_LOWER_LIMIT_RELEASED
```

At startup, `limitSwitchSyncState()` reads both inputs and synchronizes the physical mechanism position with the state machine.

Possible startup conditions:

```text
Upper active, lower released
    → STOWED

Upper released, lower active
    → DEPLOYED

Both released
    → POSITION_UNKNOWN

Both active
    → FAULT
```

Both switches being active simultaneously is treated as an invalid physical condition and generates a fault.

## Motion Timeout

Nozzle movement is protected by a one-shot FreeRTOS software timer.

Public API:

```c
esp_err_t motionTimeoutInit(void);
esp_err_t motionTimeoutStart(void);
esp_err_t motionTimeoutStop(void);
```

When entering either movement state:

```text
LOWERING
RAISING
```

the timeout is started.

When the expected endpoint is reached and the movement state is exited, the timeout is stopped.

If no limit switch is reached before the timeout expires:

```text
Motion timeout expires
        │
        ▼
SM_EVENT_FAULT
        │
        ▼
STATE_MACHINE_FAULT
        │
        ├── Servo STOP
        └── Pump OFF
```

The current development timeout is:

```text
10 seconds
```

This is a temporary value and will be calibrated using the final mechanical travel time.

The timeout component has been independently tested and integration-tested with both lowering and raising states.

## ESP-IDF Component Dependencies

The state-machine dependency direction is intentionally layered:

```text
state_machine_common
        ▲
        │
 state_machine
        ▲
        │
state_machine_states
        ▲
        │
       main
```

Hardware components such as:

```text
nozzle_servo
pump
motion_timeout
```

are private dependencies of the application-specific state component where required.

The generic `state_machine` component does not depend on individual application states.

This keeps the state-machine engine reusable and prevents application-specific behaviour from leaking into the generic state-management layer.

## Startup Sequence

The current initialization sequence is approximately:

```text
nozzleServoInit()
pumpInit()

stateMachineStatesRegister()
stateMachineInit()

motionTimeoutInit()
limitSwitchInit()

SM_EVENT_SYSTEM_READY
limitSwitchSyncState()
```

The state-machine event queue is initialized before interrupt-driven input components begin generating events.

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

## Testing

The firmware currently includes both component-level bench tests and state-machine integration tests.

Validated state-machine behaviour includes:

- Normal deployment and retraction sequence
- Invalid event handling
- Pump commands rejected outside the deployed state
- Incorrect limit-switch events ignored during motion
- Interrupted motion producing `POSITION_UNKNOWN`
- Global FAULT handling
- Latched fault behaviour
- RESET recovery
- HALT behaviour across all relevant states
- State initialization failure handling
- State deinitialization failure handling

Hardware/component testing has validated:

- MCPWM servo extend/retract/neutral behaviour
- Pump ON/OFF control
- Upper limit active/released detection
- Lower limit active/released detection
- Mechanical switch debounce
- Startup limit-state synchronization
- Simultaneous upper/lower limit fault detection
- Motion timeout expiry
- Motion timeout cancellation
- Motion timeout restart behaviour
- Lowering timeout → FAULT
- Raising timeout → FAULT
- Successful endpoint detection cancelling the timeout

## Safety Design

The firmware currently implements the following safety behaviours:

- Pump OFF on startup
- Servo neutral/stop command on startup
- Pump operation only while the nozzle is confirmed deployed
- Motion completion only after the correct limit switch is reached
- Unknown position after interrupted movement
- HALT immediately stops active movement
- FAULT is latched and rejects normal operating commands
- Entering FAULT stops the servo
- Entering FAULT turns the pump OFF
- Simultaneous upper and lower limit activation produces FAULT
- Lowering and raising are protected by motion timeouts
- State callback failures transition to the registered failure state

Planned RC safety behaviour:

- RC/Pixhawk signal-loss detection
- Pump shutdown on RC signal loss
- Safe handling of invalid or stale RC commands

Hardware-level fail-safe design should also ensure actuator outputs remain safe while the ESP32 is resetting, booting, or unpowered.

## Development Roadmap

Completed:

1. Implement state-machine framework
2. Implement state lifecycle and failure handling
3. Implement MCPWM nozzle-servo driver
4. Integrate servo control with state transitions
5. Implement pump GPIO driver
6. Integrate pump control with state transitions
7. Implement upper/lower limit-switch inputs
8. Add GPIO interrupt and debounce handling
9. Add startup limit-state synchronization
10. Add simultaneous-limit fault detection
11. Implement motion timeout
12. Integrate motion timeout with nozzle movement
13. Bench-test implemented safety paths

Next:

14. Implement RC/Pixhawk PWM capture
15. Add RC signal-loss detection
16. Integrate RC commands with state-machine events
17. Test the complete mechanical spool/nozzle assembly
18. Calibrate final servo and motion-timeout values
19. Add additional diagnostics and watchdog behaviour as required

## Design Philosophy

The firmware follows a few core rules:

- **Events describe what happened**
- **States describe the current system condition**
- **Only the state machine owns the current state**
- **Hardware drivers know how to control hardware, not when**
- **Input components post events rather than directly commanding actuators**
- **State entry and exit callbacks own state-specific hardware actions**
- **Safe states explicitly establish safe actuator outputs**
- **RTOS tasks are only added where concurrency is actually required**

This keeps the firmware modular, testable, and scalable as additional inputs, diagnostics, and safety features are added.