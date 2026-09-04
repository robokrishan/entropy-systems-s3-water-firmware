# ESP32-S3 Water Sampling Firmware

RTOS-based firmware for an ESP32-S3 controller used to operate and monitor a drone-mounted water-sampling mechanism.

The system controls and monitors:

- A continuous-rotation servo for nozzle deployment and retraction
- A water pump
- Upper and lower mechanical limit switches
- RC/Pixhawk PWM command inputs
- Motion timeout safety monitoring
- INA226 voltage, current, and power monitoring
- SSD1306 OLED diagnostics display

The firmware is built with **ESP-IDF** and **FreeRTOS** and uses a modular, event-driven state-machine architecture rather than a traditional super-loop.

---

## Current Status

### Implemented and Tested

#### State Machine

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
- RESET recovery
- Unknown-position handling for the continuous-rotation servo
- Negative-path and invalid-event testing

#### Actuator Control

- MCPWM-based nozzle-servo driver
- GPIO-based pump driver
- Servo integration with state entry/exit actions
- Pump integration with state entry/exit actions
- Hardware bench testing of servo and pump behaviour

#### Limit Switches

- Upper and lower limit-switch driver
- GPIO interrupt handling
- Mechanical switch debounce
- Startup position synchronization
- Invalid simultaneous-limit detection
- State-machine event integration

#### Motion Safety

- FreeRTOS software-timer based motion timeout
- Motion timeout integration with lowering and raising states
- Automatic transition to FAULT on movement timeout
- Timeout cancellation after successful endpoint detection

#### RC Input

- MCPWM capture-based PWM input measurement
- Multi-channel RC input support
- Pump command input
- Nozzle extend/retract command input
- RC signal-loss timeout detection
- RC command integration with state-machine events
- Hardware testing using a FlySky FS-iA10B receiver
- Physical pump control from transmitter
- Physical nozzle control from transmitter
- Full manual pump/nozzle integration testing

#### I2C and Diagnostics

- Shared ESP-IDF I2C master-bus component
- Multiple devices operating on the same I2C bus
- INA226 current/voltage/power-monitor component
- INA226 device identification and calibration
- Bus-voltage measurement
- Shunt-voltage measurement
- Current measurement
- Power measurement
- SSD1306 128x64 OLED component
- OLED initialization and display clearing
- 5x7 ASCII text rendering
- Periodic diagnostics FreeRTOS task
- Live state-machine status display
- Live INA226 voltage/current/power display

### Current Development Issue

The `rc_input` component is currently being investigated for a stability issue when the RC receiver is not connected.

The component has previously operated successfully with a physical receiver and transmitter, including pump and nozzle integration testing. However, an unconnected/floating RC input can produce invalid capture activity and has exposed a crash path that still requires debugging.

RC input is therefore temporarily disabled during unrelated bench development where no receiver is available.

### Remaining Work

- Debug RC-input behaviour with missing/unconnected receiver
- Revalidate RC signal-loss handling after the fix
- Test the complete mechanical spool/nozzle assembly
- Calibrate final motion timeout using the production mechanism
- Validate INA226 monitoring on the complete subsystem power rail
- Finalize production power architecture and PCB
- Add additional diagnostics or safety monitoring as requirements are finalized

---

## Architecture

The firmware separates high-level system behaviour, hardware control, command inputs, and diagnostics.

```text
                         Event Producers
                 ┌────────────┼────────────┐
                 │            │            │
             RC Input    Limit Switches   Tests
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
                MCPWM                   GPIO
                   │                     │
                   ▼                     ▼
          Continuous Servo        Pump Driver


                Motion Timeout
                      │
                      └────────► SM_EVENT_FAULT


                Diagnostic Sources
              ┌────────┴─────────┐
              │                  │
           INA226          State Machine
              │                  │
              └────────┬─────────┘
                       ▼
                Diagnostics Task
                       │
                       ▼
                  SSD1306 OLED
```

The state machine owns the current logical system state.

Hardware and input components do not directly change that state. Instead, they generate events which are posted to the state-machine queue.

Hardware drivers know **how** to control their hardware, while state handlers determine **when** those actions should occur.

The diagnostics subsystem is deliberately passive. Measurement or display failures currently do not affect actuator control or state-machine safety behaviour.

---

## State Machine

Current states:

```text
STATE_MACHINE_INIT
STATE_MACHINE_STOOWED
STATE_MACHINE_LOWERING
STATE_MACHINE_DEPLOYED
STATE_MACHINE_PUMPING
STATE_MACHINE_RAISING
STATE_MACHINE_POSITION_UNKNOWN
STATE_MACHINE_FAULT
```

> Note: use `STATE_MACHINE_STOWED` above if that is the exact enum name in the source.

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

---

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

The registered failure state is:

```text
STATE_MACHINE_FAULT
```

---

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

`state_machine_common` contains shared event, state ID, and state-object definitions.

`state_machine` contains the generic state-machine engine.

`state_machine_states` contains the application-specific state implementations and registers them with the generic engine during startup.

Individual state files do not directly modify the global state.

---

## Nozzle Servo

The nozzle is driven using an RC-style continuous-rotation servo.

The driver uses the ESP32-S3 **MCPWM** peripheral rather than software-generated PWM.

Current PWM configuration:

```text
PWM frequency:  50 Hz
Period:         20,000 us

Retract:        ~1200 us
Neutral/Stop:   calibrated neutral value
Extend:         ~1800 us
```

The exact neutral value should be maintained in the source configuration and may require final calibration with the production mechanical assembly.

Public driver API:

```c
esp_err_t nozzleServoInit(void);
esp_err_t nozzleServoExtend(void);
esp_err_t nozzleServoRetract(void);
esp_err_t nozzleServoStop(void);
```

The component contains no FreeRTOS task. MCPWM generates the servo waveform directly in hardware.

The servo driver has been bench-tested and integrated with the state-machine lifecycle.

---

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

---

## Limit Switches

Two mechanical limit switches provide endpoint feedback:

```text
Upper limit → nozzle fully stowed
Lower limit → nozzle fully deployed
```

The switches are configured as active-low inputs using GPIO pull-ups:

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
Debounce delay
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

At startup, the component synchronizes the physical mechanism position with the state machine.

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

---

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

The current development timeout is approximately:

```text
10 seconds
```

This remains a development value and will be calibrated using the final mechanical travel time.

---

## RC Input

RC command input is implemented using the ESP32-S3 MCPWM capture peripheral.

The current command path is:

```text
FlySky transmitter
        │
        ▼
FS-iA10B receiver
        │
   PWM channels
        │
        ▼
ESP32-S3 MCPWM Capture
        │
        ▼
RC Input Task
        │
        ▼
State-Machine Events
```

The architecture is also intended to support the production path through the flight controller:

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

The component currently supports multiple RC PWM channels using:

- One shared MCPWM capture timer
- One capture channel per RC input
- One FreeRTOS sample queue
- One processing task
- Per-channel command-state tracking

Current command mappings include:

```text
Pump channel:
    HIGH → PUMP_OFF
    LOW  → PUMP_ON

Nozzle channel:
    HIGH → NOZZLE_RETRACT
    LOW  → NOZZLE_EXTEND
```

Duplicate commands are suppressed so unchanged switch positions do not continuously generate state-machine events.

A software timer detects total PWM disappearance and can generate:

```text
SM_EVENT_RC_SIGNAL_LOST
```

Radio-link failsafe behaviour between the transmitter and receiver is expected to be handled primarily by the Pixhawk in the production system.

The ESP32-side signal-loss mechanism remains as an additional safeguard for physical/AUX PWM disappearance.

### Known RC Input Issue

The RC component is currently under investigation for unstable behaviour when no receiver is connected and the capture input is left floating.

The component has already passed physical integration testing when connected to the receiver, but this disconnected-input condition requires further debugging.

---

## I2C Bus

A shared I2C master-bus component provides access to multiple diagnostic peripherals.

Current bus topology:

```text
ESP32-S3
    │
    ├── INA226   0x40
    │
    └── SSD1306  0x3C
```

The implementation uses the modern ESP-IDF I2C master driver.

The bus component is responsible for:

- Bus initialization
- Device registration
- Address probing
- Data writes
- Combined write/read transactions

Both devices have been tested operating simultaneously on the shared bus.

---

## INA226 Power Monitor

The INA226 provides electrical monitoring for the water-sampling subsystem.

Implemented measurements:

```c
esp_err_t ina226ReadBusVoltage(float* pVoltageV);
esp_err_t ina226ReadShuntVoltage(float* pShuntV);
esp_err_t ina226ReadCurrent(float* pCurrentA);
```

and power measurement:

```c
esp_err_t ina226ReadPower(float* pPowerW);
```

The current development module uses:

```text
Shunt resistor: 0.100 Ω (R100)
I2C address:    0x40
```

The component performs:

- Manufacturer-ID verification
- Die-ID readback
- Calibration-register programming
- Bus-voltage conversion
- Signed shunt-voltage conversion
- Signed current conversion
- Power conversion

Bench testing successfully validated the relationship between measured shunt voltage and calculated current.

The intended production measurement location is on the subsystem battery input:

```text
6S Battery
    │
    ▼
 INA226
 [Shunt]
    │
    ▼
Subsystem Power Junction
    │
    ├── 12 V regulator → Pump
    │
    └── 5 V regulator  → ESP32 + Servo + Diagnostics
```

This allows the INA226 to measure total battery-side power consumption of the complete sampling subsystem rather than only one downstream rail.

The INA226 monitoring is currently diagnostic only and does not directly affect state-machine safety behaviour.

---

## SSD1306 OLED Display

A 128x64 SSD1306 OLED provides local field diagnostics without requiring a serial connection.

Current functionality includes:

- Display initialization
- Display clearing
- 5x7 ASCII font rendering
- Row-based text output
- Shared operation on the I2C bus with the INA226

Public API:

```c
esp_err_t ssd1306Init(void);
esp_err_t ssd1306Clear(void);
esp_err_t ssd1306WriteText(
    uint8_t ubRow,
    const char* pText
);
```

The display is currently powered from the ESP32 3.3 V rail so that the I2C logic remains in the ESP32-safe 3.3 V domain.

---

## Diagnostics

The `diagnostics` component owns a low-priority FreeRTOS task that periodically collects system information and updates the OLED.

Public API:

```c
esp_err_t diagnosticsInit(void);
```

The diagnostics task currently reads:

```text
State-machine state
INA226 bus voltage
INA226 current
INA226 power
```

and displays values similar to:

```text
WATER SAMPLER

STATE: DEPLOYED

BAT: 23.74 V
CUR: 0.243 A
PWR: 5.77 W
```

The display updates periodically without blocking control-system operation.

The diagnostics subsystem is intentionally passive:

```text
Diagnostics failure
        │
        ├── Log warning
        └── Retry later
```

It does not currently generate state-machine faults or stop actuators.

---

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

Diagnostic dependencies are similarly layered:

```text
             i2c_bus
             ▲     ▲
             │     │
          INA226  SSD1306
             ▲     ▲
              \   /
               \ /
            diagnostics
```

The diagnostics component also queries the generic state-machine API for the current system state.

Hardware components such as:

```text
nozzle_servo
pump
motion_timeout
```

are private dependencies of the application-specific state component where required.

The generic `state_machine` component does not depend on individual application states or diagnostic hardware.

---

## Startup Sequence

The initialization sequence is approximately:

```text
nozzleServoInit()
pumpInit()

stateMachineStatesRegister()
stateMachineInit()

motionTimeoutInit()
limitSwitchInit()

rcInputInit()

i2cBusInit()
ina226Init()
ssd1306Init()
diagnosticsInit()

SM_EVENT_SYSTEM_READY
limitSwitchSyncState()
```

During development without a receiver connected, `rcInputInit()` may temporarily be disabled while the disconnected-input stability issue is investigated.

The state-machine event queue is initialized before interrupt-driven event producers begin generating events.

---

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

---

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

---

## Testing

The firmware contains component-level bench tests and state-machine integration tests.

### State Machine

Validated behaviour includes:

- Normal deployment and retraction sequence
- Invalid event handling
- Pump commands rejected outside the deployed state
- Incorrect limit-switch events ignored during motion
- Interrupted motion producing `POSITION_UNKNOWN`
- Global FAULT handling
- Latched fault behaviour
- RESET recovery
- HALT behaviour
- State initialization failure handling
- State deinitialization failure handling
- RC signal-loss state policy

### Hardware and Input Components

Validated behaviour includes:

- MCPWM servo extend/retract/neutral operation
- Pump ON/OFF control
- Upper limit active/released detection
- Lower limit active/released detection
- Mechanical switch debounce
- Startup position synchronization
- Simultaneous upper/lower limit fault detection
- Motion timeout expiry
- Motion timeout cancellation
- Motion timeout restart behaviour
- Lowering timeout → FAULT
- Raising timeout → FA safety
- Successful endpoint detection cancelling timeout
- RC pump command input
- RC nozzle command input
- Physical transmitter-to-pump integration
- Physical transmitter-to-servo integration
- RC PWM disappearance detection

### Diagnostics

Validated diagnostics behaviour includes:

- Shared I2C operation with INA226 and SSD1306
- INA226 manufacturer and die identification
- INA226 bus-voltage measurement
- INA226 shunt-voltage measurement
- INA226 current measurement
- INA226 power measurement
- SSD1306 text rendering
- Live state-machine state updates
- Periodic voltage/current/power display updates

---

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
- RC signal loss stops active pumping
- RC signal loss stops active motion and returns position knowledge to UNKNOWN where appropriate

Diagnostics are currently separated from control safety. INA226 readings are displayed but do not automatically generate fault events.

Hardware-level fail-safe design must also ensure actuator outputs remain safe while the ESP32 is resetting, booting, or unpowered.

---

## Development Roadmap

### Completed

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
13. Implement RC/PWM capture
14. Implement multi-channel RC input
15. Add RC signal-loss detection
16. Integrate RC commands with state-machine events
17. Complete physical RC pump/nozzle integration testing
18. Implement shared I2C bus
19. Implement INA226 voltage/current/power monitoring
20. Implement SSD1306 OLED display
21. Implement periodic diagnostics task
22. Integrate live state and power telemetry on OLED

### Next

23. Debug RC-input behaviour with the receiver disconnected
24. Revalidate RC signal-loss handling
25. Test the complete mechanical spool/nozzle assembly
26. Calibrate final servo and motion-timeout values
27. Validate INA226 measurements on the full subsystem
28. Finalize production PCB power and diagnostic architecture
29. Add additional safety or diagnostic behaviour as project requirements are finalized

---

## Design Philosophy

The firmware follows a few core rules:

- **Events describe what happened**
- **States describe the current system condition**
- **Only the state machine owns the current state**
- **Hardware drivers know how to control hardware, not when**
- **Input components post events rather than directly commanding actuators**
- **State entry and exit callbacks own state-specific hardware actions**
- **Safe states explicitly establish safe actuator outputs**
- **Diagnostics observe the control system without owning it**
- **RTOS tasks are only added where concurrency is actually required**

This keeps the firmware modular, testable, and scalable as additional inputs, diagnostics, and safety features are added.