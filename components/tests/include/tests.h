#ifndef COMPONENTS_TESTS_H_
#define COMPONENTS_TESTS_H_

typedef enum {
    TEST_NORMAL_SEQUENCE = 0,
    TEST_WRONG_SEQUENCE,
    TEST_FAULT_SEQUENCE,
    TEST_STATE_INIT_FAILURE,
    TEST_STATE_DEINIT_FAILURE,
    TEST_HALT_SEQUENCE,

    TEST_NOZZLE_SERVO_SEQUENCE,
    TEST_NOZZLE_SERVO_NEUTRAL,
    TEST_PUMP_SEQUENCE,

    TEST_LIMIT_SWITCH_SEQUENCE,
    TEST_MOTION_TIMEOUT_SEQUENCE,
    TEST_MOTION_TIMEOUT_INTEGRATION_SEQUENCE,

    TEST_RC_SIGNAL_LOSS_SEQUENCE,
    TEST_RC_SIGNAL_LOSS_FSM_SEQUENCE,
    TEST_RC_PUMP_INTEGRATION_SEQUENCE,
    TEST_RC_NOZZLE_INTEGRATION_SEQUENCE,

    TEST_INA226_BASIC,
    TEST_SSD1306_BASIC,
    TEST_SSD1306_WRITE_TEXT,
    TEST_SSD1306_DIAGNOSTICS,

    TEST_ALL_AUTOMATIC
} TestId_t;



/**
 * @brief Test the complete nominal state-machine operating sequence.
 *
 * Verifies normal system startup, nozzle deployment, pumping,
 * and nozzle retraction through the expected state transitions.
 *
 * The test initializes and deinitializes all required components internally.
 */
void testNormalSequence(void);

/**
 * @brief Test handling of invalid and out-of-sequence events.
 *
 * Verifies that invalid actuator commands and incorrect limit events
 * are rejected without causing unintended state transitions. Also
 * verifies interrupted motion, fault latching, and reset recovery.
 *
 * The test initializes and deinitializes all required components internally.
 */
void testWrongSequence(void);

/**
 * @brief Test fault handling and recovery during nozzle movement.
 *
 * Injects a fault while the nozzle is moving and verifies that the
 * system enters FAULT, rejects normal commands while faulted, recovers
 * through RESET, and can subsequently resume normal operation.
 *
 * The test initializes and deinitializes all required components internally.
 */
void testFaultSequence(void);


/**
 * @brief Test state-machine behavior when a state initialization callback fails.
 *
 * Requires the LOWERING state initialization callback to be deliberately
 * configured to return an error. Verifies transition to FAULT, fault
 * latching, and recovery through RESET.
 *
 * The test initializes and deinitializes all required components internally.
 */
void testStateInitFailure(void);


/**
 * @brief Test state-machine behavior when a state deinitialization callback fails.
 *
 * Requires the LOWERING state deinitialization callback to be deliberately
 * configured to return an error. Verifies that the failed transition forces
 * the state machine into FAULT and that recovery through RESET remains possible.
 *
 * The test initializes and deinitializes all required components internally.
 */
void testStateDeinitFailure(void);


/**
 * @brief Test HALT handling across relevant state-machine states.
 *
 * Verifies that HALT leaves stationary states unchanged, interrupts active
 * movement by transitioning to POSITION_UNKNOWN, stops pumping while retaining
 * the known deployed position, and does not clear a latched fault.
 *
 * The test initializes and deinitializes all required components internally.
 */
void testHaltSequence(void);


/**
 * @brief Test the nozzle servo movement sequence.
 *
 * Initializes the nozzle servo component and verifies the expected actuation
 * sequence by commanding extension, disabling the PWM output, commanding
 * retraction, and disabling the PWM output again.
 *
 * The test uses timed delays so that each movement and stop condition can be
 * observed physically. The servo component is deinitialized before the test
 * exits, including after an intermediate failure.
 */
void testNozzleServoSequence(void);


/**
 * @brief Test the nozzle servo neutral PWM behavior.
 *
 * Initializes the nozzle servo component, applies the configured neutral pulse
 * width, and keeps the PWM output enabled for an extended period so that any
 * unintended movement or jitter can be observed.
 *
 * The servo output is disabled and the component is deinitialized before the
 * test exits.
 */
void testNozzleServoNeutral(void);


/**
 * @brief Test basic pump driver operation.
 *
 * Initializes the pump component, verifies that it starts in the OFF state,
 * commands the pump ON for a short period, then turns it OFF again.
 *
 * The pump component is deinitialized before the test exits, ensuring the
 * output is returned to a safe OFF state even if an intermediate operation
 * fails.
 */
void testPumpSequence(void);


/**
 * @brief Test limit-switch integration with the state machine.
 *
 * Verifies upper and lower limit-switch detection, startup position
 * synchronization, and the expected state transitions when each physical
 * limit is activated and released.
 *
 * The test also verifies that simultaneous activation of both limit switches
 * triggers FAULT, that the fault remains latched after the switches are
 * released, and that RESET returns the system to POSITION_UNKNOWN.
 *
 * The test initializes and deinitializes all required components internally.
 */
void testLimitSwitchSequence(void);


/**
 * @brief Test basic motion-timeout behavior.
 *
 * Verifies that an active motion timeout triggers the expected fault event,
 * that a stopped timeout does not trigger a fault, and that restarting an
 * active timeout resets its expiry period.
 *
 * The test also verifies fault recovery through RESET.
 *
 * The test initializes and deinitializes all required components internally.
 */
void testMotionTimeoutSequence(void);


/**
 * @brief Test motion-timeout integration during nozzle movement.
 *
 * Verifies that nozzle movement enters FAULT when the configured motion
 * timeout expires before the expected limit switch is reached.
 *
 * The test exercises both lowering and raising timeout scenarios and also
 * verifies that reaching the expected limit switch before timeout completes
 * the movement normally without a later timeout fault.
 *
 * The test initializes and deinitializes all required components internally.
 */
void testMotionTimeoutIntegrationSequence(void);


/**
 * @brief Test RC input signal-loss detection and state-machine response.
 *
 * Verifies that loss of valid RC PWM input is detected after the configured
 * signal timeout and converted into SM_EVENT_RC_SIGNAL_LOST.
 *
 * The test confirms that signal loss while pumping stops the pump and returns
 * the system to DEPLOYED, and that RC input processing resumes when valid
 * signals are restored.
 *
 * The test initializes and deinitializes all required base and RC input
 * components internally.
 */
void testRcSignalLossSequence(void);


/**
 * @brief Test state-machine handling of RC signal loss.
 *
 * Verifies the expected FSM response when SM_EVENT_RC_SIGNAL_LOST is received
 * while lowering, pumping, raising, and while already in FAULT.
 *
 * Confirms that movement is interrupted safely, pumping stops while preserving
 * the deployed position, faults remain latched, and normal recovery through
 * SM_EVENT_RESET is still possible.
 *
 * The test initializes and deinitializes all required base components
 * internally.
 */
void testRcSignalLossFsmSequence(void);


/**
 * @brief Test integration between RC pump input and the state machine.
 *
 * Verifies that physical RC switch changes are converted into the appropriate
 * pump events and handled correctly by the FSM.
 *
 * Confirms that PUMP_ON is rejected while STOWED, accepted while DEPLOYED,
 * transitions the system into PUMPING, and that PUMP_OFF returns the system
 * to DEPLOYED. The test also verifies duplicate RC input suppression by
 * holding the switch in a fixed position.
 *
 * The test initializes and deinitializes all required base and RC input
 * components internally.
 */
void testRcPumpIntegrationSequence(void);


/**
 * @brief Test end-to-end RC control of the nozzle mechanism.
 *
 * Verifies integration between the RC input, state machine, nozzle servo,
 * motion timeout, and physical limit switches.
 *
 * The test begins with the nozzle at the upper limit, waits for STOWED,
 * commands extension through the RC transmitter, waits for the lower limit
 * and DEPLOYED state, then commands retraction and waits for the upper limit
 * and STOWED state.
 *
 * The test initializes and deinitializes all required components internally
 * and aborts safely if an expected state is not reached within its timeout.
 */
void testRcNozzleIntegrationSequence(void);


/**
 * @brief Test basic INA226 initialization and measurement reads.
 *
 * Initializes the shared I2C bus and INA226 sensor, then verifies that
 * bus voltage, shunt voltage, current, and power measurements can be read
 * successfully from the device.
 *
 * The INA226 component and I2C bus are deinitialized before the test exits.
 */
void testIna226Basic(void);


/**
 * @brief Test basic SSD1306 initialization.
 *
 * Initializes the shared I2C bus and SSD1306 display to verify that the
 * display can be added to the bus and configured successfully.
 *
 * The SSD1306 component and I2C bus are deinitialized before the test exits.
 */
void testSsd1306Basic(void);


/**
 * @brief Test text output on the SSD1306 display.
 *
 * Initializes the shared I2C bus and SSD1306 display, then writes several
 * test strings to different display rows to verify basic text rendering.
 *
 * The SSD1306 component and I2C bus are deinitialized before the test exits,
 * including when an intermediate write operation fails.
 */
void testSsd1306WriteText(void);


/**
 * @brief Test INA226 measurement output on the SSD1306 diagnostics display.
 *
 * Initializes the shared I2C bus, INA226 sensor, and SSD1306 display,
 * reads the current voltage, current, and power measurements, and renders
 * the formatted values on the OLED.
 *
 * All initialized components are deinitialized before the test exits,
 * including when an intermediate operation fails.
 */
void testSsd1306Diagnostics(void);


#endif /* COMPONENTS_TESTS_H_ */