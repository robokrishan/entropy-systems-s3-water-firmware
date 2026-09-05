#ifndef COMPONENTS_TESTS_H_
#define COMPONENTS_TESTS_H_

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


void testPumpSequence(void);


void testLimitSwitchSequence(void);


void testMotionTimeoutSequence(void);


void testMotionTimeoutIntegrationSequence(void);


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


void testIna226Basic(void);


void testSsd1306Basic(void);


void testSsd1306WriteText(void);


void testSsd1306Diagnostics(void);


#endif /* COMPONENTS_TESTS_H_ */