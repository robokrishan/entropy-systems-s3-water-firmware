#ifndef COMPONENTS_TESTS_H_
#define COMPONENTS_TESTS_H_

/**
 * @brief Executes the standard valid operating sequence test.
 * Verifies that the system transitions through the expected normal
 *          states and behaves correctly under the nominal flow sequence.
 */
void testNormalSequence(void);

/**
 * @brief Executes the invalid-sequence test.
 * Confirms the system rejects an unexpected or out-of-order sequence
 *          and raises the appropriate error or safety response.
 */
void testWrongSequence(void);

/**
 * @brief Executes the fault-handling sequence test.
 * Validates that a fault condition is detected, handled, and reported
 *          without allowing the system to continue in a dangerous state.
 */
void testFaultSequence(void);

/**
 * @brief Executes the nozzle servo movement test.
 * Checks that the nozzle servo follows the expected motion profile
 *          during a sequence that exercises actuation behavior.
 */
void testNozzleServoSequence(void);

/**
 * @brief Executes the nozzle servo neutral-position test.
 * Verifies the nozzle servo returns to the neutral position and stays
 *          stable when no actuation is required.
 */
void testNozzlServoNeutral(void);


void testStateInitFailure(void);


void testStateDeinitFailure(void);


void testHaltSequence(void);


void testPumpSequence(void);


void testLimitSwitchSequence(void);


void testMotionTimeoutSequence(void);


void testMotionTimeoutIntegrationSequence(void);


void testRcSignalLossSequence(void);


void testRcSignalLossFsmSequence(void);


void testRcPumpIntegrationSequence(void);


void testRcNozzleIntegrationSequence(void);


void testIna226Basic(void);


void testSsd1306Basic(void);


void testSsd1306WriteText(void);

#endif /* COMPONENTS_TESTS_H_ */