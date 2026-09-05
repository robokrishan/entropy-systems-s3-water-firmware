#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

#include "stdbool.h"
#include "stdint.h"

#include "state_machine_common.h"


/**
 * @brief Verify that the state machine is currently in the expected state.
 *
 * @param eExpectedState Expected state-machine state.
 *
 * @return true if the current state matches the expected state.
 * @return false otherwise.
 */
bool testCheckState(StateMachineStateId_t eExpectedState);


/**
 * @brief Post an event to the state machine and wait for processing.
 *
 * @param eEvent Event to post.
 * @param ulDelayMs Delay after posting the event, in milliseconds.
 */
void testPostEvent(StateMachineEventId_t eEvent, uint32_t ulDelayMs);


/**
 * @brief Wait until the state machine reaches the expected state.
 *
 * Polls the current state periodically and returns immediately if the
 * state machine enters FAULT before reaching the requested state.
 *
 * @param eExpectedState State to wait for.
 *
 * @return true if the expected state is reached.
 * @return false if FAULT is entered first.
 */
bool testWaitForState(StateMachineStateId_t eExpectedState, uint32_t ulTimeoutMs);


#endif /* TEST_HELPERS_H */