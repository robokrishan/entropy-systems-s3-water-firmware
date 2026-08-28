#ifndef COMPONENTS_STATE_MACHINE_H_
#define COMPONENTS_STATE_MACHINE_H_

#include "esp_err.h"
#include "state_machine_common.h"
#include "stdbool.h"


/*
    Initialize the state machine. This function must be called before any
    other state machine functions.

    Returns ESP_OK on success, or an error code on failure.
*/
esp_err_t stateMachineInit(void);


/*
    Register a state with the state machine. This function must be called
    before the state machine can transition to the state.

    Returns ESP_OK on success, or an error code on failure.
*/
esp_err_t stateMachineRegisterState(StateMachineState_t* pState, bool isInitialState);


/*
    Post an event to the state machine.

    Returns ESP_OK on success, or an error code on failure.
*/
esp_err_t stateMachinePostEvent(StateMachineEventId_t eEventId);


/*
    Get the current state of the state machine.

    Returns the current state ID.
*/
StateMachineStateId_t stateMachineGetCurrentState(void);

#endif /* COMPONENTS_STATE_MACHINE_H_ */
