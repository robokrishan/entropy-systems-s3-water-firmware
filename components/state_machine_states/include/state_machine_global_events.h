#ifndef COMPONENTS_STATE_MACHINE_GLOBAL_EVENTS_H_
#define COMPONENTS_STATE_MACHINE_GLOBAL_EVENTS_H_

#include "stdbool.h"
#include "state_machine_common.h"


bool isGlobalEventProcess(StateMachineEvent_t* pEvent);

bool stateMachineGlobalEventGetNextState(const StateMachineEvent_t* pEvent, StateMachineStateId_t* pNextState);


#endif /* COMPONENTS_STATE_MACHINE_GLOBAL_EVENTS_H_ */