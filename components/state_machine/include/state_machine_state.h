#ifndef COMPONENTS_STATE_MACHINE_STATE_H_
#define COMPONENTS_STATE_MACHINE_STATE_H_

#include "string.h"

typedef enum {
    STATE_MACHINE_INIT = 0,
    STATE_MACHINE_IDLE,
    STATE_MACHINE_LOWERING,
    STATE_MACHINE_PUMPING,
    STATE_MACHINE_RAISING,
    STATE_MACHINE_FAULT
} StateMachineState_t;


char* StateMachineStateName(StateMachineState_t eState);

#endif /* COMPONENTS_STATE_MACHINE_STATE_H_ */