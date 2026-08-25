#include "state_machine_state.h"

char* StateMachineStateName(StateMachineState_t eState) {
    switch(eState) {
        case STATE_MACHINE_INIT:        return "STATE_MACHINE_INIT";
        case STATE_MACHINE_IDLE:        return "STATE_MACHINE_IDLE";
        case STATE_MACHINE_LOWERING:    return "STATE_MACHINE_LOWERING";
        case STATE_MACHINE_PUMPING:     return "STATE_MACHINE_PUMPING";
        case STATE_MACHINE_RAISING:     return "STATE_MACHINE_RAISING";
        case STATE_MACHINE_FAULT:       return "STATE_MACHINE_FAULT";
        default:                        return "unknown state";
    }
}