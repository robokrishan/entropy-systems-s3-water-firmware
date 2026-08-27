#include "state_machine_common.h"

const char* stateMachineEventName(StateMachineEvent_t sEvent) {
    switch(sEvent.eId) {
        case SM_EVENT_SYSTEM_READY:         return "SM_EVENT_SYSTEM_READY";
        case SM_EVENT_HALT:                 return "SM_EVENT_HALT";
        case SM_EVENT_PUMP_ON:              return "SM_EVENT_PUMP_ON";
        case SM_EVENT_PUMP_OFF:             return "SM_EVENT_PUMP_OFF";
        case SM_EVENT_NOZZLE_EXTEND:        return "SM_EVENT_NOZZLE_EXTEND";
        case SM_EVENT_NOZZLE_RETRACT:       return "SM_EVENT_NOZZLE_RETRACT";
        case SM_EVENT_STOP_SPOOL:           return "SM_EVENT_STOP_SPOOL";
        case SM_EVENT_UPPER_LIMIT_ACTIVE:   return "SM_EVENT_UPPER_LIMIT_ACTIVE";
        case SM_EVENT_UPPER_LIMIT_RELEASED: return "SM_EVENT_UPPER_LIMIT_RELEASED";
        case SM_EVENT_LOWER_LIMIT_ACTIVE:   return "SM_EVENT_LOWER_LIMIT_ACTIVE";
        case SM_EVENT_LOWER_LIMIT_RELEASED: return "SM_EVENT_LOWER_LIMIT_RELEASED";
        case SM_EVENT_RC_SIGNAL_LOST:       return "SM_EVENT_RC_SIGNAL_LOST";
        case SM_EVENT_FAULT:                return "SM_EVENT_FAULT";
        case SM_EVENT_RESET:                return "SM_EVENT_RESET";

        default:                            return "unknown event";
    }
}

const char* stateMachineStateName(StateMachineStateId_t eState) {
    switch(eState) {
        case STATE_MACHINE_INIT:                return "STATE_MACHINE_INIT";
        case STATE_MACHINE_STOWED:              return "STATE_MACHINE_STOWED";
        case STATE_MACHINE_LOWERING:            return "STATE_MACHINE_LOWERING";
        case STATE_MACHINE_DEPLOYED:            return "STATE_MACHINE_DEPLOYED";
        case STATE_MACHINE_PUMPING:             return "STATE_MACHINE_PUMPING";
        case STATE_MACHINE_RAISING:             return "STATE_MACHINE_RAISING";
        case STATE_MACHINE_POSITION_UNKNOWN:    return "STATE_MACHINE_POSITION_UNKNOWN";
        case STATE_MACHINE_FAULT:               return "STATE_MACHINE_FAULT";
        default:                                return "unknown state";
    }
}
