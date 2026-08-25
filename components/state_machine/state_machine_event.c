#include "state_machine_event.h"

char* stateMachineEventName(StateMachineEvent_t sEvent) {
    switch(sEvent.eId) {
        case SM_EVENT_SYSTEM_READY:     return "SM_EVENT_SYSTEM_READY";
        case SM_EVENT_HALT:             return "SM_EVENT_HALT";
        case SM_EVENT_PUMP_ON:          return "SM_EVENT_PUMP_ON";
        case SM_EVENT_PUMP_OFF:         return "SM_EVENT_PUMP_OFF";
        case SM_EVENT_LOWER_NOZZLE:     return "SM_EVENT_LOWER_NOZZLE";
        case SM_EVENT_RAISE_NOZZLE:     return "SM_EVENT_RAISE_NOZZLE";
        case SM_EVENT_STOP_SPOOL:       return "SM_EVENT_STOP_SPOOL";
        case SM_EVENT_LIMIT_ACTIVE:     return "SM_EVENT_LIMIT_ACTIVE";
        case SM_EVENT_LIMIT_RELEASED:   return "SM_EVENT_LIMIT_RELEASED";
        case SM_EVENT_RC_SIGNAL_LOST:   return "SM_EVENT_RC_SIGNAL_LOST";
        case SM_EVENT_FAULT:            return "SM_EVENT_FAULT";
        case SM_EVENT_RESET:            return "SM_EVENT_RESET";
        default:                        return "unknown event";
    }
}

