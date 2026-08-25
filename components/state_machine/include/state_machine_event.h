#ifndef COMPONENTS_STATE_MACHINE_EVENT_H_
#define COMPONENTS_STATE_MACHINE_EVENT_H_

#include <stdint.h>
#include "string.h"

typedef enum {
    SM_EVENT_SYSTEM_READY = 0,
    SM_EVENT_HALT,
    SM_EVENT_PUMP_ON,
    SM_EVENT_PUMP_OFF,
    SM_EVENT_LOWER_NOZZLE,
    SM_EVENT_RAISE_NOZZLE,
    SM_EVENT_STOP_SPOOL,
    SM_EVENT_LIMIT_ACTIVE,
    SM_EVENT_LIMIT_RELEASED,
    SM_EVENT_RC_SIGNAL_LOST,
    SM_EVENT_FAULT,
    SM_EVENT_RESET
} StateMachineEventId_t;

typedef struct {
    StateMachineEventId_t eId;
    uint32_t ulData;
} StateMachineEvent_t;

char* stateMachineEventName(StateMachineEvent_t sEvent);

#endif /* COMPONENTS_STATE_MACHINE_EVENT_H_ */