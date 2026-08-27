#ifndef COMPONENTS_STATE_MACHINE_COMMON_H_
#define COMPONENTS_STATE_MACHINE_COMMON_H_

#include <stdint.h>
#include "esp_err.h"

#define ESP_LOG_EVENT(x)    ESP_LOGI(TAG, "%s", stateMachineEventName(x))


/** State Machine Event IDs */
typedef enum {
    SM_EVENT_SYSTEM_READY = 0,

    SM_EVENT_HALT,

    SM_EVENT_PUMP_ON,
    SM_EVENT_PUMP_OFF,

    SM_EVENT_NOZZLE_EXTEND,
    SM_EVENT_NOZZLE_RETRACT,
    SM_EVENT_STOP_SPOOL,

    SM_EVENT_UPPER_LIMIT_ACTIVE,
    SM_EVENT_UPPER_LIMIT_RELEASED,
    SM_EVENT_LOWER_LIMIT_ACTIVE,
    SM_EVENT_LOWER_LIMIT_RELEASED,

    SM_EVENT_RC_SIGNAL_LOST,

    SM_EVENT_FAULT,
    SM_EVENT_RESET
} StateMachineEventId_t;


/** State Machine State IDs */
typedef enum {
    STATE_MACHINE_INIT = 0,

    STATE_MACHINE_STOWED,
    STATE_MACHINE_LOWERING,
    STATE_MACHINE_DEPLOYED,
    STATE_MACHINE_PUMPING,
    STATE_MACHINE_RAISING,
    STATE_MACHINE_POSITION_UNKNOWN,
    STATE_MACHINE_FAULT,

    STATE_MACHINE_STATE_MAX
} StateMachineStateId_t;


/* State Machine Event Structure  */
typedef struct {
    StateMachineEventId_t eId;
    uint32_t ulData;
} StateMachineEvent_t;


/* State Init Callback*/
typedef esp_err_t (*StateMachineStateInit_t)(void);


/* State Deinit Callback */
typedef esp_err_t (*StateMachineStateDeinit_t)(void);


/* State Process Callback */
typedef void (*StateMachineStateProcess_t)(StateMachineEvent_t* pEvent);


/* State Next State Callback */
typedef StateMachineStateId_t (*StateMachineStateNextState_t)(const StateMachineEvent_t* pEvent);


/* State Machine State Structure */
typedef struct {
    StateMachineStateId_t eState;
    StateMachineStateInit_t cbInit;
    StateMachineStateDeinit_t cbDeinit;
    StateMachineStateProcess_t cbProcess;
    StateMachineStateNextState_t cbNextState;
} StateMachineState_t;


/* Public Function Prototypes */
const char* stateMachineEventName(StateMachineEvent_t sEvent);

const char* stateMachineStateName(StateMachineStateId_t eState);

#endif /* COMPONENTS_STATE_MACHINE_COMMON_H_ */
