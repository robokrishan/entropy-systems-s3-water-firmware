#include "state_machine_state_init.h"
#include "state_machine_global_events.h"
#include "esp_log.h"


static const char *TAG = "SM_INIT";


/* state initialization */
static esp_err_t s_stateInit(void) {
    

    return ESP_OK;
}


/* state deinitialization */
static esp_err_t s_stateDeinit(void) {


    return ESP_OK;
}


/* event processing */
static void s_stateProcess(StateMachineEvent_t* pEvent) {

    // global fault event handling
    if(isGlobalEventProcess(pEvent)) {
        return;
    }

    switch(pEvent->eId) {
        case SM_EVENT_SYSTEM_READY:
            ESP_LOG_EVENT(*pEvent);
            break;

        default:
            ESP_LOGW(TAG, "Event %s ignored while in INIT", stateMachineEventName(*pEvent));
            break;
    }
}


/* next state */
static StateMachineStateId_t s_stateNextState(const StateMachineEvent_t* pEvent) {

    // global fault event handling
    StateMachineStateId_t eNextState;
    if(stateMachineGlobalEventGetNextState(pEvent, &eNextState)) {
        return eNextState;
    }

    switch(pEvent->eId) {
        case SM_EVENT_SYSTEM_READY:
            return STATE_MACHINE_POSITION_UNKNOWN;

        default:
            return STATE_MACHINE_INIT;
    }
}


/* state definition */
StateMachineState_t g_stateMachineStateInit = {
    .eState = STATE_MACHINE_INIT,
    .cbInit = s_stateInit,
    .cbDeinit = s_stateDeinit,
    .cbProcess = s_stateProcess,
    .cbNextState = s_stateNextState
};