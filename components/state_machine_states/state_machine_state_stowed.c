#include "state_machine_state_stowed.h"
#include "state_machine_global_events.h"
#include "nozzle_servo.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "SM_STOWED";


/* state initialization */
static esp_err_t s_stateInit(void) {
    esp_err_t lErr = ESP_OK;

    lErr = nozzleServoStop();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init state. Code: 0x%X", lErr);
    }

    return lErr;
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

    switch (pEvent->eId) {

        case SM_EVENT_NOZZLE_EXTEND:
            ESP_LOG_EVENT(*pEvent);
            break;


        case SM_EVENT_UPPER_LIMIT_ACTIVE:
            /*
             * Already at the upper limit.
             * No state change required.
             */
            break;


        default:
            ESP_LOGW(TAG, "Event %s ignored while in STOWED", 
                stateMachineEventName(*pEvent)
            );

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
        case SM_EVENT_NOZZLE_EXTEND:
            return STATE_MACHINE_LOWERING;

        default:
            return STATE_MACHINE_STOWED;
    }
}


/* state definition */
StateMachineState_t g_stateMachineStateStowed = {
    .eState = STATE_MACHINE_STOWED,
    .cbInit = s_stateInit,
    .cbDeinit = s_stateDeinit,
    .cbProcess = s_stateProcess,
    .cbNextState = s_stateNextState
};