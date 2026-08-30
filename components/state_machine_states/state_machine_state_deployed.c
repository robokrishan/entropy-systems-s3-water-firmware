#include "state_machine_state_deployed.h"
#include "state_machine_global_events.h"
#include "nozzle_servo.h"
#include "esp_log.h"

static const char *TAG = "SM_DEPLOYED";


/* state inititalization */
static esp_err_t s_stateInit(void) {
    esp_err_t lErr = ESP_OK;

    lErr = nozzleServoStop();
    if(lErr) {
        ESP_LOGE(TAG, "state init failed. Code: 0x%X", lErr);
    }

    return lErr;
}


/* state deinitialization */
static esp_err_t s_stateDeinit(void) {
    

    return ESP_OK;
}


/* event processing */
static void s_stateProcess(StateMachineEvent_t *pEvent) {

    // global fault event handling
    if(isGlobalEventProcess(pEvent)) {
        return;
    }

    switch (pEvent->eId) {

        case SM_EVENT_PUMP_ON:
            ESP_LOG_EVENT(*pEvent);
            break;

        case SM_EVENT_NOZZLE_RETRACT:
            ESP_LOG_EVENT(*pEvent);
            break;

        case SM_EVENT_HALT:
            ESP_LOG_EVENT(*pEvent);
            break;

        case SM_EVENT_LOWER_LIMIT_ACTIVE:
            /*
             * Already at the lower limit.
             * No state transition required.
             */
            ESP_LOG_EVENT(*pEvent);
            break;

        case SM_EVENT_RC_SIGNAL_LOST:
            ESP_LOG_EVENT(*pEvent);
            break;

        default:
            ESP_LOGW(TAG, "Event %s ignored while in DEPLOYED",
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
        case SM_EVENT_PUMP_ON:
            return STATE_MACHINE_PUMPING;

        case SM_EVENT_NOZZLE_RETRACT:
            return STATE_MACHINE_RAISING;

        default:
            return STATE_MACHINE_DEPLOYED;
    }
}


/* state definition */
StateMachineState_t g_stateMachineStateDeployed = {
    .eState = STATE_MACHINE_DEPLOYED,
    .cbInit = s_stateInit,
    .cbDeinit = s_stateDeinit,
    .cbProcess = s_stateProcess,
    .cbNextState = s_stateNextState
};
