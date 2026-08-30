#include "state_machine_state_pos_unknown.h"
#include "state_machine_global_events.h"
#include "nozzle_servo.h"
#include "esp_log.h"


static const char *TAG = "SM_POS_UNKNOWN";


/* state initialization */
static esp_err_t s_stateInit(void) {
    esp_err_t lErr = ESP_OK;

    lErr = nozzleServoStop();
    if(lErr) {
        ESP_LOGE(TAG, "failed to init state. Code: 0x%X", lErr);
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

        case SM_EVENT_NOZZLE_EXTEND:
            ESP_LOG_EVENT(*pEvent);
            break;


        case SM_EVENT_NOZZLE_RETRACT:
            ESP_LOG_EVENT(*pEvent);
            break;

        
        case SM_EVENT_HALT:
            ESP_LOG_EVENT(*pEvent);
            break;

        
        case SM_EVENT_RC_SIGNAL_LOST:
            ESP_LOG_EVENT(*pEvent);
            break;


        case SM_EVENT_UPPER_LIMIT_ACTIVE:
            /*
             * Upper limit switch provides a known
             * physical reference position.
             */
            ESP_LOG_EVENT(*pEvent);
            break;


        case SM_EVENT_LOWER_LIMIT_ACTIVE:
            /*
             * Lower limit switch provides a known
             * physical reference position.
             */
            ESP_LOG_EVENT(*pEvent);
            break;


        default:
            ESP_LOGW(TAG, "Event %s ignored while nozzle position is unknown",
                stateMachineEventName(*pEvent)
            );

            break;
    }
}


/* next state */
static StateMachineStateId_t s_stateNextState(const StateMachineEvent_t *pEvent) {

    // global fault event handling
    StateMachineStateId_t eNextState;
    if(stateMachineGlobalEventGetNextState(pEvent, &eNextState)) {
        return eNextState;
    }
    
    switch(pEvent->eId) {
        case SM_EVENT_NOZZLE_EXTEND:
            return STATE_MACHINE_LOWERING;

        case SM_EVENT_NOZZLE_RETRACT:
            return STATE_MACHINE_RAISING;

        case SM_EVENT_UPPER_LIMIT_ACTIVE:
            return STATE_MACHINE_STOWED;

        case SM_EVENT_LOWER_LIMIT_ACTIVE:
            return STATE_MACHINE_DEPLOYED;

        default:
            return STATE_MACHINE_POSITION_UNKNOWN;
    }
}


/* state definition */
StateMachineState_t g_stateMachineStatePositionUnknown = {
    .eState = STATE_MACHINE_POSITION_UNKNOWN,
    .cbInit = s_stateInit,
    .cbDeinit = s_stateDeinit,
    .cbProcess = s_stateProcess,
    .cbNextState = s_stateNextState
};