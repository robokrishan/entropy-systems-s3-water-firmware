#include "state_machine_state_pumping.h"
#include "state_machine_global_events.h"
#include "pump.h"
#include "esp_log.h"

static const char *TAG = "SM_PUMPING";


/* state initialization */
static esp_err_t s_stateInit(void) {
    esp_err_t lErr = ESP_OK;

    lErr = pumpOn();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init state. Code: 0x%X", lErr);
    }

    return lErr;
}


/* state deinitialization */
static esp_err_t s_stateDeinit(void) {
    esp_err_t lErr = ESP_OK;

    lErr = pumpOff();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to deinit state. Code: 0x%X", lErr);
    }

    return lErr;
}


/* event processing */
static void s_stateProcess(StateMachineEvent_t *pEvent) {

    // global fault event handling
    if(isGlobalEventProcess(pEvent)) {
        return;
    }

    switch (pEvent->eId) {

        case SM_EVENT_PUMP_OFF:
            ESP_LOG_EVENT(*pEvent);
            break;

        case SM_EVENT_RC_SIGNAL_LOST:
            /*
             * Fail-safe behaviour:
             * loss of the RC control signal must stop the pump.
             *
             * The nozzle position is still known to be fully
             * deployed, so return to DEPLOYED rather than
             * POSITION_UNKNOWN.
             */
            ESP_LOG_EVENT(*pEvent);
            ESP_LOGW(TAG, "RC signal lost. Stopping pump");
            break;


        case SM_EVENT_LOWER_LIMIT_ACTIVE:
            /*
             * The nozzle is already known to be deployed.
             * Repeated lower-limit events do not require
             * a state transition.
             */
            ESP_LOG_EVENT(*pEvent);
            break;

        
        case SM_EVENT_HALT:
            ESP_LOG_EVENT(*pEvent);
            break;


        default:
            ESP_LOGW(TAG, "Event %s ignored while in PUMPING",
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
        case SM_EVENT_HALT:
        case SM_EVENT_PUMP_OFF:
        case SM_EVENT_RC_SIGNAL_LOST:
            return STATE_MACHINE_DEPLOYED;

        default:
            return STATE_MACHINE_PUMPING;
    }
}


/* state definition */
StateMachineState_t g_stateMachineStatePumping = {
    .eState = STATE_MACHINE_PUMPING,
    .cbInit = s_stateInit,
    .cbDeinit = s_stateDeinit,
    .cbProcess = s_stateProcess,
    .cbNextState = s_stateNextState
};
