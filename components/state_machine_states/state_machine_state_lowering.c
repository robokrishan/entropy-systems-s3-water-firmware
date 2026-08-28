#include "state_machine_state_lowering.h"
#include "nozzle_servo.h"
#include "esp_log.h"

static const char *TAG = "SM_LOWERING";


/* state initialization */
static esp_err_t s_stateInit(void) {
    esp_err_t lErr = ESP_OK;

    lErr = nozzleServoExtend();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init state. Code: 0x%X", lErr);
    }

    return lErr;
}


/* state deinitialization */
static esp_err_t s_stateDeinit(void) {
    esp_err_t lErr = ESP_OK;
    
    lErr = nozzleServoStop();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to deinit state. Code: 0x%X", lErr);
    }

    return lErr;
}


/* event processing */
static void s_stateProcess(StateMachineEvent_t *pEvent) {
    switch (pEvent->eId) {

        case SM_EVENT_LOWER_LIMIT_ACTIVE:
            /*
             * The lower limit switch confirms that the
             * nozzle is fully extended.
             */
            ESP_LOG_EVENT(*pEvent);
            break;


        case SM_EVENT_STOP_SPOOL:
            /*
             * The spool was stopped before reaching a
             * known endpoint, so the nozzle position
             * can no longer be considered known.
             */
            ESP_LOG_EVENT(*pEvent);
            break;


        case SM_EVENT_UPPER_LIMIT_RELEASED:
            /*
             * This is expected shortly after beginning
             * movement if the nozzle initially started
             * at the upper limit.
             *
             * No state transition is required.
             */
            ESP_LOG_EVENT(*pEvent);
            break;


        default:
            ESP_LOGW(TAG, "Event %s ignored while in LOWERING",
                stateMachineEventName(*pEvent)
            );

            break;
    }
}


/* next state */
static StateMachineStateId_t s_stateNextState(const StateMachineEvent_t* pEvent) {
    switch(pEvent->eId) {
        case SM_EVENT_LOWER_LIMIT_ACTIVE:
            return STATE_MACHINE_DEPLOYED;

        case SM_EVENT_STOP_SPOOL:
            return STATE_MACHINE_POSITION_UNKNOWN;

        default:
            return STATE_MACHINE_LOWERING;
    }
}

/* state definition */
StateMachineState_t g_stateMachineStateLowering = {
    .eState = STATE_MACHINE_LOWERING,
    .cbInit = s_stateInit,
    .cbDeinit = s_stateDeinit,
    .cbProcess = s_stateProcess,
    .cbNextState = s_stateNextState
};
