#include "state_machine_state_raising.h"
#include "nozzle_servo.h"
#include "esp_log.h"

static const char *TAG = "SM_RAISING";


/* state initialization */
static esp_err_t s_stateInit(void) {
    esp_err_t lErr = ESP_OK;

    lErr = nozzleServoRetract();
    if(lErr) {
        ESP_LOGE(TAG, "failed to init state. Code: 0x%X", lErr);
    }

    return lErr;
}


/* state deinitialization */
static esp_err_t s_stateDeinit(void) {
    esp_err_t lErr = ESP_OK;

    lErr = nozzleServoStop();
    if(lErr) {
        ESP_LOGE(TAG, "failed to deinit state. Code: 0x%X", lErr);
    }

    return lErr;
}


/* event processing */
static void s_stateProcess(StateMachineEvent_t *pEvent) {
    switch (pEvent->eId) {

        case SM_EVENT_UPPER_LIMIT_ACTIVE:
            /*
             * The upper limit switch confirms that the
             * nozzle is fully retracted/stowed.
             */
            ESP_LOG_EVENT(*pEvent);
            break;


        case SM_EVENT_STOP_SPOOL:
            /*
             * The spool was stopped before reaching the
             * upper limit, so the exact nozzle position
             * is no longer known.
             */
            ESP_LOG_EVENT(*pEvent);
            break;


        case SM_EVENT_LOWER_LIMIT_RELEASED:
            /*
             * Expected shortly after beginning movement
             * away from the fully deployed position.
             *
             * No state transition is required.
             */
            ESP_LOG_EVENT(*pEvent);
            break;


        default:
            ESP_LOGW(TAG, "Event %s ignored while in RAISING",
                stateMachineEventName(*pEvent)
            );

            break;
    }
}


/* next state */
static StateMachineStateId_t s_stateNextState(const StateMachineEvent_t *pEvent) {
    switch(pEvent->eId) {
        case SM_EVENT_UPPER_LIMIT_ACTIVE:
            return STATE_MACHINE_STOWED;

        case SM_EVENT_STOP_SPOOL:
            return STATE_MACHINE_POSITION_UNKNOWN;

        default:
            return STATE_MACHINE_RAISING;
    }
}


/* state definition */
StateMachineState_t g_stateMachineStateRaising = {
    .eState = STATE_MACHINE_RAISING,
    .cbInit = s_stateInit,
    .cbDeinit = s_stateDeinit,
    .cbProcess = s_stateProcess,
    .cbNextState = s_stateNextState
};

