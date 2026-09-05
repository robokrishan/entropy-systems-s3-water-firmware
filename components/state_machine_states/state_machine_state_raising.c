#include "state_machine_state_raising.h"
#include "state_machine_global_events.h"
#include "nozzle_servo.h"
#include "esp_log.h"
#include "motion_timeout.h"

static const char *TAG = "SM_RAISING";


/* state initialization */
static esp_err_t s_stateInit(void) {
    esp_err_t lErr = ESP_OK;

    lErr = nozzleServoRetract();
    if(lErr) {
        ESP_LOGE(TAG, "failed to init state. Code: 0x%X", lErr);
        
        return lErr;
    }

    lErr = nozzleServoEnable();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to enable nozzle servo. Code: 0x%X", lErr);
        
        return lErr;
    }

    lErr = motionTimeoutStart();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to start motion timeout. Code: 0x%X", lErr);

        nozzleServoDisable();

        return lErr;
    }

    return ESP_OK;
}


/* state deinitialization */
static esp_err_t s_stateDeinit(void) {
    esp_err_t lErr = ESP_OK;
    esp_err_t lTimeoutErr = ESP_OK;
    
    lErr = nozzleServoDisable();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to disable nozzle servo. Code: 0x%X", lErr);
    }

    lTimeoutErr = motionTimeoutStop();
    if(lTimeoutErr) {
        ESP_LOGE(TAG, "Failed to stop motion timeout. Code: 0x%X", lTimeoutErr);

        if(ESP_OK == lErr) {
            lErr = lTimeoutErr;
        }
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

        
        case SM_EVENT_RC_SIGNAL_LOST:
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

        case SM_EVENT_HALT:
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

    // global fault event handling
    StateMachineStateId_t eNextState;
    if(stateMachineGlobalEventGetNextState(pEvent, &eNextState)) {
        return eNextState;
    }
    
    switch(pEvent->eId) {
        case SM_EVENT_UPPER_LIMIT_ACTIVE:
            return STATE_MACHINE_STOWED;

        case SM_EVENT_HALT:
        case SM_EVENT_STOP_SPOOL:
        case SM_EVENT_RC_SIGNAL_LOST:
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

