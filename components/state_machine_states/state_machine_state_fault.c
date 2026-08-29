#include "state_machine_state_fault.h"
#include "nozzle_servo.h"
#include "pump.h"
#include "esp_log.h"


static const char *TAG = "SM_FAULT";


/* state initialization */
static esp_err_t s_stateInit(void) {
    esp_err_t lErr = ESP_OK;

    /* put all actuators in safe mode */
    lErr = nozzleServoStop();
    if(lErr) {
        ESP_LOGE(TAG, "failed to init state. Code: 0x%X", lErr);
        goto end_state_init;
    }

    lErr = pumpOff();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to stop pump. Code: 0x%X", lErr);
        goto end_state_init;
    }

end_state_init:

    return lErr;
}


/* state deinitialization */
static esp_err_t s_stateDeinit(void) {
    

    return ESP_OK;
}


/* event processing */
static void s_stateProcess(StateMachineEvent_t* pEvent) {
    switch (pEvent->eId) {

        case SM_EVENT_RESET:
            /*
             * Clear the fault, but do not assume that the
             * nozzle is at a known physical position.
             *
             * The system must re-establish its position
             * using one of the limit switches.
             */
            ESP_LOG_EVENT(*pEvent);
            break;


        case SM_EVENT_FAULT:
            /*
                already in fault state, remain in FAULT
            */
            ESP_LOG_EVENT(*pEvent);
            break;

        case SM_EVENT_HALT:
            ESP_LOG_EVENT(*pEvent);
            break;

        default:
            ESP_LOGW(TAG, "Event %s ignored while in FAULT", 
                stateMachineEventName(*pEvent));
            
            break;
    }
}


/* next state */
static StateMachineStateId_t s_stateNextState(const StateMachineEvent_t* pEvent) {
    switch(pEvent->eId) {
        case SM_EVENT_RESET:
            return STATE_MACHINE_POSITION_UNKNOWN;

        default:
            return STATE_MACHINE_FAULT;
    }
}


/* state definition */
StateMachineState_t g_stateMachineStateFault = {
    .eState = STATE_MACHINE_FAULT,
    .cbInit = s_stateInit,
    .cbDeinit = s_stateDeinit,
    .cbProcess = s_stateProcess,
    .cbNextState = s_stateNextState
};
