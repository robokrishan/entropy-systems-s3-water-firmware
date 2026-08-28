#include "state_machine_global_events.h"
#include "esp_log.h"


static const char* TAG = "SM_GLOBAL";


bool isGlobalEventProcess(StateMachineEvent_t* pEvent) {
    if(NULL == pEvent) {
        ESP_LOGE(TAG, "Invalid event pointer");
        return false;
    }

    switch(pEvent->eId) {
        case SM_EVENT_FAULT:
            ESP_LOGI(TAG, "Processing global event: %s", stateMachineEventName(*pEvent));
            return true;

        default:
            return false;
    }
}

bool stateMachineGlobalEventGetNextState(
    const StateMachineEvent_t* pEvent, 
    StateMachineStateId_t* pNextState
) {
    if((NULL == pEvent) || (NULL == pNextState)) {
        ESP_LOGE(TAG, "Invalid argument");
        return false;
    }

    switch(pEvent->eId) {
        case SM_EVENT_FAULT:
            *pNextState = STATE_MACHINE_FAULT;
            return true;

        default:
            return false;
    }
}