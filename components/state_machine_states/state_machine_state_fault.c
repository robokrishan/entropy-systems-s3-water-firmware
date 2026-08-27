#include "state_machine_common.h"

#include "esp_log.h"

static const char *TAG = "SM_FAULT";


StateMachineStateId_t stateFaultProcessEvent(const StateMachineEvent_t *pEvent) {
    switch (pEvent->eId) {

        case SM_EVENT_RESET:
            /*
             * Clear the fault, but do not assume that the
             * nozzle is at a known physical position.
             *
             * The system must re-establish its position
             * using one of the limit switches.
             */
            ESP_LOGI(TAG,"Fault reset requested. Nozzle position must be re-established");

            return STATE_MACHINE_POSITION_UNKNOWN;


        default:
            ESP_LOGW(TAG, "Event %s ignored while in FAULT", stateMachineEventName(*pEvent));

            return STATE_MACHINE_FAULT;
    }
}