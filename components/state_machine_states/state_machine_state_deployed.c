#include "state_machine_common.h"

#include "esp_log.h"

static const char *TAG = "SM_DEPLOYED";


StateMachineState_t stateDeployedProcessEvent(
    const StateMachineEvent_t *pEvent
)
{
    switch (pEvent->eId) {

        case SM_EVENT_PUMP_ON:
            ESP_LOGI(TAG, "Starting pump");

            return STATE_MACHINE_PUMPING;


        case SM_EVENT_NOZZLE_RETRACT:
            ESP_LOGI(TAG, "Retracting nozzle");

            return STATE_MACHINE_RAISING;


        case SM_EVENT_LOWER_LIMIT_ACTIVE:
            /*
             * Already at the lower limit.
             * No state transition required.
             */
            return STATE_MACHINE_DEPLOYED;


        default:
            ESP_LOGW(TAG, "Event %s ignored while in DEPLOYED",
                stateMachineEventName(*pEvent)
            );

            return STATE_MACHINE_DEPLOYED;
    }
}