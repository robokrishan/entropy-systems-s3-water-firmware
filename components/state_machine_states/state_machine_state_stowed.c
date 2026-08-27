#include "state_machine_common.h"
#include "state_machine_state_stowed.h"

#include "esp_log.h"

static const char *TAG = "SM_STOWED";


StateMachineStateId_t stateStowedProcessEvent(
    const StateMachineEvent_t *pEvent
)
{
    switch (pEvent->eId) {

        case SM_EVENT_NOZZLE_EXTEND:
            ESP_LOGI(
                TAG,
                "Extending nozzle"
            );

            return STATE_MACHINE_LOWERING;


        case SM_EVENT_UPPER_LIMIT_ACTIVE:
            /*
             * Already at the upper limit.
             * No state change required.
             */
            return STATE_MACHINE_STOWED;


        default:
            ESP_LOGW(
                TAG,
                "Event %s ignored while in STOWED",
                stateMachineEventName(*pEvent)
            );

            return STATE_MACHINE_STOWED;
    }
}