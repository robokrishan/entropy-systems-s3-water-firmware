#include "state_machine_common.h"

#include "esp_log.h"

static const char *TAG = "SM_LOWERING";


StateMachineState_t stateLoweringProcessEvent(
    const StateMachineEvent_t *pEvent
)
{
    switch (pEvent->eId) {

        case SM_EVENT_LOWER_LIMIT_ACTIVE:
            /*
             * The lower limit switch confirms that the
             * nozzle is fully extended.
             */
            ESP_LOGI(TAG, "Lower limit reached. Nozzle fully deployed");

            return STATE_MACHINE_DEPLOYED;


        case SM_EVENT_STOP_SPOOL:
            /*
             * The spool was stopped before reaching a
             * known endpoint, so the nozzle position
             * can no longer be considered known.
             */
            ESP_LOGI(TAG, "Spool stopped before lower limit");

            return STATE_MACHINE_POSITION_UNKNOWN;


        case SM_EVENT_LOWER_LIMIT_RELEASED:
            /*
             * This is expected shortly after beginning
             * movement if the nozzle initially started
             * at the lower limit.
             *
             * No state transition is required.
             */
            return STATE_MACHINE_LOWERING;


        default:
            ESP_LOGW(TAG, "Event %s ignored while in LOWERING",
                stateMachineEventName(*pEvent)
            );

            return STATE_MACHINE_LOWERING;
    }
}