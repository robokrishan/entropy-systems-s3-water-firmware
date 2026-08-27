#include "state_machine_common.h"

#include "esp_log.h"

static const char *TAG = "SM_RAISING";


StateMachineStateId_t stateRaisingProcessEvent(
    const StateMachineEvent_t *pEvent
)
{
    switch (pEvent->eId) {

        case SM_EVENT_UPPER_LIMIT_ACTIVE:
            /*
             * The upper limit switch confirms that the
             * nozzle is fully retracted/stowed.
             */
            ESP_LOGI(TAG, "Upper limit reached. Nozzle fully stowed");

            return STATE_MACHINE_STOWED;


        case SM_EVENT_STOP_SPOOL:
            /*
             * The spool was stopped before reaching the
             * upper limit, so the exact nozzle position
             * is no longer known.
             */
            ESP_LOGI(TAG, "Spool stopped before upper limit");

            return STATE_MACHINE_POSITION_UNKNOWN;


        case SM_EVENT_UPPER_LIMIT_RELEASED:
            /*
             * No transition required.
             * This event can simply be ignored while raising.
             */
            return STATE_MACHINE_RAISING;


        default:
            ESP_LOGW(TAG, "Event %s ignored while in RAISING",
                stateMachineEventName(*pEvent)
            );

            return STATE_MACHINE_RAISING;
    }
}