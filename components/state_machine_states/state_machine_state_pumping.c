#include "state_machine_common.h"

#include "esp_log.h"

static const char *TAG = "SM_PUMPING";


StateMachineStateId_t statePumpingProcessEvent(
    const StateMachineEvent_t *pEvent
)
{
    switch (pEvent->eId) {

        case SM_EVENT_PUMP_OFF:
            ESP_LOGI(
                TAG,
                "Stopping pump"
            );

            return STATE_MACHINE_DEPLOYED;


        case SM_EVENT_RC_SIGNAL_LOST:
            /*
             * Fail-safe behaviour:
             * loss of the RC control signal must stop the pump.
             *
             * The nozzle position is still known to be fully
             * deployed, so return to DEPLOYED rather than
             * POSITION_UNKNOWN.
             */
            ESP_LOGW(TAG, "RC signal lost. Stopping pump");

            return STATE_MACHINE_DEPLOYED;


        case SM_EVENT_LOWER_LIMIT_ACTIVE:
            /*
             * The nozzle is already known to be deployed.
             * Repeated lower-limit events do not require
             * a state transition.
             */
            return STATE_MACHINE_PUMPING;


        default:
            ESP_LOGW(TAG, "Event %s ignored while in PUMPING",
                stateMachineEventName(*pEvent)
            );

            return STATE_MACHINE_PUMPING;
    }
}