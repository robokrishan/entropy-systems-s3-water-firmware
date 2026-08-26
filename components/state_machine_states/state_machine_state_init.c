#include "state_machine_common.h"

#include "esp_log.h"

static const char *TAG = "SM_INIT";


StateMachineState_t stateInitProcessEvent(
    const StateMachineEvent_t *pEvent
)
{
    switch (pEvent->eId) {

        case SM_EVENT_SYSTEM_READY:
            /*
             * At startup we know the firmware is initialized,
             * but we do not necessarily know the physical
             * position of the continuous-rotation servo.
             */
            ESP_LOGI(TAG, "System ready. Nozzle position unknown");

            return STATE_MACHINE_POSITION_UNKNOWN;

        default:
            ESP_LOGW(TAG, "Event %s ignored while in INIT",
                stateMachineEventName(*pEvent)
            );

            return STATE_MACHINE_INIT;
    }
}