#include "state_machine_common.h"

#include "esp_log.h"

static const char *TAG = "SM_POS_UNKNOWN";


StateMachineStateId_t statePositionUnknownProcessEvent(StateMachineEvent_t *pEvent) {
    switch (pEvent->eId) {

        case SM_EVENT_NOZZLE_EXTEND:
            ESP_LOGI(TAG, "Extending nozzle from unknown position");

            return STATE_MACHINE_LOWERING;


        case SM_EVENT_NOZZLE_RETRACT:
            ESP_LOGI(TAG, "Retracting nozzle from unknown position");

            return STATE_MACHINE_RAISING;


        case SM_EVENT_UPPER_LIMIT_ACTIVE:
            /*
             * Upper limit switch provides a known
             * physical reference position.
             */
            ESP_LOGI(TAG, "Upper limit active. Nozzle position resolved as STOWED");

            return STATE_MACHINE_STOWED;


        case SM_EVENT_LOWER_LIMIT_ACTIVE:
            /*
             * Lower limit switch provides a known
             * physical reference position.
             */
            ESP_LOGI(TAG, "Lower limit active. Nozzle position resolved as DEPLOYED");

            return STATE_MACHINE_DEPLOYED;


        default:
            ESP_LOGW(TAG, "Event %s ignored while nozzle position is unknown",
                stateMachineEventName(*pEvent)
            );

            return STATE_MACHINE_POSITION_UNKNOWN;
    }
}