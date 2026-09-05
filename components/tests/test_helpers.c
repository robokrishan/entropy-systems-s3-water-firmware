#include "test_helpers.h"

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "state_machine.h"


#define TEST_STATE_POLL_INTERVAL_MS             50


static const char* TAG = "TEST_HELPERS";


/* State Machine Test */
bool testCheckState(StateMachineStateId_t eExpectedState) {
    StateMachineStateId_t eActualState = stateMachineGetCurrentState();

    if(eActualState != eExpectedState) {
        ESP_LOGE(TAG, "State check failed. Expected %s. Got %s",
            stateMachineStateName(eExpectedState),
            stateMachineStateName(eActualState)
        );
        return false;

    } else {
        ESP_LOGI(TAG, "State check passed");
        return true;
    }
}


void testPostEvent(StateMachineEventId_t eEvent, uint32_t ulDelayMs) {
    StateMachineEvent_t sTempEvent = {
        .eId = eEvent,
        .ulData = 0
    };

    ESP_LOGI(TAG, "Posting test event: %s", stateMachineEventName(sTempEvent));

    esp_err_t lErr = ESP_OK;

    lErr = stateMachinePostEvent(eEvent);
    if(ESP_OK != lErr) {
        ESP_LOGE(TAG, "Failed to post event: %s", esp_err_to_name(lErr));
    }

    vTaskDelay(pdMS_TO_TICKS(ulDelayMs));
}


bool testWaitForState(StateMachineStateId_t eExpectedState) {
    StateMachineStateId_t eCurrentState = stateMachineGetCurrentState();
    StateMachineStateId_t ePreviousState = eCurrentState;

    uint32_t ulElapsedMs = 0;

    ESP_LOGI(
        TAG,
        "Waiting for state: %s",
        stateMachineStateName(eExpectedState)
    );

    while(eCurrentState != eExpectedState) {

        /*
         * Don't hang forever if the mechanism enters FAULT
         * before reaching the expected state.
         */
        if(STATE_MACHINE_FAULT == eCurrentState) {
            ESP_LOGE(TAG, "Entered FAULT while waiting for %s", 
                stateMachineStateName(eExpectedState));

            return false;
        }

        /*
         * Prevent failed test from blocking infinitely
         */
        if(ulElapsedMs >= ulTimeoutMs) {
            ESP_LOGE(TAG, "Timed out waiting for %s. Current State: %s",
                stateMachineStateName(eExpectedState),
                stateMachineStateName(eCurrentState)
            );

            return false;
        }

        /*
         * Only print when the FSM actually changes state.
         */
        if(eCurrentState != ePreviousState) {
            ESP_LOGI(TAG, "Current state: %s", stateMachineStateName(eCurrentState));

            ePreviousState = eCurrentState;
        }

        /*
         * This isn't an actuation delay. It just yields CPU time
         * while polling the FSM.
         */
        vTaskDelay(pdMS_TO_TICKS(TEST_STATE_POLL_INTERVAL_MS));

        ulElapsedMs += TEST_STATE_POLL_INTERVAL_MS;
        eCurrentState = stateMachineGetCurrentState();
    }

    ESP_LOGI(TAG, "Reached state: %s", stateMachineStateName(eExpectedState));

    return true;
}
