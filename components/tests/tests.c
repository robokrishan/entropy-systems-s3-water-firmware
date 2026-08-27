#include "tests.h"
#include "esp_err.h"
#include "esp_log.h"
#include "state_machine_common.h"
#include "state_machine.h"
#include "nozzle_servo.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "TEST";


/* State Machine Test */
static void s_postTestEvent(StateMachineEventId_t eEvent, uint32_t ulDelayMs) {
    StateMachineEvent_t temp = {
        .eId = eEvent,
        .ulData = 0
    };

    ESP_LOGI(TAG, "Posting test event: %s", stateMachineEventName(temp));

    esp_err_t lErr = ESP_OK;

    lErr = stateMachinePostEvent(eEvent);
    if(ESP_OK != lErr) {
        ESP_LOGE(TAG, "Failed to post event: %s", esp_err_to_name(lErr));
    }

    vTaskDelay(pdMS_TO_TICKS(ulDelayMs));
}

void testNormalSequence(void) {
    s_postTestEvent(SM_EVENT_SYSTEM_READY, 1000);
    s_postTestEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 1000);
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 1000);
    s_postTestEvent(SM_EVENT_LOWER_LIMIT_ACTIVE, 1000);
    s_postTestEvent(SM_EVENT_PUMP_ON, 1000);
    s_postTestEvent(SM_EVENT_PUMP_OFF, 1000);
    s_postTestEvent(SM_EVENT_NOZZLE_RETRACT, 1000);
    s_postTestEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 1000);
}

void testWrongSequence(void)
{
    /*
     * Establish known starting position.
     *
     * INIT -> POSITION_UNKNOWN -> STOWED
     */
    s_postTestEvent(SM_EVENT_SYSTEM_READY, 500);
    s_postTestEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 500);


    /*
     * STOWED + PUMP_ON
     *
     * Must be rejected.
     * Remain in STOWED.
     */
    s_postTestEvent(SM_EVENT_PUMP_ON, 1000);


    /*
     * STOWED -> LOWERING
     */
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 1000);


    /*
     * LOWERING + PUMP_ON
     *
     * Must be rejected.
     * Remain in LOWERING.
     */
    s_postTestEvent(SM_EVENT_PUMP_ON, 1000);


    /*
     * WRONG LIMIT TEST:
     *
     * We are LOWERING, so the upper limit should not
     * complete the movement.
     *
     * Must remain in LOWERING.
     */
    s_postTestEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 1000);


    /*
     * Stop before reaching lower limit.
     *
     * LOWERING -> POSITION_UNKNOWN
     */
    s_postTestEvent(SM_EVENT_STOP_SPOOL, 1000);


    /*
     * POSITION_UNKNOWN -> RAISING
     */
    s_postTestEvent(SM_EVENT_NOZZLE_RETRACT, 1000);


    /*
     * WRONG LIMIT TEST:
     *
     * We are RAISING, so the lower limit should not
     * complete the movement.
     *
     * Must remain in RAISING.
     */
    s_postTestEvent(SM_EVENT_LOWER_LIMIT_ACTIVE, 1000);


    /*
     * Correct upper limit.
     *
     * RAISING -> STOWED
     */
    s_postTestEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 1000);


    /*
     * STOWED -> LOWERING
     */
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 1000);


    /*
     * Global fault.
     *
     * LOWERING -> FAULT
     */
    s_postTestEvent(SM_EVENT_FAULT, 1000);


    /*
     * Must be rejected while in FAULT.
     */
    s_postTestEvent(SM_EVENT_PUMP_ON, 1000);


    /*
     * FAULT -> POSITION_UNKNOWN
     */
    s_postTestEvent(SM_EVENT_RESET, 1000);
}


/* Nozzle Servo Test */
void testNozzleServoSequence(void) {
    esp_err_t lErr = ESP_OK;

    ESP_LOGI(TAG, "Starting nozzle servo test");

    lErr = nozzleServoInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init nozzle servo! Code: 0x%X", lErr);
        return;
    }

    ESP_LOGW(TAG, "Initialized");

    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGW(TAG, "Extending");

    lErr = nozzleServoExtend();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to extend servo! Code: 0x%X", lErr);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGW(TAG, "Stopping");

    lErr = nozzleServoStop();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to stop servo! Code: 0x%X", lErr);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGW(TAG, "Retracting");

    lErr = nozzleServoRetract();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to retract servo! Code: 0x%X", lErr);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGW(TAG, "Stopping");

    lErr = nozzleServoStop();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to stop servo! Code: 0x%X", lErr);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5000));

    nozzleServoDeinit();

    ESP_LOGW(TAG, "De-initialized");

    ESP_LOGI(TAG, "Nozzle servo test complete");
}


void testNozzlServoNeutral(void) {
    esp_err_t lErr = ESP_OK;

    ESP_LOGI(TAG, "Starting nozzle servo test");

    lErr = nozzleServoInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init nozzle servo! Code: 0x%X", lErr);
        return;
    }

    ESP_LOGW(TAG, "Initialized");

    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGW(TAG, "Stopping");

    lErr = nozzleServoStop();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to stop servo! Code: 0x%X", lErr);
        return;
    }

    ESP_LOGW(TAG, "Servo should not move or jitter!");

    vTaskDelay(pdMS_TO_TICKS(30000));

    nozzleServoDeinit();

    ESP_LOGW(TAG, "De-initialized");

    ESP_LOGI(TAG, "Nozzle servo test complete");
}
