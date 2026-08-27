#include <stdio.h>
#include "esp_log.h"
#include "state_machine.h"
#include "nozzle_servo.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tests.h"

const char* TAG = "main";



void app_main(void) {

    ESP_LOGI(TAG, "Starting state machine test");
    
    esp_err_t lErr = ESP_OK;
    lErr = nozzleServoInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init nozzle servo! Code: 0x%X", lErr);
        return;
    }

    lErr = stateMachineInit();
    if(ESP_OK != lErr) {
        ESP_LOGE(TAG, "Failed to init state machine! Code: 0x%X", lErr);
        return;
    }

    // testNormalSequence();
    // testWrongSequence();

    // testNozzleServoSequence();
    testNozzlServoNeutral();

    ESP_LOGI(TAG, "END TEST");
    
}