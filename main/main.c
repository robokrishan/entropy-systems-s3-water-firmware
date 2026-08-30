#include <stdio.h>
#include "esp_log.h"
#include "state_machine.h"
#include "nozzle_servo.h"
#include "state_machine_states.h"
#include "pump.h"
#include "limit_switch.h"
#include "motion_timeout.h"
#include "rc_input.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tests.h"

const char* TAG = "main";

#define RUN_ONLY 1

void app_main(void) {

    ESP_LOGI(TAG, "Starting state machine test");
    
    esp_err_t lErr = ESP_OK;

#ifndef RUN_ONLY
    lErr = nozzleServoInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init nozzle servo! Code: 0x%X", lErr);
        return;
    }

    lErr = pumpInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init pump. Code: 0x%X", lErr);
        return;
    }

    lErr = stateMachineStatesRegister();
    if(ESP_OK != lErr) {
        ESP_LOGE(TAG, "Failed to register states! Code: 0x%X", lErr);
        return;
    }

    lErr = stateMachineInit();
    if(ESP_OK != lErr) {
        ESP_LOGE(TAG, "Failed to init state machine! Code: 0x%X", lErr);
        return;
    }

    lErr = motionTimeoutInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init motion timeout! Code: 0x%X", lErr);
        return;
    }

    lErr = limitSwitchInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init limit switches! Code: 0x%X", lErr);
        return;
    }

#endif

    lErr = rcInputInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init rc input! Code: 0x%X", lErr);
        return;
    }

#ifndef RUN_ONLY
    stateMachinePostEvent(SM_EVENT_SYSTEM_READY);
    limitSwitchSyncState();
#endif

    // testNormalSequence();
    // testWrongSequence();
    // testFaultSequence();

    // testNozzleServoSequence();
    // testNozzlServoNeutral();
    // testStateInitFailure();
    // testStateDeinitFailure();
    // testHaltSequence();
    // testPumpSequence();
    // testLimitSwitchSequence();
    // testMotionTimeoutSequence();
    // testMotionTimeoutIntegrationSequence();

    ESP_LOGI(TAG, "END TEST");
    
}