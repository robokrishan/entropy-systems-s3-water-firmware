#include <stdio.h>
#include "esp_log.h"
#include "state_machine.h"
#include "nozzle_servo.h"
#include "state_machine_states.h"
#include "pump.h"
#include "limit_switch.h"
#include "motion_timeout.h"
#include "rc_input.h"
#include "i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tests.h"

const char* TAG = "main";

#define RUN_ONLY 1

static esp_err_t s_initComponents(void) {

    esp_err_t lErr = ESP_OK;

    lErr = nozzleServoInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init nozzle servo! Code: 0x%X", lErr);
        
        goto end_component_init;
    }

    lErr = pumpInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init pump. Code: 0x%X", lErr);
        goto end_component_init;
    }

    lErr = stateMachineStatesRegister();
    if(ESP_OK != lErr) {
        ESP_LOGE(TAG, "Failed to register states! Code: 0x%X", lErr);
        goto end_component_init;
    }

    lErr = stateMachineInit();
    if(ESP_OK != lErr) {
        ESP_LOGE(TAG, "Failed to init state machine! Code: 0x%X", lErr);
        goto end_component_init;
    }

    lErr = motionTimeoutInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init motion timeout! Code: 0x%X", lErr);
        goto end_component_init;
    }

    lErr = limitSwitchInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init limit switches! Code: 0x%X", lErr);
        goto end_component_init;
    }

    lErr = rcInputInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init rc input! Code: 0x%X", lErr);
        goto end_component_init;
    }

    stateMachinePostEvent(SM_EVENT_SYSTEM_READY);
    limitSwitchSyncState();

    lErr = i2cBusInit(); 
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init i2c");
        goto end_component_init;
    }

end_component_init:

    return lErr;
}

static void s_testI2cScan(void) {
    uint8_t ubDeviceCount = 0;

    ESP_LOGI(TAG, "===== I2C Bus Scan ====");

    for(uint8_t ubAddr = 0x08; ubAddr <= 0x77; ubAddr++) {
        esp_err_t lErr = ESP_OK;

        lErr = i2cBusProbe(ubAddr);

        if(ESP_OK == lErr) {
            ESP_LOGI(TAG, "Found device at 0x%02X", ubAddr);

            ubDeviceCount++;
        }
    }

    ESP_LOGI(TAG, "I2C scan complete. %u devices found", ubDeviceCount);
}


void app_main(void) {

    ESP_LOGI(TAG, "Starting state machine test");
    
    esp_err_t lErr = ESP_OK;

    lErr = s_initComponents();
    if(!lErr) {
        ESP_LOGI(TAG, "Initialized all components");
    }

    vTaskDelay(pdMS_TO_TICKS(3000));

    s_testI2cScan();

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
    // testRcSignalLossSequence();
    // testRcSignalLossFsmSequence();
    // testRcPumpIntegrationSequence();
    // testRcNozzleIntegrationSequence();

    
    ESP_LOGI(TAG, "END TEST");
    
}