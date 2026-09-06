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
#include "ina226.h"
#include "ssd1306.h"
#include "diagnostics.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tests.h"

static const char* TAG = "main";

#define RUN_TESTS       1
#define TEST_TO_RUN     TEST_ALL_AUTOMATIC


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

    // lErr = rcInputInit();
    // if(lErr) {
    //     ESP_LOGE(TAG, "Failed to init rc input! Code: 0x%X", lErr);
    //     goto end_component_init;
    // }

    stateMachinePostEvent(SM_EVENT_SYSTEM_READY);
    limitSwitchSyncState();

    lErr = i2cBusInit(); 
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init i2c");
        goto end_component_init;
    }

    lErr = ina226Init();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init ina226");
        goto end_component_init;
    }

    lErr = ssd1306Init();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init ina226");
        goto end_component_init;
    }

    lErr = diagnosticsInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init diagnostics");
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


static void s_runAutomaticTests(void) {
    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, "RUNNING COMPLETE TEST SUITE");
    ESP_LOGW(TAG, "========================================");


    /* State-machine tests */
    testNormalSequence();
    vTaskDelay(pdMS_TO_TICKS(500));

    testWrongSequence();
    vTaskDelay(pdMS_TO_TICKS(500));

    testFaultSequence();
    vTaskDelay(pdMS_TO_TICKS(500));

    testHaltSequence();
    vTaskDelay(pdMS_TO_TICKS(500));

    testRcSignalLossFsmSequence();
    vTaskDelay(pdMS_TO_TICKS(500));


    /* Driver tests */
    testNozzleServoSequence();
    vTaskDelay(pdMS_TO_TICKS(500));

    testNozzleServoNeutral();
    vTaskDelay(pdMS_TO_TICKS(500));

    testPumpSequence();
    vTaskDelay(pdMS_TO_TICKS(500));


    /* Timeout tests */
    testMotionTimeoutSequence();
    vTaskDelay(pdMS_TO_TICKS(500));


    /* I2C / diagnostics tests */
    testIna226Basic();
    vTaskDelay(pdMS_TO_TICKS(500));

    testSsd1306Basic();
    vTaskDelay(pdMS_TO_TICKS(500));

    testSsd1306WriteText();
    vTaskDelay(pdMS_TO_TICKS(500));

    testSsd1306Diagnostics();


    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, "COMPLETE TEST SUITE FINISHED");
    ESP_LOGW(TAG, "========================================");
}


static void s_runSelectedTest(void) {
    switch(TEST_TO_RUN) {
        case TEST_NORMAL_SEQUENCE:
            testNormalSequence();
            break;

        case TEST_WRONG_SEQUENCE:
            testWrongSequence();
            break;

        case TEST_FAULT_SEQUENCE:
            testFaultSequence();
            break;

        case TEST_STATE_INIT_FAILURE:
            testStateInitFailure();
            break;

        case TEST_STATE_DEINIT_FAILURE:
            testStateDeinitFailure();
            break;

        case TEST_HALT_SEQUENCE:
            testHaltSequence();
            break;

        case TEST_NOZZLE_SERVO_SEQUENCE:
            testNozzleServoSequence();
            break;

        case TEST_NOZZLE_SERVO_NEUTRAL:
            testNozzleServoNeutral();
            break;

        case TEST_PUMP_SEQUENCE:
            testPumpSequence();
            break;

        case TEST_LIMIT_SWITCH_SEQUENCE:
            testLimitSwitchSequence();
            break;

        case TEST_MOTION_TIMEOUT_SEQUENCE:
            testMotionTimeoutSequence();
            break;

        case TEST_MOTION_TIMEOUT_INTEGRATION_SEQUENCE:
            testMotionTimeoutIntegrationSequence();
            break;

        case TEST_RC_SIGNAL_LOSS_SEQUENCE:
            testRcSignalLossSequence();
            break;

        case TEST_RC_SIGNAL_LOSS_FSM_SEQUENCE:
            testRcSignalLossFsmSequence();
            break;

        case TEST_RC_PUMP_INTEGRATION_SEQUENCE:
            testRcPumpIntegrationSequence();
            break;

        case TEST_RC_NOZZLE_INTEGRATION_SEQUENCE:
            testRcNozzleIntegrationSequence();
            break;

        case TEST_INA226_BASIC:
            testIna226Basic();
            break;

        case TEST_SSD1306_BASIC:
            testSsd1306Basic();
            break;

        case TEST_SSD1306_WRITE_TEXT:
            testSsd1306WriteText();
            break;

        case TEST_SSD1306_DIAGNOSTICS:
            testSsd1306Diagnostics();
            break;

        case TEST_ALL_AUTOMATIC:
            s_runAutomaticTests();
            break;

        default:
            ESP_LOGE(TAG, "Invalid test selection");
            break;
    }
}



void app_main(void) {

#if RUN_TESTS

    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, "TEST MODE");
    ESP_LOGW(TAG, "========================================");

    vTaskDelay(pdMS_TO_TICKS(3000));

    s_runSelectedTest();

    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, "TEST EXECUTION COMPLETE");
    ESP_LOGW(TAG, "========================================");

#else

    ESP_LOGI(TAG, "Starting water sampler firmware");

    esp_err_t lErr = s_initComponents();

    if(lErr) {
        ESP_LOGE(TAG, "System initialization failed. Code: 0x%X", lErr);

        return;
    }

    ESP_LOGI(TAG, "All system components initialized");

#endif
}