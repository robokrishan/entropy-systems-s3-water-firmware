#include "tests.h"

#include "esp_err.h"
#include "esp_log.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nozzle_servo.h"


const char* TAG = "TEST_NOZZLE_SERVO";


void testNozzleServoSequence(void) {
    esp_err_t lErr = ESP_OK;

    ESP_LOGI(TAG, "=== TEST: Nozzle servo sequence ===");

    lErr = nozzleServoInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init nozzle servo. Code: 0x%X", lErr);
        goto test_cleanup;
    }

    ESP_LOGI(TAG, "Nozzle servo initialized");

    vTaskDelay(pdMS_TO_TICKS(3000));


    /*
     * Extend.
     *
     * Set the desired pulse width before enabling the PWM output.
     */
    ESP_LOGI(TAG, "Extending");

    lErr = nozzleServoExtend();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to set extend command. Code: 0x%X", lErr);
        goto test_cleanup;
    }

    lErr = nozzleServoEnable();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to enable nozzle servo. Code: 0x%X", lErr);
        goto test_cleanup;
    }

    vTaskDelay(pdMS_TO_TICKS(5000));


    /*
     * Stop by completely disabling the servo signal.
     */
    ESP_LOGI(TAG, "Disabling servo output");

    lErr = nozzleServoDisable();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to disable nozzle servo. Code: 0x%X", lErr);
        goto test_cleanup;
    }

    vTaskDelay(pdMS_TO_TICKS(5000));


    /*
     * Retract.
     */
    ESP_LOGI(TAG, "Retracting");

    lErr = nozzleServoRetract();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to set retract command. Code: 0x%X", lErr);
        goto test_cleanup;
    }

    lErr = nozzleServoEnable();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to enable nozzle servo. Code: 0x%X", lErr);
        goto test_cleanup;
    }

    vTaskDelay(pdMS_TO_TICKS(5000));


    /*
     * Stop by disabling the PWM output.
     */
    ESP_LOGI(TAG, "Disabling servo output");

    lErr = nozzleServoDisable();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to disable nozzle servo. Code: 0x%X", lErr);
        goto test_cleanup;
    }

    vTaskDelay(pdMS_TO_TICKS(5000));


test_cleanup:

    nozzleServoDeinit();

    ESP_LOGI(TAG, "=== END TEST ===");
}


void testNozzleServoNeutral(void) {
    esp_err_t lErr = ESP_OK;

    ESP_LOGI(TAG, "=== TEST: Nozzle servo neutral ===");

    lErr = nozzleServoInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init nozzle servo. Code: 0x%X", lErr);
        goto test_cleanup;
    }


    /*
     * Configure the neutral pulse before enabling PWM.
     */
    lErr = nozzleServoStop();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to set neutral command. Code: 0x%X", lErr);
        goto test_cleanup;
    }

    lErr = nozzleServoEnable();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to enable nozzle servo. Code: 0x%X", lErr);
        goto test_cleanup;
    }

    ESP_LOGW(TAG, "Neutral PWM active - servo should remain stationary");

    vTaskDelay(pdMS_TO_TICKS(30000));


    /*
     * Disconnect the PWM signal at the end of the test.
     */
    lErr = nozzleServoDisable();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to disable nozzle servo. Code: 0x%X", lErr);
    }


test_cleanup:

    nozzleServoDeinit();

    ESP_LOGI(TAG, "=== END TEST ===");
}

