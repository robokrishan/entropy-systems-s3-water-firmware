#include "tests.h"

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pump.h"


static const char* TAG = "TEST_PUMP";


void testPumpSequence(void) {
    esp_err_t lErr = ESP_OK;

    ESP_LOGW(TAG, "=== TEST: PUMP SEQUENCE ===");

    lErr = pumpInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to initialize pump. Code: 0x%X", lErr);
        return;
    }

    ESP_LOGI(TAG, "Pump initialized - pump should be OFF");

    vTaskDelay(pdMS_TO_TICKS(3000));

    // Turn pump ON
    ESP_LOGI(TAG, "Turning pump ON");

    lErr = pumpOn();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to power on pump. Code: 0x%X", lErr);
        goto test_cleanup;
    }

    vTaskDelay(pdMS_TO_TICKS(5000));

    // Turn pump OFF
    ESP_LOGI(TAG, "Turning pump OFF");

    lErr = pumpOff();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to power off pump. Code: 0x%X", lErr);
        goto test_cleanup;
    }

    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG, "Pump sequence completed successfully");


test_cleanup:

    pumpDeinit();

    ESP_LOGW(TAG, "=== END PUMP TEST ===");
}

