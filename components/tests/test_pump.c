#include "tests.h"
#include "esp_log.h"

#include "pump.h"


const char* TAG = "TEST_PUMP";


void testPumpSequence(void) {
    esp_err_t lErr = ESP_OK;

    lErr = pumpInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to initialize pump. Code: 0x%X", lErr);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(3000));

    lErr = pumpOn();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to power on pump. Code: 0x%X", lErr);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(5000));

    lErr = pumpOff();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to power off pump. Code: 0x%X", lErr);
        return;
    }

    ESP_LOGI(TAG, "Pump test success.");
}

