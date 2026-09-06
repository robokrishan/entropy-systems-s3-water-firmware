#include "tests.h"

#include "esp_log.h"
#include "esp_err.h"

#include "i2c.h"
#include "ssd1306.h"


static const char* TAG = "TEST_SSD1306";


void testSsd1306Basic(void) {
    esp_err_t lErr = ESP_OK;

    ESP_LOGW(TAG, "=== TEST: SSD1306 BASIC ===");

    lErr = i2cBusInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    lErr = ssd1306Init();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to initialize SSD1306. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    ESP_LOGI(TAG, "SSD1306 initialization PASSED");

test_cleanup:

    ssd1306Deinit();
    i2cBusDeinit();

    ESP_LOGW(TAG, "=== END SSD1306 BASIC TEST ===");
}


void testSsd1306WriteText(void) {
    esp_err_t lErr = ESP_OK;

    ESP_LOGW(TAG, "=== TEST: SSD1306 TEXT OUTPUT ===");

    lErr = i2cBusInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus. Code: 0x%X", lErr);

        goto test_cleanup;
    }
    
    lErr = ssd1306Init();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to initialize SSD1306. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    lErr = ssd1306WriteText(0, "WATER SAMPLER");
    if(lErr) {
        ESP_LOGE(TAG, "Failed to write line 0. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    lErr = ssd1306WriteText(2, "OLED TEST");
    if(lErr) {
        ESP_LOGE(TAG, "Failed to write line 2. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    lErr = ssd1306WriteText(4, "HELLO ESP32");
    if(lErr) {
        ESP_LOGE(TAG, "Failed to write line 4. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    ESP_LOGI(TAG, "SSD1306 text output PASSED");

test_cleanup:

    ssd1306Deinit();
    i2cBusDeinit();

    ESP_LOGW(TAG, "=== END SSD1306 TEXT TEST ===");
}



