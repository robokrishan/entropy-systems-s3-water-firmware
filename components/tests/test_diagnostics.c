#include "tests.h"

#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"

#include "i2c.h"
#include "ina226.h"
#include "ssd1306.h"

static const char* TAG = "TEST_DIAGNOSTICS";


/**
 * @brief Read INA226 measurements and write them to the diagnostic display.
 *
 * Reads the current bus voltage, current, and power measurements and renders
 * the formatted values on the SSD1306 display.
 *
 * @return
 *      - ESP_OK on success
 *      - Appropriate esp_err_t error code if a measurement or display
 *        operation fails
 */
static esp_err_t s_updateDiagnosticsDisplay(void) {
    float fBusVoltageV = 0.0f;
    float fCurrentA = 0.0f;
    float fPowerW = 0.0f;

    esp_err_t lErr = ina226ReadBusVoltage(&fBusVoltageV);
    if(lErr) {
        return lErr;
    }

    lErr = ina226ReadCurrent(&fCurrentA);
    if(lErr) {
        return lErr;
    }

    lErr = ina226ReadPower(&fPowerW);
    if(lErr) {
        return lErr;
    }

    /*
     * 21 visible characters maximum per row with the current
     * 5x7 font + 1 pixel spacing.
     */
    char cVoltageText[22] = {0};
    char cCurrentText[22] = {0};
    char cPowerText[22] = {0};

    // parse display texts
    snprintf(cVoltageText, sizeof(cVoltageText), "BAT: %.2f V", fBusVoltageV);
    snprintf(cCurrentText, sizeof(cCurrentText), "CUR: %.2f mA", fCurrentA * 1000.0f);
    snprintf(cPowerText, sizeof(cPowerText), "PWR: %.1f mW", fPowerW * 1000.0f);

    lErr = ssd1306WriteText(0, "WATER SAMPLER");
    if(lErr) {
        return lErr;
    }

    lErr = ssd1306WriteText(2, cVoltageText);
    if(lErr) {
        return lErr;
    }

    lErr = ssd1306WriteText(3, cCurrentText);
    if(lErr) {
        return lErr;
    }

    lErr = ssd1306WriteText(4, cPowerText);
    if(lErr) {
        return lErr;
    }

    return ESP_OK;
}


void testSsd1306Diagnostics(void) {
    esp_err_t lErr = ESP_OK;

    ESP_LOGW(TAG, "=== TEST: SSD1306 DIAGNOSTICS ===");

    lErr = i2cBusInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    lErr = ina226Init();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to initialize INA226. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    lErr = ssd1306Init();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to initialize SSD1306. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    lErr = s_updateDiagnosticsDisplay();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to update diagnostics display. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    ESP_LOGI(TAG, "Diagnostics display test PASSED");

test_cleanup:

    ssd1306Deinit();
    ina226Deinit();
    i2cBusDeinit();

    ESP_LOGW(TAG, "=== END SSD1306 DIAGNOSTICS TEST ===");
}