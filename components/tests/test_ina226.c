#include "tests.h"
#include "esp_log.h"
#include "esp_err.h"

#include "i2c.h"
#include "ina226.h"


static const char* TAG = "TEST_INA226";



void testIna226Basic(void) {
    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 6;

    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, "INA226 BASIC TEST");
    ESP_LOGW(TAG, "========================================");


    /* ============================================================
     * TEST 1: I2C BUS INITIALIZATION
     * ============================================================ */

    esp_err_t lErr = i2cBusInit();

    if(ESP_OK == lErr) {
        ESP_LOGI(TAG, "I2C bus initialization PASSED");
        ubTestPassCount++;
    } else {
        ESP_LOGE(TAG, "I2C bus initialization FAILED. Code: 0x%X", lErr);

        goto test_complete;
    }

    /* ============================================================
     * TEST 2: INA226 INITIALIZATION / DEVICE IDENTIFICATION
     * ============================================================ */

    lErr = ina226Init();

    if(ESP_OK == lErr) {
        ESP_LOGI(TAG, "INA226 initialization PASSED");
        ubTestPassCount++;
    } else {
        ESP_LOGE(TAG, "INA226 initialization FAILED. Code: 0x%X",lErr);

        goto test_complete;
    }


    /* ============================================================
     * TEST 3: BUS VOLTAGE REGISTER READ
     * ============================================================ */

    float fBusVoltage = 0.0f;
    float fShuntVoltage = 0.0f;
    float fCurrent = 0.0f;

    lErr = ina226ReadBusVoltage(&fBusVoltage);
    if(ESP_OK == lErr) {
        ESP_LOGI(TAG, "Bus voltage: %.3f V", fBusVoltage);

        ubTestPassCount++;
    } else {
        ESP_LOGE(TAG, "Failed to read bus voltage. Code: 0x%X", lErr);
    }


    lErr = ina226ReadShuntVoltage(&fShuntVoltage);
    if(ESP_OK == lErr) {
        ESP_LOGI(TAG, "Shunt voltage: %.6f V", fShuntVoltage);
        ubTestPassCount++;
    } else {
        ESP_LOGE(TAG, "Failed to read shunt voltage. Code: 0x%X", lErr);
    }


    lErr = ina226ReadCurrent(&fCurrent);
    if(ESP_OK == lErr) {
        ESP_LOGI(TAG, "Current: %.6f A", fCurrent);
        ubTestPassCount++;
    } else {
        ESP_LOGE(TAG, "Failed to read current. Code: 0x%X", lErr);
    }

    float fPower = 0;

    lErr = ina226ReadPower(&fPower);
    if(ESP_OK == lErr) {
        ESP_LOGI(TAG, "Power: %.6f A", fPower);
        ubTestPassCount++;
    } else {
        ESP_LOGE(TAG, "Failed to read power. Code: 0x%X", lErr);
    }

test_complete:

    ESP_LOGW(TAG, "========================================");

    if(ubTestPassCount == ubTestCount) {
        ESP_LOGI(TAG, "TEST PASSED: %u / %u checks passed",
            ubTestPassCount,
            ubTestCount
        );
    } else {
        ESP_LOGE(TAG, "TEST FAILED: %u / %u checks passed",
            ubTestPassCount,
            ubTestCount
        );
    }

    ESP_LOGW(TAG, "========================================");
}
