#include "i2c.h"
#include "device_config.h"
#include <stdbool.h>
#include "esp_log.h"

#define I2C_PORT        I2C_NUM_0
#define I2C_TIMEOUT_MS  100


static const char* TAG = "i2c_bus";

static i2c_master_bus_handle_t s_pBusHandle = NULL;
static bool s_isInitialized = false;


esp_err_t i2cBusInit(void) {
    esp_err_t lErr = ESP_OK;

    if(s_isInitialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_master_bus_config_t sBusConfig = {
        .i2c_port = I2C_PORT,
        .sda_io_num = CONFIG_PIN_I2C_SDA,
        .scl_io_num = CONFIG_PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    lErr = i2c_new_master_bus(&sBusConfig, &s_pBusHandle);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init I2C bus. Code: 0x%X", lErr);

        return lErr;
    }

    s_isInitialized = true;

    ESP_LOGI(TAG, "I2C bus initialized");

    return ESP_OK;
}



esp_err_t i2cBusAddDevice(
    uint8_t ubAddress,
    uint32_t ulSpeedHz,
    i2c_master_dev_handle_t* pDeviceHandle
) {
    if(!s_isInitialized) {
        ESP_LOGE(TAG, "i2c not initialized");

        return ESP_ERR_INVALID_STATE;
    }

    if(NULL == pDeviceHandle || ubAddress > 0x7F || 0 == ulSpeedHz) {
        ESP_LOGE(TAG, "invalid arg");

        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t sDeviceConfig = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ubAddress,
        .scl_speed_hz = ulSpeedHz
    };


    esp_err_t lErr = i2c_master_bus_add_device(
                                    s_pBusHandle,
                                    &sDeviceConfig,
                                    pDeviceHandle
    );

    if(lErr) {
        ESP_LOGE(TAG, "Failed to add i2c device 0x%X. Code: 0x%X",
            ubAddress,
            lErr
        );

        return lErr;
    }

    ESP_LOGI(TAG, "Added i2c device: 0x%X", ubAddress);

    return ESP_OK;
}


esp_err_t i2cBusProbe(uint8_t ubAddress) {
    if(!s_isInitialized) {
        ESP_LOGE(TAG, "i2c not initialized");

        return ESP_ERR_INVALID_STATE;
    }

    if(ubAddress > 0x7F) {
        ESP_LOGE(TAG, "Invalid i2c address: 0x%X", ubAddress);

        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t lErr = i2c_master_probe(s_pBusHandle, ubAddress, I2C_TIMEOUT_MS);

    return lErr;
}


esp_err_t i2cBusWrite(
    i2c_master_dev_handle_t pDeviceHandle,
    const uint8_t* pData,
    size_t ulSize
) {
    if(!s_isInitialized) {
        ESP_LOGE(TAG, "i2c not initialized");

        return ESP_ERR_INVALID_STATE;
    }

    if((NULL == pDeviceHandle) || (NULL == pData) || (0 == ulSize)) {
        ESP_LOGE(TAG, "Invalid arg");

        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t lErr = i2c_master_transmit(
        pDeviceHandle,
        pData,
        ulSize,
        I2C_TIMEOUT_MS
    );

    if(lErr) {
        ESP_LOGE(TAG, "i2c write failed. Code: 0x%X", lErr);
    }

    return lErr;
}


esp_err_t i2cBusWriteRead(
    i2c_master_dev_handle_t pDeviceHandle,
    const uint8_t* pWriteData,
    size_t ulWriteSize,
    uint8_t* pReadData,
    size_t ulReadSize
) {
    if(!s_isInitialized) {
        ESP_LOGE(TAG, "i2c not initialized");

        return ESP_ERR_INVALID_STATE;
    }

    if((NULL == pDeviceHandle) || (NULL == pWriteData) ||
       (0 == ulWriteSize) || (NULL == pReadData) ||
       (0 == ulReadSize)) {

        ESP_LOGE(TAG, "Invalid arg");

        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t lErr = i2c_master_transmit_receive(
        pDeviceHandle,
        pWriteData,
        ulWriteSize,
        pReadData,
        ulReadSize,
        I2C_TIMEOUT_MS
    );

    if(lErr) {
        ESP_LOGE(TAG, "i2c write-read failed. Code: 0x%X", lErr);
    }

    return lErr;
}

