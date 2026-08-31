#include "ina226.h"
#include "i2c.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_log.h"


#define INA226_I2C_ADDRESS          0x40
#define INA226_I2C_SPEED_HZ         400000
#define INA226_REG_BUS_VOLTAGE      0x02
#define INA226_REG_MANUFACTURE_ID   0xFE
#define INA226_REG_DIE_ID           0xFF
#define INA226_REG_CONFIGURATION    0x00

#define INA226_MANUFACTURE_ID       0x5449
#define INA226_DIA_ID               0x2260

#define INA226_BUS_VOLTAGE_LSB_V    0.00125f


static const char* TAG = "ina226";


static i2c_master_dev_handle_t s_pDeviceHandle = NULL;
static bool s_isInitialized = false;


static esp_err_t s_readRegister(uint8_t ubRegister, uint16_t* pValue) {
    if(NULL == pValue) {
        return ESP_ERR_INVALID_ARG;
    }

    if(NULL == s_pDeviceHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t ubData[2] = {0};

    esp_err_t lErr = ESP_OK;
    
    lErr = i2cBusWriteRead(
        s_pDeviceHandle,
        &ubRegister,
        sizeof(ubRegister),
        ubData,
        sizeof(ubData)
    );

    if(lErr) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X. Code: 0x%X",
            ubRegister,
            lErr
        );

        return lErr;
    }

    *pValue = ((uint16_t)ubData[0] << 8) | ubData[1];

    return ESP_OK;
}


esp_err_t ina226Init(void) {
    esp_err_t lErr = ESP_OK;

    if(s_isInitialized) {
        ESP_LOGW(TAG, "Already initialized");

        return ESP_ERR_INVALID_STATE;
    }

    lErr = i2cBusAddDevice(
        INA226_I2C_ADDRESS,
        INA226_I2C_SPEED_HZ,
        &s_pDeviceHandle
    );

    if(lErr) {
        ESP_LOGE(TAG, "Failed to add INA226 to I2C bus. Code: 0x%X", lErr);

        return lErr;
    }

    uint16_t uwManufactureId = 0;

    lErr = s_readRegister(INA226_REG_MANUFACTURE_ID, &uwManufactureId);
    if(lErr) {
        return lErr;
    }

    ESP_LOGI(TAG, "Manufacture ID: 0x%04X", uwManufactureId);


    if(INA226_MANUFACTURE_ID != uwManufactureId) {
        ESP_LOGW(TAG, "Manufacturer ID not recognized");
    }

    uint16_t uwDieId = 0;

    lErr = s_readRegister(INA226_REG_DIE_ID, &uwDieId);
    if(lErr) {
        return lErr;
    }

    ESP_LOGI(TAG, "Die ID: 0x%04X", uwDieId);

    s_isInitialized = true;

    ESP_LOGI(TAG, "INA226 initialized");

    uint16_t uwConfig = 0;

    lErr = s_readRegister(
        INA226_REG_CONFIGURATION,
        &uwConfig
    );

    if(lErr) {
        return lErr;
    }

    ESP_LOGI(
        TAG,
        "Configuration register: 0x%04X",
        uwConfig
    );

    return ESP_OK;
}


esp_err_t ina226ReadBusVoltage(float* pVoltageV) {
    if(!s_isInitialized) {
        return ESP_ERR_INVALID_STATE;
    }


    if(NULL == pVoltageV) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t uwRawVoltage = 0;

    esp_err_t lErr = s_readRegister(INA226_REG_BUS_VOLTAGE, &uwRawVoltage);

    if(lErr) {
        return lErr;
    }

    *pVoltageV = (float)uwRawVoltage*INA226_BUS_VOLTAGE_LSB_V;


    return ESP_OK;
}



