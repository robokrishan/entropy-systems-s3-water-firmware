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
#define INA226_REG_SHUNT_VOLTAGE    0x01
#define INA226_REG_CURRENT          0x04
#define INA226_REG_CALIBRATION      0x05
#define INA226_REG_POWER            0x03
#define INA226_POWER_LSB_W          0.000625f


#define INA226_SHUNT_RESISTANCE_OHM 0.100f
#define INA226_CURRENT_LSB_A        0.000025f
#define INA226_SHUNT_VOLTAGE_LSB_V          0.0000025f
#define INA226_CALIBRATION_VALUE    2048
#define INA226_MANUFACTURER_ID       0x5449

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


static esp_err_t s_writeRegister(uint8_t ubRegister, uint16_t uwValue) {
    uint8_t ubData[3] = {
        ubRegister,
        (uint8_t)(uwValue >> 8),
        (uint8_t)(uwValue & 0xFF)
    };

    esp_err_t lErr = i2cBusWrite(
        s_pDeviceHandle,
        ubData,
        sizeof(ubData)
    );

    if(lErr) {
        ESP_LOGE(TAG, "Failed to write register 0x%02X. Code: 0x%X", 
            ubRegister, 
            lErr
        );
    }

    return lErr;
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


    if(INA226_MANUFACTURER_ID != uwManufactureId) {
        ESP_LOGW(TAG, "Manufacturer ID not recognized");
    }

    uint16_t uwDieId = 0;

    lErr = s_readRegister(INA226_REG_DIE_ID, &uwDieId);
    if(lErr) {
        return lErr;
    }

    ESP_LOGI(TAG, "Die ID: 0x%04X", uwDieId);

    lErr = s_writeRegister(INA226_REG_CALIBRATION, INA226_CALIBRATION_VALUE);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to configure calibration register. Code: 0x%X", lErr);

        return lErr;
    }

    // check calibration register
    uint16_t uwCalibration = 0;

    lErr = s_readRegister(
        INA226_REG_CALIBRATION,
        &uwCalibration
    );

    if(lErr) {
        return lErr;
    }

    ESP_LOGI(
        TAG,
        "Calibration register: 0x%04X",
        uwCalibration
    );

    // check configuration register
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

    s_isInitialized = true;

    ESP_LOGI(TAG, "INA226 initialized");

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


esp_err_t ina226ReadCurrent(float* pCurrentA) {
    if(!s_isInitialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if(NULL == pCurrentA) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t uwRawCurrent = 0;

    esp_err_t lErr = ESP_OK;

    lErr = s_readRegister(INA226_REG_CURRENT, &uwRawCurrent);
    if(lErr) {
        return lErr;
    }

    // current register gives signed value
    int16_t wRawCurrent = (int16_t)uwRawCurrent;

    *pCurrentA = (float)wRawCurrent * INA226_CURRENT_LSB_A;

    return lErr;
}


esp_err_t ina226ReadShuntVoltage(float* pShuntV) {
    if(!s_isInitialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if(NULL == pShuntV) {
        return ESP_ERR_INVALID_ARG;
    }


    uint16_t uwRawVoltage = 0;

    esp_err_t lErr = ESP_OK;

    lErr = s_readRegister(INA226_REG_SHUNT_VOLTAGE, &uwRawVoltage);
    if(lErr) {
        return lErr;
    }

    int16_t wRawVoltage = (int16_t)uwRawVoltage;

    *pShuntV = (float)wRawVoltage * INA226_SHUNT_VOLTAGE_LSB_V;

    return lErr;
}


esp_err_t ina226ReadPower(float* pPower) {

    esp_err_t lErr = ESP_OK;

    if(!s_isInitialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if(NULL == pPower) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t uwRawPower = 0;
    
    lErr = s_readRegister(INA226_REG_POWER, &uwRawPower);
    if(lErr) {
        return lErr;
    }

    int16_t wRawPower = (int16_t)uwRawPower;

    *pPower = (float)wRawPower * INA226_POWER_LSB_W;

    return lErr;
}