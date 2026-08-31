#ifndef COMPONENTS_I2C_H_
#define COMPONENTS_I2C_H_

#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"


esp_err_t i2cBusInit(void);

esp_err_t i2cBusAddDevice(
    uint8_t ubAddress,
    uint32_t ulSpeedHz,
    i2c_master_dev_handle_t* pDeviceHandle
);

esp_err_t i2cBusProbe(uint8_t ubAddress);

esp_err_t i2cBusWrite(
    i2c_master_dev_handle_t pDeviceHandle,
    const uint8_t* pData,
    size_t ulSize
);

esp_err_t i2cBusWriteRead(
    i2c_master_dev_handle_t pDeviceHandle,
    const uint8_t* pWriteData,
    size_t ulWriteSize,
    uint8_t* pReadData,
    size_t ulReadSize
);

#endif /* COMPONENTS_I2C_H_ */