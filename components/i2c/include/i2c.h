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


/**
 * @brief Remove a device from the I2C master bus.
 *
 * Removes the specified device handle from the initialized I2C bus.
 *
 * @param pDeviceHandle I2C device handle to remove.
 *
 * @return ESP_OK on success, otherwise an appropriate error code.
 */
esp_err_t i2cBusRemoveDevice(i2c_master_dev_handle_t pDeviceHandle);


/**
 * @brief Deinitialize the I2C master bus.
 *
 * Deletes the I2C master bus and releases its associated resources.
 * All devices must be removed from the bus before this function is called.
 *
 * Cleanup is null-safe. If bus deletion fails, the bus handle and
 * initialization state are retained so cleanup can be attempted again.
 */
void i2cBusDeinit(void);

#endif /* COMPONENTS_I2C_H_ */