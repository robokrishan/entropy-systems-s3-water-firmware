#ifndef COMPONENTS_I2C_H_
#define COMPONENTS_I2C_H_

#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"


/**
 * @brief Initialize the shared I2C master bus.
 *
 * Configures and creates the I2C master bus using the SDA and SCL pins
 * defined in the device configuration.
 *
 * If the bus is already initialized, the function returns ESP_OK without
 * creating another bus instance.
 *
 * @return ESP_OK on success, otherwise an appropriate error code.
 */
esp_err_t i2cBusInit(void);


/**
 * @brief Add a device to the initialized I2C master bus.
 *
 * Registers an I2C device with the specified 7-bit address and bus speed,
 * and returns the created device handle through pDeviceHandle.
 *
 * @param ubAddress 7-bit I2C device address.
 * @param ulSpeedHz I2C clock speed for the device in Hz.
 * @param pDeviceHandle Pointer used to return the created device handle.
 *
 * @return ESP_OK on success, otherwise an appropriate error code.
 */
esp_err_t i2cBusAddDevice(
    uint8_t ubAddress,
    uint32_t ulSpeedHz,
    i2c_master_dev_handle_t* pDeviceHandle
);


/**
 * @brief Probe the I2C bus for a device at the specified address.
 *
 * Checks whether a device acknowledges communication at the given
 * 7-bit I2C address.
 *
 * @param ubAddress 7-bit I2C address to probe.
 *
 * @return ESP_OK if a device responds, otherwise an appropriate error code.
 */
esp_err_t i2cBusProbe(uint8_t ubAddress);


/**
 * @brief Write data to an I2C device.
 *
 * Transmits the supplied data buffer to the specified I2C device using
 * the configured bus timeout.
 *
 * @param pDeviceHandle Handle of the target I2C device.
 * @param pData Pointer to the data buffer to transmit.
 * @param ulSize Number of bytes to transmit.
 *
 * @return ESP_OK on success, otherwise an appropriate error code.
 */
esp_err_t i2cBusWrite(
    i2c_master_dev_handle_t pDeviceHandle,
    const uint8_t* pData,
    size_t ulSize
);


/**
 * @brief Write data to an I2C device and then read its response.
 *
 * Performs a combined write-read transaction using the specified device
 * handle. The write phase is completed first, followed by the read phase
 * without releasing control of the transaction between operations.
 *
 * @param pDeviceHandle Handle of the target I2C device.
 * @param pWriteData Pointer to the data buffer to transmit.
 * @param ulWriteSize Number of bytes to transmit.
 * @param pReadData Pointer to the buffer used to store received data.
 * @param ulReadSize Number of bytes to read.
 *
 * @return ESP_OK on success, otherwise an appropriate error code.
 */
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