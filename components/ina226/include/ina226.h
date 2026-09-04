#ifndef COMPONENTS_INA226_H_
#define COMPONENTS_INA226_H_

#include "esp_err.h"


/**
 * @brief Initialize the INA226 current and power monitor.
 *
 * Registers the INA226 device on the I2C bus, verifies device
 * identification, configures the calibration register, and prepares
 * the device for voltage, current, and power measurements.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the INA226 is already initialized
 *      - ESP_ERR_NOT_FOUND if the detected device is not recognized
 *      - Appropriate esp_err_t error code on I2C communication failure
 */
esp_err_t ina226Init(void);


/**
 * @brief Read the monitored bus voltage.
 *
 * @param[out] pVoltageV Pointer to store the bus voltage in volts.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if INA226 is not initialized
 *      - ESP_ERR_INVALID_ARG if pVoltageV is NULL
 *      - Appropriate esp_err_t error code on communication failure
 */
esp_err_t ina226ReadBusVoltage(float* pVoltageV);


/**
 * @brief Read the measured load current.
 *
 * @param[out] pCurrentA Pointer to store current in amperes.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if INA226 is not initialized
 *      - ESP_ERR_INVALID_ARG if pCurrentA is NULL
 *      - Appropriate esp_err_t error code on communication failure
 */
esp_err_t ina226ReadCurrent(float* pCurrentA);


/**
 * @brief Read the voltage drop across the shunt resistor.
 *
 * @param[out] pShuntV Pointer to store the shunt voltage in volts.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if INA226 is not initialized
 *      - ESP_ERR_INVALID_ARG if pShuntV is NULL
 *      - Appropriate esp_err_t error code on communication failure
 */
esp_err_t ina226ReadShuntVoltage(float* pShuntV);


/**
 * @brief Read the measured load power.
 *
 * @param[out] pPower Pointer to store the measured power in watts.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if INA226 is not initialized
 *      - ESP_ERR_INVALID_ARG if pPower is NULL
 *      - Appropriate esp_err_t error code on communication failure
 */
esp_err_t ina226ReadPower(float* pPowerW);

#endif /* COMPONENTS_INA226_H_ */