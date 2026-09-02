#ifndef COMPONENTS_INA226_H_
#define COMPONENTS_INA226_H_

#include "esp_err.h"


esp_err_t ina226Init(void);

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


esp_err_t ina226ReadShuntVoltage(float* pShuntV);


esp_err_t ina226ReadPower();

#endif /* COMPONENTS_INA226_H_ */