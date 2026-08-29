#ifndef COMPONENTS_PUMP_H_
#define COMPONENTS_PUMP_H_

#include "esp_err.h"

/**
 * @brief Initialize the pump driver.
 *
 * Configures the pump control GPIO and places the pump in a safe
 * default OFF state.
 *
 * @return
 *      - ESP_OK on success
 *      - Appropriate esp_err_t error code on failure
 */
esp_err_t pumpInit(void);


/**
 * @brief Turn the pump on.
 *
 * Activates the pump control output.
 *
 * @return
 *      - ESP_OK on success
 *      - Appropriate esp_err_t error code on failure
 */
esp_err_t pumpOn(void);


/**
 * @brief Turn the pump off.
 *
 * Deactivates the pump control output and places the pump in its
 * safe inactive state.
 *
 * @return
 *      - ESP_OK on success
 *      - Appropriate esp_err_t error code on failure
 */
esp_err_t pumpOff(void);

#endif /* COMPONENTS_PUMP_H_ */