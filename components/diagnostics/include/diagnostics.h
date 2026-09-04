#ifndef COMPONENTS_DIAGNOSTICS_H_
#define COMPONENTS_DIAGNOSTICS_H_

#include "esp_err.h"


/**
 * @brief Initialize the diagnostics subsystem.
 *
 * Creates and starts the diagnostics task responsible for periodically
 * collecting system status information and updating the diagnostic display.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if diagnostics is already initialized
 *      - Appropriate esp_err_t error code if task creation or initialization fails
 */
esp_err_t diagnosticsInit(void);

#endif /* COMPONENTS_DIAGNOSTICS_H_ */