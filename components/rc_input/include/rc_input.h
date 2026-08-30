#ifndef COMPONENTS_RC_INPUT_H_
#define COMPONENTS_RC_INPUT_H_

#include "esp_err.h"


/**
 * @brief Initialize the RC PWM input component.
 *
 * Configures the hardware required to capture incoming RC/Pixhawk PWM
 * signals and prepares the component to translate valid input changes
 * into state-machine events.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if already initialized
 *      - Appropriate esp_err_t error code on failure
 */
esp_err_t rcInputInit(void);

#endif /* COMPONENTS_RC_INPUT_H_ */