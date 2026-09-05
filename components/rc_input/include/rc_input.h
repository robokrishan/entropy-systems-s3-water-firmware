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


/**
 * @brief Deinitialize the RC input component.
 *
 * Stops PWM capture and signal-loss monitoring, deletes the processing task
 * and queue, releases MCPWM capture resources, and resets internal channel
 * state.
 *
 * Cleanup is best-effort; a failure while releasing one resource is logged
 * without preventing the remaining resources from being cleaned up.
 *
 * The function is safe to call when the component is partially initialized,
 * already deinitialized, or was never initialized.
 */
void rcInputDeinit(void);

#endif /* COMPONENTS_RC_INPUT_H_ */