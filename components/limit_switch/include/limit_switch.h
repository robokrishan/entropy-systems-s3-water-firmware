#ifndef COMPONENTS_LIMIT_SWITCH_H_
#define COMPONENTS_LIMIT_SWITCH_H_

#include "esp_err.h"


/**
 * @brief Initialize the upper and lower limit switches.
 *
 * Configures the limit-switch GPIOs, installs the interrupt handlers,
 * and starts the debounce handling required to generate stable
 * state-machine events.
 *
 * @return
 *      - ESP_OK on success
 *      - Appropriate esp_err_t error code on failure
 */
esp_err_t limitSwitchInit(void);


/**
 * @brief Synchronize the current physical limit-switch state with the
 *        state machine.
 *
 * Reads the upper and lower limit switches and posts the appropriate
 * state-machine event to establish the mechanism's physical position
 * after startup.
 *
 * If both limit switches are active simultaneously, a fault event is
 * posted because this represents an invalid physical condition.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the limit-switch driver is not initialized
 *      - Appropriate esp_err_t error code if event posting fails
 */
esp_err_t limitSwitchSyncState(void);


#endif /* COMPONENTS_LIMIT_SWITCH_H_ */