#ifndef COMPONENTS_MOTION_TIMEOUT_H_
#define COMPONENTS_MOTION_TIMEOUT_H_

#include "esp_err.h"

/**
 * @brief Initialize the motion timeout component.
 *
 * Creates the one-shot timer used to detect nozzle motion that
 * fails to reach an expected limit switch within the allowed time.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if already initialized
 *      - Appropriate esp_err_t error code on failure
 */
esp_err_t motionTimeoutInit(void);


/**
 * @brief Start the motion timeout.
 *
 * Starts the one-shot motion timer. If the timer is already active,
 * the timeout period is restarted.
 *
 * If the timer expires before being stopped, a state-machine fault
 * event is generated.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the component is not initialized
 *      - Appropriate esp_err_t error code on failure
 */
esp_err_t motionTimeoutStart(void);


/**
 * @brief Stop the motion timeout.
 *
 * Cancels the active motion timeout. Calling this function when the
 * timer is not running is safe.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the component is not initialized
 *      - Appropriate esp_err_t error code on failure
 */
esp_err_t motionTimeoutStop(void);

#endif /* COMPONENTS_MOTION_TIMEOUT_H_ */