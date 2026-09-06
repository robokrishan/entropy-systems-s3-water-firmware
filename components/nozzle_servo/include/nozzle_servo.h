#ifndef COMPONENTS_NOZZLE_SERVO_H_
#define COMPONENTS_NOZZLE_SERVO_H_

#include "esp_err.h"


/**
 * @brief Initialize the nozzle servo component.
 *
 * Configures the MCPWM timer, operator, comparator, and other resources
 * required to control the continuous-rotation nozzle servo.
 *
 * The PWM output remains disabled after initialization until
 * nozzleServoEnable() is called.
 *
 * @return
 *      - ESP_OK on success
 *      - Appropriate esp_err_t error code on failure
 */
esp_err_t nozzleServoInit(void);


/**
 * @brief Deinitialize the nozzle servo component.
 *
 * Disables the servo PWM output and releases all resources owned by the
 * nozzle servo component.
 *
 * Cleanup is null-safe and best-effort so the function can be called after
 * partial initialization or repeated during shutdown.
 */
void nozzleServoDeinit(void);


/**
 * @brief Configure the nozzle servo for extension.
 *
 * Sets the servo PWM pulse width to the configured extension value.
 * The servo will only receive the command while PWM output is enabled.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the component is not initialized
 *      - Appropriate esp_err_t error code on driver failure
 */
esp_err_t nozzleServoExtend(void);


/**
 * @brief Configure the nozzle servo for retraction.
 *
 * Sets the servo PWM pulse width to the configured retraction value.
 * The servo will only receive the command while PWM output is enabled.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the component is not initialized
 *      - Appropriate esp_err_t error code on driver failure
 */
esp_err_t nozzleServoRetract(void);


/**
 * @brief Configure the nozzle servo with the neutral pulse width.
 *
 * Sets the servo PWM pulse width to the configured neutral value intended
 * to stop continuous rotation. This does not disable the PWM output.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the component is not initialized
 *      - Appropriate esp_err_t error code on driver failure
 */
esp_err_t nozzleServoStop(void);


/**
 * @brief Enable the nozzle servo PWM output.
 *
 * Connects the configured MCPWM signal to the servo output GPIO so that
 * the currently selected pulse width is driven to the servo.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the component is not initialized or the
 *        output is already enabled
 *      - Appropriate esp_err_t error code on driver failure
 */
esp_err_t nozzleServoEnable(void);


/**
 * @brief Disable the nozzle servo PWM output.
 *
 * Disconnects the MCPWM signal from the servo output and releases the GPIO
 * so that no PWM signal is driven while the mechanism is stationary.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the component is not initialized
 *      - Appropriate esp_err_t error code on driver failure
 */
esp_err_t nozzleServoDisable(void);

#endif /* COMPONENTS_NOZZLE_SERVO_H_ */