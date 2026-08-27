#ifndef COMPONENTS_NOZZLE_SERVO_H_
#define COMPONENTS_NOZZLE_SERVO_H_

#include "esp_err.h"


/*
    * @brief Initialize the nozzle servo
    *
    * @return ESP_OK on success, or an error code on failure
*/
esp_err_t nozzleServoInit(void);

/*
    * @brief De-initialize the nozzle servo
    *
    * @return void
*/
void nozzleServoDeinit(void);

/*
    * @brief Extend the nozzle servo
    *
    * @return ESP_OK on success, or an error code on failure
*/
esp_err_t nozzleServoExtend(void);

/*
    * @brief Retract the nozzle servo
    *
    * @return ESP_OK on success, or an error code on failure
*/
esp_err_t nozzleServoRetract(void);

/*
    * @brief Stop the nozzle servo
    *
    * @return ESP_OK on success, or an error code on failure
*/
esp_err_t nozzleServoStop(void);


#endif /* COMPONENTS_NOZZLE_SERVO_H_ */