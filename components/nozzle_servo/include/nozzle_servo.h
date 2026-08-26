#ifndef COMPONENTS_NOZZLE_SERVO_H_
#define COMPONENTS_NOZZLE_SERVO_H_

#include "esp_err.h"

#define NOZZLE_SERVO_STOP_US      1500
#define NOZZLE_SERVO_EXTEND_US    1800
#define NOZZLE_SERVO_RETRACT_US   1200


esp_err_t nozzleServoInit(void);

esp_err_t nozzleServoExtend(void);

esp_err_t nozzleServoRetract(void);

esp_err_t nozzleServoStop(void);


#endif /* COMPONENTS_NOZZLE_SERVO_H_ */