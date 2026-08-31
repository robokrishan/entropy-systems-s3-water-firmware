#ifndef COMPONENTS_INA226_H_
#define COMPONENTS_INA226_H_

#include "esp_err.h"


esp_err_t ina226Init(void);

esp_err_t ina226ReadBusVoltage(float* pVoltageV);

#endif /* COMPONENTS_INA226_H_ */