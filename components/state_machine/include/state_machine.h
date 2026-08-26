#ifndef COMPONENTS_STATE_MACHINE_H_
#define COMPONENTS_STATE_MACHINE_H_

#include "esp_err.h"
#include "state_machine_common.h"

esp_err_t stateMachineInit(void);

esp_err_t stateMachinePostEvent(StateMachineEventId_t eEventId);


#endif /* COMPONENTS_STATE_MACHINE_H_ */
