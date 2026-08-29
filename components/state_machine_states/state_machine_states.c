#include "state_machine_states.h"
#include "state_machine.h"

#include "state_machine_state_init.h"
#include "state_machine_state_stowed.h"
#include "state_machine_state_lowering.h"
#include "state_machine_state_deployed.h"
#include "state_machine_state_pumping.h"
#include "state_machine_state_raising.h"
#include "state_machine_state_pos_unknown.h"
#include "state_machine_state_fault.h"

#include "esp_log.h"


static const char* TAG = "SM_STATES";


esp_err_t stateMachineStatesRegister(void) {
    esp_err_t lErr = ESP_OK;

    // register init state as initial state
    lErr = stateMachineRegisterState(&g_stateMachineStateInit, true);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to register INIT state");
        goto end_register;
    }

    // register stowed state
    lErr = stateMachineRegisterState(&g_stateMachineStateStowed, false);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to register STOWED state");
        goto end_register;
    }

    // register lowering state
    lErr = stateMachineRegisterState(&g_stateMachineStateLowering, false);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to register LOWERING state");
        goto end_register;
    }

    // register deployed state
    lErr = stateMachineRegisterState(&g_stateMachineStateDeployed, false);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to register DEPLOYED state");
        goto end_register;
    }

    // register pumping state
    lErr = stateMachineRegisterState(&g_stateMachineStatePumping, false);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to register PUMPING state");
        goto end_register;
    }

    // register raising state
    lErr = stateMachineRegisterState(&g_stateMachineStateRaising, false);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to register RAISING state");
        goto end_register;
    }

    // register position unknown state
    lErr = stateMachineRegisterState(&g_stateMachineStatePositionUnknown, false);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to register POS_UNKNOWN state");
        goto end_register;
    }

    // register fault state
    lErr = stateMachineRegisterState(&g_stateMachineStateFault, false);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to register FAULT state");
        goto end_register;
    }

    // set failure state
    lErr = stateMachineSetFailureState(STATE_MACHINE_FAULT);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to set failure state. Code: 0x%X", lErr);
        goto end_register;
    }

    ESP_LOGI(TAG, "All states registered in state machine");

end_register:

    return lErr;
}