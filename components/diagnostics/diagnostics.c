#include "diagnostics.h"

#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "ina226.h"
#include "ssd1306.h"
#include "state_machine.h"


/*          Task Configuration          */
#define DIAGNOSTICS_TASK_STACK_SIZE         4096
#define DIAGNOSTICS_TASK_PRIORITY           1
#define DIAGNOSTICS_UPDATE_PERIOD_MS        500


/*          Display Configuration       */
#define DIAGNOSTICS_TEXT_BUFFER_SIZE        22


static const char* TAG = "diagnostics";


// static vars
static TaskHandle_t s_pTaskHandle = NULL;
static bool s_isInitialized = false;
static StaticTask_t s_sTaskBuffer;
static StackType_t s_sTaskStack[DIAGNOSTICS_TASK_STACK_SIZE];


// get state names
static const char* s_stateName(StateMachineStateId_t eState) {
    switch(eState) {
        case STATE_MACHINE_INIT:                return "INIT";
        case STATE_MACHINE_STOWED:              return "STOWED";
        case STATE_MACHINE_LOWERING:            return "LOWERING";
        case STATE_MACHINE_DEPLOYED:            return "DEPLOYED";
        case STATE_MACHINE_PUMPING:             return "PUMPING";
        case STATE_MACHINE_RAISING:             return "RAISING";
        case STATE_MACHINE_POSITION_UNKNOWN:    return "POS_UNKNOWN";
        case STATE_MACHINE_FAULT:               return "FAULT";

        default:                                return "INVALID";
    }
}

// update display
static esp_err_t s_updateDisplay(void) {
    float fBusVoltageV = 0.0f;
    float fCurrentA = 0.0f;
    float fPowerW = 0.0f;

    esp_err_t lErr = ina226ReadBusVoltage(&fBusVoltageV);

    if(lErr) {
        return lErr;
    }

    lErr = ina226ReadCurrent(&fCurrentA);

    if(lErr) {
        return lErr;
    }

    lErr = ina226ReadPower(&fPowerW);

    if(lErr) {
        return lErr;
    }

    StateMachineStateId_t eState = stateMachineGetCurrentState();

    char cStateText[DIAGNOSTICS_TEXT_BUFFER_SIZE] = {0};
    char cVoltageText[DIAGNOSTICS_TEXT_BUFFER_SIZE] = {0};
    char cCurrentText[DIAGNOSTICS_TEXT_BUFFER_SIZE] = {0};
    char cPowerText[DIAGNOSTICS_TEXT_BUFFER_SIZE] = {0};


    snprintf(cStateText, sizeof(cStateText), "STATE: %s", s_stateName(eState));
    snprintf(cVoltageText, sizeof(cVoltageText), "BAT: %.2f V", fBusVoltageV);
    snprintf(cCurrentText, sizeof(cCurrentText), "CUR: %.3f A", fCurrentA);
    snprintf(cPowerText, sizeof(cPowerText), "PWR: %.2f W", fPowerW);


    lErr = ssd1306WriteText(0, "WATER SAMPLER");

    if(lErr) {
        return lErr;
    }

    lErr = ssd1306WriteText(2, cStateText);

    if(lErr) {
        return lErr;
    }

    lErr = ssd1306WriteText(4, cVoltageText);

    if(lErr) {
        return lErr;
    }

    lErr = ssd1306WriteText(5, cCurrentText);

    if(lErr) {
        return lErr;
    }

    lErr = ssd1306WriteText(6, cPowerText);

    if(lErr) {
        return lErr;
    }

    return ESP_OK;
}

// diagnostics task
static void s_diagnosticsTask(void* pParameters) {
    (void)pParameters;

    while(true) {
        esp_err_t lErr = s_updateDisplay();
        if(lErr) {
            ESP_LOGW(TAG, "Failed to update diagnostics display. Code: 0x%X", lErr);
        }

        vTaskDelay(pdMS_TO_TICKS(DIAGNOSTICS_UPDATE_PERIOD_MS));
    }
}


esp_err_t diagnosticsInit(void) {
    if(s_isInitialized) {
        ESP_LOGW(TAG, "Already initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    s_pTaskHandle = xTaskCreateStatic(
        s_diagnosticsTask,
        "diagnostics",
        DIAGNOSTICS_TASK_STACK_SIZE,
        NULL,
        DIAGNOSTICS_TASK_PRIORITY,
        s_sTaskStack,
        &s_sTaskBuffer
    );

    if(NULL == s_pTaskHandle) {
        ESP_LOGE(TAG, "Failed to create diagnostics task");

        return ESP_ERR_NO_MEM;
    }

    s_isInitialized = true;

    ESP_LOGI(TAG, "Diagnostics initialized");

    return ESP_OK;
}
