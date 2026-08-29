#include "motion_timeout.h"
#include "state_machine.h"

#include <stdbool.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#define MOTION_TIMEOUT_MS           10000

static const char* TAG = "motion_timeout";


static TimerHandle_t s_pTimerHandle = NULL;
static StaticTimer_t s_sTimerBuffer;
static bool s_isInitialized = false;


/* motion timeout callback */
static void s_motionTimeoutCallback(TimerHandle_t pTimer) {
    (void)pTimer;
    
    ESP_LOGE(TAG, "Motion timeout expired");

    esp_err_t lErr = ESP_OK;

    lErr = stateMachinePostEvent(SM_EVENT_FAULT);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to post fault event. Code: 0x%X", lErr);
    }
}

/* initialize motion timeout */
esp_err_t motionTimeoutInit(void) {
    esp_err_t lErr = ESP_OK;

    if(s_isInitialized) {
        ESP_LOGW(TAG, "Motion timeout already initialized");
        lErr = ESP_ERR_INVALID_STATE;

        goto end_init;
    }

    // create one-shot timer
    s_pTimerHandle = xTimerCreateStatic(
        "motion_timeout",
        pdMS_TO_TICKS(MOTION_TIMEOUT_MS),
        pdFALSE,
        NULL,
        s_motionTimeoutCallback,
        &s_sTimerBuffer
    );

    if(NULL == s_pTimerHandle) {
        ESP_LOGE(TAG, "Failed to create motion timeout timer");

        lErr = ESP_FAIL;
        goto end_init;
    }

    s_isInitialized = true;

    ESP_LOGI(TAG, "Motion timeout initialized. Timeout: %d ms", MOTION_TIMEOUT_MS);

end_init:

    return lErr;
}


/* start motion timeout */
esp_err_t motionTimeoutStart(void) {
    esp_err_t lErr = ESP_OK;

    if(!s_isInitialized) {
        ESP_LOGE(TAG, "Motion timeout not initialized.");
        lErr = ESP_ERR_INVALID_STATE;

        goto end_start;
    }

    BaseType_t xResult = xTimerReset(
        s_pTimerHandle,
        portMAX_DELAY
    );

    if(pdPASS != xResult) {
        ESP_LOGE(TAG, "Failed to start motion timeout");
        lErr = ESP_FAIL;

        goto end_start;
    }

    ESP_LOGI(TAG, "Motion timeout started");

end_start:

    return lErr;
}


/* stop motion timeout */
esp_err_t motionTimeoutStop(void) {
    esp_err_t lErr = ESP_OK;

    if(!s_isInitialized) {
        ESP_LOGE(TAG, "Motion timeout not initialized");
        lErr = ESP_ERR_INVALID_STATE;

        goto end_stop;
    }

    if(pdFALSE == xTimerIsTimerActive(s_pTimerHandle)) {
        goto end_stop;
    }

    BaseType_t xResult = xTimerStop(
        s_pTimerHandle,
        portMAX_DELAY
    );

    if(pdPASS != xResult) {
        ESP_LOGE(TAG, "Failed to stop motion timeout");
        lErr = ESP_FAIL;

        goto end_stop;
    }

    ESP_LOGI(TAG, "Motion timeout stopped");

end_stop:

    return lErr;
}


