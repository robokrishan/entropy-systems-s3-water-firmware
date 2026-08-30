#include "rc_input.h"
#include "device_config.h"

#include <stdbool.h>
#include <stdint.h>
#include "driver/mcpwm_cap.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RC_CAPTURE_GROUP            0
#define RC_INPUT_TASK_STACK         2048
#define RC_INPUT_TASK_PRIORITY      5

static const char* TAG = "rc_input";

static mcpwm_cap_timer_handle_t s_pCaptureTimer = NULL;
static mcpwm_cap_channel_handle_t s_pCaptureChannel = NULL;
static TaskHandle_t s_pTaskHandle = NULL;
static bool s_isInitialized = false;
static uint32_t s_ulCaptureResolutionHz = 0;


/* pwm signal capture callback */
static bool s_captureCallback(
    mcpwm_cap_channel_handle_t pCaptureChannel,
    const mcpwm_capture_event_data_t* pEventData,
    void* pUserData
) {
    (void)pCaptureChannel;
    (void)pUserData;

    static uint32_t s_ulRisingEdgeValue = 0;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(MCPWM_CAP_EDGE_POS == pEventData->cap_edge) {
        
        // beginning of high pulse
        s_ulRisingEdgeValue = pEventData->cap_value;

    } else if(MCPWM_CAP_EDGE_NEG == pEventData->cap_edge) {
        
        // end of high pulse
        uint32_t ulPulseWidthTicks = pEventData->cap_value - s_ulRisingEdgeValue;
        uint32_t ulPulseWidthUs = (uint32_t)(((uint64_t)ulPulseWidthTicks*1000000ULL) /
                                    s_ulCaptureResolutionHz);

        xTaskNotifyFromISR(
            s_pTaskHandle,
            ulPulseWidthUs,
            eSetValueWithOverwrite,
            &xHigherPriorityTaskWoken
        );
    }

    return (pdTRUE == xHigherPriorityTaskWoken);
}


static void s_rcInputTask(void* pArg) {
    (void)pArg;

    uint32_t ulPulseWidthUs = 0;

    while(true) {
        
        // wait for pulse-width measurement from capture ISR
        xTaskNotifyWait(
            0,
            UINT32_MAX,
            &ulPulseWidthUs,
            portMAX_DELAY
        );

        ESP_LOGI(TAG, "RC pulse width: %lu us", (unsigned long)ulPulseWidthUs);
    }
}


// rc_input initialization
esp_err_t rcInputInit(void) {
    esp_err_t lErr = ESP_OK;

    if(s_isInitialized) {
        ESP_LOGW(TAG, "Already initialized");
        lErr = ESP_ERR_INVALID_STATE;

        goto end_init;
    }

    // create mcpwm capture timer config
    mcpwm_capture_timer_config_t sTimerConfig = {
        .group_id = RC_CAPTURE_GROUP,
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .resolution_hz = s_ulCaptureResolutionHz
    };

    // configure mcpwm capture timer
    lErr = mcpwm_new_capture_timer(
        &sTimerConfig,
        &s_pCaptureTimer
    );

    if(lErr) {
        ESP_LOGE(TAG, "Failed to create capture timer. Code: 0x%X", lErr);

        goto end_init;
    }


    // get timer resolution
    lErr = mcpwm_capture_timer_get_resolution(
        s_pCaptureTimer,
        &s_ulCaptureResolutionHz
    );

    if(lErr) {
        ESP_LOGE(TAG, "Failed to get capture timer resolution. Code: 0x%X", lErr);

        goto cleanup_timer;
    }

    ESP_LOGI(TAG, "Capture timer resolution: %lu Hz",
        (unsigned long)s_ulCaptureResolutionHz
    );


    // configure pwm input capture
    mcpwm_capture_channel_config_t sChannelConfig = {
        .gpio_num = CONFIG_PIN_RC_INPUT,
        .prescale = 1,
        .flags.pos_edge = true,
        .flags.neg_edge = true
    };

    lErr = mcpwm_new_capture_channel(
        s_pCaptureTimer,
        &sChannelConfig,
        &s_pCaptureChannel
    );

    if(lErr) {
        ESP_LOGE(TAG, "Failed to create capture channel. Code: 0x%X", lErr);

        goto cleanup_timer;
    }


    // register capture callback
    mcpwm_capture_event_callbacks_t sCallback = {
        .on_cap = s_captureCallback
    };

    lErr = mcpwm_capture_channel_register_event_callbacks(
        s_pCaptureChannel,
        &sCallback,
        NULL
    );

    if(lErr) {
        ESP_LOGE(TAG, "Failed to register capture callback. Code: 0x%X", lErr);

        goto cleanup_channel;
    }


    // cretae processing task before enabling capture interrupt
    BaseType_t xResult = xTaskCreate(
        s_rcInputTask,
        "rc_input",
        RC_INPUT_TASK_STACK,
        NULL,
        RC_INPUT_TASK_PRIORITY,
        &s_pTaskHandle
    );

    if(pdPASS != xResult) {
        ESP_LOGE(TAG, "Failed to create RC input task");

        s_pTaskHandle = NULL;
        lErr = ESP_FAIL;

        goto cleanup_channel;
    }

    lErr = mcpwm_capture_channel_enable(s_pCaptureChannel);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to enable capture channel. Code: 0x%X", lErr);

        goto cleanup_task;
    }

    lErr = mcpwm_capture_timer_enable(s_pCaptureTimer);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to enable capture timer. Code: 0x%X", lErr);

        goto cleanup_enabled_channel;
    }

    lErr = mcpwm_capture_timer_start(s_pCaptureTimer);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to start capture timer. Code: 0x%X", lErr);

        goto cleanup_enabled_timer;
    }


    s_isInitialized = true;
    ESP_LOGI(TAG, "RC Input initialized");
    goto end_init;

cleanup_enabled_timer:

    mcpwm_capture_timer_disable(s_pCaptureTimer);

cleanup_enabled_channel:

    mcpwm_capture_channel_disable(s_pCaptureChannel);

cleanup_task:

    vTaskDelete(s_pTaskHandle);
    s_pTaskHandle = NULL;

cleanup_channel:

    mcpwm_del_capture_channel(s_pCaptureChannel);
    s_pCaptureChannel = NULL;

cleanup_timer:

    mcpwm_del_capture_timer(s_pCaptureTimer);
    s_pCaptureTimer = NULL;

end_init:

    return lErr;
}