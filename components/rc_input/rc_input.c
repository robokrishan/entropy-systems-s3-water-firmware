#include "rc_input.h"
#include "device_config.h"
#include "state_machine.h"

#include <stdbool.h>
#include <stdint.h>
#include "driver/mcpwm_cap.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#define RC_CAPTURE_GROUP            0
#define RC_INPUT_TASK_STACK         2048
#define RC_INPUT_TASK_PRIORITY      5

// signal config
#define RC_LOW_MAX_US               1200        // pwm deadband def
#define RC_HIGH_MIN_US              1500
#define RC_VALID_MIN_US             800
#define RC_VALID_MAX_US             2200
#define RC_SIGNAL_TIMEOUT_MS        500        // signal timeout

static const char* TAG = "rc_input";


typedef enum {
    RC_INPUT_STATE_UNKNOWN = 0,
    RC_INPUT_STATE_LOW,
    RC_INPUT_STATE_HIGH,
} RcInputState_t;

static mcpwm_cap_timer_handle_t s_pCaptureTimer = NULL;
static mcpwm_cap_channel_handle_t s_pCaptureChannel = NULL;
static TaskHandle_t s_pTaskHandle = NULL;
static bool s_isInitialized = false;
static uint32_t s_ulCaptureResolutionHz = 0;
static RcInputState_t s_eCurrentState = RC_INPUT_STATE_UNKNOWN;
static TimerHandle_t s_pSignalLossTimer = NULL;
static StaticTimer_t s_sSignalLossTimerBuffer;
static bool s_isSignalLost = false;


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


/* pwm signal loss callback */
static void s_signalLossCallback(TimerHandle_t pTimer) {
    (void)pTimer;

    if(s_isSignalLost) {
        return;
    }

    s_isSignalLost = true;

    /* 
        force next valid rc command to be treated as 
        new state when signal returns
    */
   s_eCurrentState = RC_INPUT_STATE_UNKNOWN;

   ESP_LOGE(TAG, "RC signal lost");

   esp_err_t lErr = stateMachinePostEvent(SM_EVENT_RC_SIGNAL_LOST);

   if(lErr) {
    ESP_LOGE(TAG, "Failed to post signal loss event. Code: 0x%X", lErr);
   }
}


static esp_err_t s_processCommand(RcInputState_t eState) {
    esp_err_t lErr = ESP_OK;

    switch(eState) {
        case RC_INPUT_STATE_HIGH:
            ESP_LOGI(TAG, "RC command PUMP_OFF");
            lErr = stateMachinePostEvent(SM_EVENT_PUMP_OFF);
            break;

        case RC_INPUT_STATE_LOW:
            ESP_LOGI(TAG, "RC command PUMP_ON");
            lErr = stateMachinePostEvent(SM_EVENT_PUMP_ON);
            break;

        default:
            ESP_LOGW(TAG, "Invalid RC input state");
            return ESP_ERR_INVALID_ARG;
    }

    if(lErr) {
        ESP_LOGE(TAG, "Failed to post RC command event. Code: 0x%X", lErr);
    }

    return lErr;
}


/* rtos task for reading pwm signal */
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

        if((ulPulseWidthUs < RC_VALID_MIN_US) || (ulPulseWidthUs > RC_VALID_MAX_US)) {
            ESP_LOGW(TAG, "Invalid RC pulse width: %lu us",
                (unsigned long)ulPulseWidthUs
            );

            continue;
        }

        BaseType_t xResult = xTimerReset(s_pSignalLossTimer, portMAX_DELAY);
        if(pdPASS != xResult) {
            ESP_LOGE(TAG, "Failed to reset signal loss timer");
            continue;
        }

        if(s_isSignalLost) {
            s_isSignalLost = false;
            ESP_LOGI(TAG, "signal restored");
        }

        RcInputState_t eNewState = RC_INPUT_STATE_UNKNOWN;

        if(ulPulseWidthUs <= RC_LOW_MAX_US) {
            eNewState = RC_INPUT_STATE_LOW;
        } else if(ulPulseWidthUs >= RC_HIGH_MIN_US) {
            eNewState = RC_INPUT_STATE_HIGH;
        } else { 
            continue;
        }


        // suppress duplicate commands
        if(eNewState == s_eCurrentState) {
            continue;
        }

        if(RC_INPUT_STATE_LOW == eNewState) {
            ESP_LOGI(TAG, "RC input LOW");
        } else {
            ESP_LOGI(TAG, "RC input HIGH");
        }

        esp_err_t lErr = s_processCommand(eNewState);
        if(lErr) {
            ESP_LOGE(TAG, "Failed to process RC command. Code: 0x%X", lErr);

            continue;
        }

        // only record new state after successful event post
        s_eCurrentState = eNewState;

#ifdef DEBUG
        ESP_LOGI(TAG, "RC pulse width: %lu us", (unsigned long)ulPulseWidthUs);
#endif
    }
}


static esp_err_t s_captureTimerInit(void) {
    esp_err_t lErr = ESP_OK;

    // create mcpwm capture timer config
    mcpwm_capture_timer_config_t sTimerConfig = {
        .group_id = RC_CAPTURE_GROUP,
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .resolution_hz = 0  // default resolution
    };

    // configure mcpwm capture timer
    lErr = mcpwm_new_capture_timer(
        &sTimerConfig,
        &s_pCaptureTimer
    );

    if(lErr) {
        ESP_LOGE(TAG, "Failed to create capture timer. Code: 0x%X", lErr);

        return lErr;
    }


    // get timer resolution
    lErr = mcpwm_capture_timer_get_resolution(
        s_pCaptureTimer,
        &s_ulCaptureResolutionHz
    );

    if(lErr) {
        ESP_LOGE(TAG, "Failed to get capture timer resolution. Code: 0x%X", lErr);

        return lErr;
    }

    ESP_LOGI(TAG, "Capture timer resolution: %lu Hz",
        (unsigned long)s_ulCaptureResolutionHz
    );

    return ESP_OK;
}


static esp_err_t s_captureChannelInit(void) {
    esp_err_t lErr = ESP_OK;

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

        return lErr;
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

        return lErr;
    }

    return ESP_OK;
}


static esp_err_t s_signalLossTimerInit(void) {
    // create signal loss timer
    s_pSignalLossTimer = xTimerCreateStatic(
        "rc_signal_loss",
        pdMS_TO_TICKS(RC_SIGNAL_TIMEOUT_MS),
        pdFALSE,
        NULL,
        s_signalLossCallback,
        &s_sSignalLossTimerBuffer
    );

    if(NULL == s_pSignalLossTimer) {
        ESP_LOGE(TAG, "Failed to create signal loss timer");

        return ESP_FAIL;
    }

    return ESP_OK;
}


static esp_err_t s_processingTaskInit(void) {

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
        return ESP_FAIL;
    }

    return ESP_OK;
}


static esp_err_t s_signalLossTimerStart(void) {
    BaseType_t xTimerResult = xTimerStart(
        s_pSignalLossTimer,
        portMAX_DELAY
    );

    if(pdPASS != xTimerResult) {
        ESP_LOGE(TAG, "Failed to start signal loss timer");

        return ESP_FAIL;
    }

    return ESP_OK;
}


static esp_err_t s_captureStart(void) {
    esp_err_t lErr = ESP_OK;

    lErr = mcpwm_capture_channel_enable(s_pCaptureChannel);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to enable capture channel. Code: 0x%X", lErr);

        return lErr;
    }

    lErr = mcpwm_capture_timer_enable(s_pCaptureTimer);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to enable capture timer. Code: 0x%X", lErr);

        mcpwm_capture_channel_disable(s_pCaptureChannel);

        return lErr;
    }

    lErr = mcpwm_capture_timer_start(s_pCaptureTimer);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to start capture timer. Code: 0x%X", lErr);

        mcpwm_capture_timer_disable(s_pCaptureTimer);
        mcpwm_capture_channel_disable(s_pCaptureChannel);

        return lErr;
    }

    return ESP_OK;
}


static void s_cleanup(void) {

    /* stop signal loss timer if created and started*/
    if(NULL != s_pSignalLossTimer) {

        if(pdTRUE == xTimerIsTimerActive(s_pSignalLossTimer)) {
            xTimerStop(s_pSignalLossTimer, portMAX_DELAY);
        }
    }

    if(NULL != s_pTaskHandle) {
        vTaskDelete(s_pTaskHandle);
        s_pTaskHandle = NULL;
    }

    if(NULL != s_pSignalLossTimer) {
        xTimerDelete(s_pSignalLossTimer, portMAX_DELAY);

        s_pSignalLossTimer = NULL;
    }

    if(NULL != s_pCaptureChannel) {
        mcpwm_del_capture_channel(s_pCaptureChannel);
        
        s_pCaptureChannel = NULL;
    }

    if(NULL != s_pCaptureTimer) {
        mcpwm_del_capture_timer(s_pCaptureTimer);

        s_pCaptureTimer = NULL;
    }

    s_ulCaptureResolutionHz = 0;
    s_eCurrentState = RC_INPUT_STATE_UNKNOWN;
    s_isSignalLost = false;
    s_isInitialized = false;
}

// rc_input initialization
esp_err_t rcInputInit(void) {
    esp_err_t lErr = ESP_OK;

    if(s_isInitialized) {
        ESP_LOGW(TAG, "Already initialized");

        return ESP_ERR_INVALID_STATE;
    }

    lErr = s_captureTimerInit();
    if(lErr) {
        goto init_failed;
    }


    lErr = s_captureChannelInit();
    if(lErr) {
        goto init_failed;
    }


    lErr = s_signalLossTimerInit();
    if(lErr) {
        goto init_failed;
    }
    

    lErr = s_processingTaskInit();
    if(lErr) {
        goto init_failed;
    }
    

    lErr = s_signalLossTimerStart();
    if(lErr) {
        goto init_failed;
    }


    lErr = s_captureStart();
    if(lErr) {
        goto init_failed;
    }
    

    s_isInitialized = true;
    ESP_LOGI(TAG, "RC Input initialized");
    
    return ESP_OK;

init_failed:

    ESP_LOGE(TAG, "Failed to initialize RC input. Code: 0x%X", lErr);

    s_cleanup();

    return lErr;
}


