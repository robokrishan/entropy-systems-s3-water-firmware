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
#include "freertos/queue.h"

#define RC_CAPTURE_GROUP            0
#define RC_INPUT_TASK_STACK         4096
#define RC_INPUT_TASK_PRIORITY      5
#define RC_INPUT_QUEUE_LENGTH       10

// signal config
#define RC_LOW_MAX_US               1200        // pwm deadband def
#define RC_HIGH_MIN_US              1500
#define RC_VALID_MIN_US             800
#define RC_VALID_MAX_US             2200
#define RC_SIGNAL_TIMEOUT_MS        500        // signal timeout

static const char* TAG = "rc_input";


// enum for PWM input signal state
typedef enum {
    RC_INPUT_STATE_UNKNOWN = 0,
    RC_INPUT_STATE_LOW,
    RC_INPUT_STATE_HIGH,
} RcInputState_t;


// enum for rc channel
typedef enum {
    RC_CHANNEL_PUMP = 0,
    RC_CHANNEL_NOZZLE,
    RC_CHANNEL_MAX
} RcChannelId_t;


// channel struct
typedef struct {
    RcChannelId_t eId;
    gpio_num_t eGpio;
    mcpwm_cap_channel_handle_t pCaptureChannel;
    uint32_t ulRisingEdgeValue;
    bool hasRisingEdge;
    RcInputState_t eCurrentState;
    bool isCaptureEnabled;
} RcInputChannel_t;


// packet sample struct for processing task
typedef struct {
    RcChannelId_t eChannel;
    uint32_t ulPulseWidthUs;
} RcInputSample_t;


static mcpwm_cap_timer_handle_t s_pCaptureTimer = NULL;
static TaskHandle_t s_pTaskHandle = NULL;
static bool s_isInitialized = false;
static uint32_t s_ulCaptureResolutionHz = 0;
static TimerHandle_t s_pSignalLossTimer = NULL;
static StaticTimer_t s_sSignalLossTimerBuffer;
static bool s_isSignalLost = false;
static QueueHandle_t s_pSampleQueue = NULL;
static StaticQueue_t s_sSampleQueueBuffer;
static uint8_t s_ubSampleQueueStorage[RC_INPUT_QUEUE_LENGTH*sizeof(RcInputSample_t)];
static bool s_isCaptureTimerEnabled = false;
static bool s_isCaptureTimerStarted = false;
static bool s_isShuttingDown = false;


static RcInputChannel_t s_sChannels[RC_CHANNEL_MAX] = {
    [RC_CHANNEL_PUMP] = {
        .eId = RC_CHANNEL_PUMP,
        .eGpio = CONFIG_PIN_RC_INPUT_PUMP,
        .pCaptureChannel = NULL,
        .ulRisingEdgeValue = 0,
        .hasRisingEdge = false,
        .eCurrentState = RC_INPUT_STATE_UNKNOWN,
        .isCaptureEnabled = false
    },

    [RC_CHANNEL_NOZZLE] = {
        .eId = RC_CHANNEL_NOZZLE,
        .eGpio = CONFIG_PIN_RC_INPUT_NOZZLE,
        .pCaptureChannel = NULL,
        .ulRisingEdgeValue = 0,
        .hasRisingEdge = false,
        .eCurrentState = RC_INPUT_STATE_UNKNOWN,
        .isCaptureEnabled = false
    }
};


/* pwm signal capture callback */
static bool s_captureCallback(
    mcpwm_cap_channel_handle_t pCaptureChannel,
    const mcpwm_capture_event_data_t* pEventData,
    void* pUserData
) {
    (void)pCaptureChannel;
    RcInputChannel_t* pChannel = (RcInputChannel_t*)pUserData;

    if(s_isShuttingDown || (NULL == pEventData) || (NULL == pChannel) ||
        (NULL == s_pSampleQueue) || (0 == s_ulCaptureResolutionHz)) {

        return false;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;


    if(MCPWM_CAP_EDGE_POS == pEventData->cap_edge) {
        
        // beginning of high pulse
        pChannel->ulRisingEdgeValue = pEventData->cap_value;
        pChannel->hasRisingEdge = true;

    } else if(MCPWM_CAP_EDGE_NEG == pEventData->cap_edge) {

        if(!pChannel->hasRisingEdge) {
            return false;
        }
        
        // end of high pulse
        uint32_t ulPulseWidthTicks = pEventData->cap_value - 
                                        pChannel->ulRisingEdgeValue;

        uint32_t ulPulseWidthUs = (uint32_t)(((uint64_t)ulPulseWidthTicks*1000000ULL) /
                                    s_ulCaptureResolutionHz);

        pChannel->hasRisingEdge = false;

        RcInputSample_t sSample = {
            .eChannel = pChannel->eId,
            .ulPulseWidthUs = ulPulseWidthUs
        };

        xQueueSendFromISR(s_pSampleQueue, &sSample, &xHigherPriorityTaskWoken);
    }

    return (pdTRUE == xHigherPriorityTaskWoken);
}


/* pwm signal loss callback */
static void s_signalLossCallback(TimerHandle_t pTimer) {
    (void)pTimer;

    if(s_isShuttingDown || !s_isInitialized) {
        return;
    }

    if(s_isSignalLost) {
        return;
    }

    s_isSignalLost = true;

    /* 
        force next valid rc command to be treated as 
        new state when signal returns
    */
    for(uint8_t i = 0; i < RC_CHANNEL_MAX; i++) {
        s_sChannels[i].eCurrentState = RC_INPUT_STATE_UNKNOWN;
        s_sChannels[i].hasRisingEdge = false;
    }

    ESP_LOGE(TAG, "RC signal lost");

    esp_err_t lErr = stateMachinePostEvent(SM_EVENT_RC_SIGNAL_LOST);

    if(lErr) {
        ESP_LOGE(TAG, "Failed to post signal loss event. Code: 0x%X", lErr);
    }
}


static esp_err_t s_processCommand(
    RcChannelId_t eChannel, 
    RcInputState_t eState
) {
    esp_err_t lErr = ESP_OK;

    switch(eChannel) {

        case RC_CHANNEL_PUMP:

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
            break;

        case RC_CHANNEL_NOZZLE:
            
            switch(eState) {
                case RC_INPUT_STATE_LOW:
                    ESP_LOGI(TAG, "RC command NOZZLE_RETRACT");
                    lErr = stateMachinePostEvent(SM_EVENT_NOZZLE_RETRACT);
                    break;

                case RC_INPUT_STATE_HIGH:
                    ESP_LOGI(TAG, "RC command NOZZLE_EXTEND");
                    lErr = stateMachinePostEvent(SM_EVENT_NOZZLE_EXTEND);
                    break;

                default:
                    ESP_LOGW(TAG, "Invalid RC input state");
                    return ESP_ERR_INVALID_ARG;
            }
            break;

        default:
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

    RcInputSample_t sSample;

    while(true) {

        if(pdTRUE != xQueueReceive(s_pSampleQueue, &sSample, portMAX_DELAY)) {
            continue;
        }

        if((uint32_t)sSample.eChannel >= (uint32_t)RC_CHANNEL_MAX) {
            ESP_LOGE(
                TAG,
                "Invalid RC channel received: %u",
                (unsigned int)sSample.eChannel
            );

            continue;
        }

        RcInputChannel_t* pChannel = &s_sChannels[sSample.eChannel];

        if((sSample.ulPulseWidthUs < RC_VALID_MIN_US) || (sSample.ulPulseWidthUs > RC_VALID_MAX_US)) {
            ESP_LOGW(TAG, "Invalid RC pulse width: %lu us",
                (unsigned long)sSample.ulPulseWidthUs
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

        if(sSample.ulPulseWidthUs <= RC_LOW_MAX_US) {
            eNewState = RC_INPUT_STATE_LOW;
        } else if(sSample.ulPulseWidthUs >= RC_HIGH_MIN_US) {
            eNewState = RC_INPUT_STATE_HIGH;
        } else { 
            continue;
        }


        // suppress duplicate commands
        if(eNewState == pChannel->eCurrentState) {
            continue;
        }

        if(RC_INPUT_STATE_LOW == eNewState) {
            ESP_LOGI(TAG, "RC input LOW");
        } else {
            ESP_LOGI(TAG, "RC input HIGH");
        }

        esp_err_t lErr = s_processCommand(pChannel->eId, eNewState);
        if(lErr) {
            ESP_LOGE(TAG, "Failed to process RC command. Code: 0x%X", lErr);

            continue;
        }

        // only record new state after successful event post
        pChannel->eCurrentState = eNewState;

#ifdef DEBUG
    ESP_LOGI(TAG, "RC pulse width: %lu us", (unsigned long)sSample.ulPulseWidthUs);
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


static esp_err_t s_captureChannelInit(RcInputChannel_t* pChannel) {
    esp_err_t lErr = ESP_OK;

    // configure pwm input capture
    mcpwm_capture_channel_config_t sChannelConfig = {
        .gpio_num = pChannel->eGpio,
        .prescale = 1,
        .flags.pos_edge = true,
        .flags.neg_edge = true
    };

    lErr = mcpwm_new_capture_channel(
        s_pCaptureTimer,
        &sChannelConfig,
        &pChannel->pCaptureChannel
    );

    if(lErr) {
        ESP_LOGE(TAG, "Failed to create capture channel %d. Code: 0x%X", 
            pChannel->eId,
            lErr
        );

        return lErr;
    }


    // register capture callback
    mcpwm_capture_event_callbacks_t sCallback = {
        .on_cap = s_captureCallback
    };

    lErr = mcpwm_capture_channel_register_event_callbacks(
        pChannel->pCaptureChannel,
        &sCallback,
        pChannel
    );

    if(lErr) {
        ESP_LOGE(TAG, "Failed to register capture callback for channel %d. \
            Code: 0x%X", 
            pChannel->eId,
            lErr
        );

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

    // create processing task before enabling capture interrupt
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

    for(uint8_t i = 0; i < RC_CHANNEL_MAX; i++) {
        lErr = mcpwm_capture_channel_enable(s_sChannels[i].pCaptureChannel);
        if(lErr) {
            ESP_LOGE(TAG, "Failed to enable capture channel %d. Code: 0x%X", 
                i,
                lErr
            );

            goto rollback_capture;
        }

        s_sChannels[i].isCaptureEnabled = true;
    }


    lErr = mcpwm_capture_timer_enable(s_pCaptureTimer);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to enable capture timer. Code: 0x%X", lErr);

        goto rollback_capture;
    }

    s_isCaptureTimerEnabled = true;


    lErr = mcpwm_capture_timer_start(s_pCaptureTimer);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to start capture timer. Code: 0x%X", lErr);

        goto rollback_capture;
    }

    s_isCaptureTimerStarted = true;

    return ESP_OK;


rollback_capture:

    if(s_isCaptureTimerStarted) {
        esp_err_t lStopErr =
            mcpwm_capture_timer_stop(s_pCaptureTimer);

        if(lStopErr) {
            ESP_LOGE(
                TAG,
                "Failed to stop capture timer during rollback. Code: 0x%X",
                lStopErr
            );
        } else {
            s_isCaptureTimerStarted = false;
        }
    }

    if(s_isCaptureTimerEnabled) {
        esp_err_t lDisableErr =
            mcpwm_capture_timer_disable(s_pCaptureTimer);

        if(lDisableErr) {
            ESP_LOGE(
                TAG,
                "Failed to disable capture timer during rollback. Code: 0x%X",
                lDisableErr
            );
        } else {
            s_isCaptureTimerEnabled = false;
        }
    }

    for(uint8_t i = 0; i < RC_CHANNEL_MAX; i++) {
        if(s_sChannels[i].isCaptureEnabled) {
            esp_err_t lDisableErr =
                mcpwm_capture_channel_disable(
                    s_sChannels[i].pCaptureChannel
                );

            if(lDisableErr) {
                ESP_LOGE(
                    TAG,
                    "Failed to disable capture channel %d during rollback. Code: 0x%X",
                    i,
                    lDisableErr
                );
            } else {
                s_sChannels[i].isCaptureEnabled = false;
            }
        }
    }

    return lErr;
}


static esp_err_t s_sampleQueueInit(void) {
    s_pSampleQueue = xQueueCreateStatic(
        RC_INPUT_QUEUE_LENGTH,
        sizeof(RcInputSample_t),
        s_ubSampleQueueStorage,
        &s_sSampleQueueBuffer
    );

    if(NULL == s_pSampleQueue) {
        ESP_LOGE(TAG, "Failed to create RC sample queue");

        return ESP_FAIL;
    }

    return ESP_OK;
}


static void s_cleanup(void) {
    esp_err_t lErr = ESP_OK;
    BaseType_t xResult = pdPASS;

    s_isShuttingDown = true;
    s_isInitialized = false;


    /*
     * Disable capture channels first so that no new ISR callbacks
     * can write samples into the queue during teardown.
     */
    for(uint8_t i = 0; i < RC_CHANNEL_MAX; i++) {
        if(
            (NULL != s_sChannels[i].pCaptureChannel) &&
            s_sChannels[i].isCaptureEnabled
        ) {
            lErr = mcpwm_capture_channel_disable(
                s_sChannels[i].pCaptureChannel
            );

            if(lErr) {
                ESP_LOGE(
                    TAG,
                    "Failed to disable capture channel %d during cleanup. Code: 0x%X",
                    i,
                    lErr
                );
            } else {
                s_sChannels[i].isCaptureEnabled = false;
            }
        }
    }


    /*
     * Stop and disable the capture timer.
     */
    if(
        (NULL != s_pCaptureTimer) &&
        s_isCaptureTimerStarted
    ) {
        lErr = mcpwm_capture_timer_stop(s_pCaptureTimer);

        if(lErr) {
            ESP_LOGE(
                TAG,
                "Failed to stop capture timer during cleanup. Code: 0x%X",
                lErr
            );
        } else {
            s_isCaptureTimerStarted = false;
        }
    }

    if(
        (NULL != s_pCaptureTimer) &&
        s_isCaptureTimerEnabled
    ) {
        lErr = mcpwm_capture_timer_disable(s_pCaptureTimer);

        if(lErr) {
            ESP_LOGE(
                TAG,
                "Failed to disable capture timer during cleanup. Code: 0x%X",
                lErr
            );
        } else {
            s_isCaptureTimerEnabled = false;
        }
    }


    /*
     * Stop the RC signal-loss timer.
     */
    if(NULL != s_pSignalLossTimer) {
        if(pdTRUE == xTimerIsTimerActive(s_pSignalLossTimer)) {
            xResult = xTimerStop(
                s_pSignalLossTimer,
                portMAX_DELAY
            );

            if(pdPASS != xResult) {
                ESP_LOGE(
                    TAG,
                    "Failed to stop signal-loss timer during cleanup"
                );
            }
        }
    }


    /*
     * Delete the processing task before deleting the queue it uses.
     */
    if(NULL != s_pTaskHandle) {
        vTaskDelete(s_pTaskHandle);
        s_pTaskHandle = NULL;
    }


    /*
     * Delete the signal-loss timer.
     */
    if(NULL != s_pSignalLossTimer) {
        xResult = xTimerDelete(
            s_pSignalLossTimer,
            portMAX_DELAY
        );

        if(pdPASS != xResult) {
            ESP_LOGE(
                TAG,
                "Failed to delete signal-loss timer during cleanup"
            );
        } else {
            s_pSignalLossTimer = NULL;
        }
    }


    /*
     * Delete the sample queue after both its producer and consumer
     * have been stopped.
     */
    if(NULL != s_pSampleQueue) {
        vQueueDelete(s_pSampleQueue);
        s_pSampleQueue = NULL;
    }


    /*
     * Delete capture channels.
     */
    for(uint8_t i = 0; i < RC_CHANNEL_MAX; i++) {
        if(NULL != s_sChannels[i].pCaptureChannel) {
            lErr = mcpwm_del_capture_channel(
                s_sChannels[i].pCaptureChannel
            );

            if(lErr) {
                ESP_LOGE(
                    TAG,
                    "Failed to delete capture channel %d. Code: 0x%X",
                    i,
                    lErr
                );
            } else {
                s_sChannels[i].pCaptureChannel = NULL;
                s_sChannels[i].isCaptureEnabled = false;
            }
        }

        s_sChannels[i].ulRisingEdgeValue = 0;
        s_sChannels[i].hasRisingEdge = false;
        s_sChannels[i].eCurrentState =
            RC_INPUT_STATE_UNKNOWN;
    }


    /*
     * Delete capture timer last.
     */
    if(NULL != s_pCaptureTimer) {
        lErr = mcpwm_del_capture_timer(
            s_pCaptureTimer
        );

        if(lErr) {
            ESP_LOGE(
                TAG,
                "Failed to delete capture timer. Code: 0x%X",
                lErr
            );
        } else {
            s_pCaptureTimer = NULL;
            s_isCaptureTimerEnabled = false;
            s_isCaptureTimerStarted = false;
        }
    }


    s_ulCaptureResolutionHz = 0;
    s_isSignalLost = false;
    s_isShuttingDown = false;

    ESP_LOGI(TAG, "RC input cleanup complete");
}

// rc_input initialization
esp_err_t rcInputInit(void) {
    esp_err_t lErr = ESP_OK;

    if(s_isInitialized || (NULL != s_pCaptureTimer) || (NULL != s_pTaskHandle) ||
        (NULL != s_pSignalLossTimer) || (NULL != s_pSampleQueue)) {

        ESP_LOGE(TAG, "RC input cannot initialize while resources are still allocated");

        return ESP_ERR_INVALID_STATE;
    }

    for(uint8_t i = 0; i < RC_CHANNEL_MAX; i++) {
        if(NULL != s_sChannels[i].pCaptureChannel) {
            ESP_LOGE(TAG, "RC capture channel %d is still allocated", i);

            return ESP_ERR_INVALID_STATE;
        }
    }


    lErr = s_captureTimerInit();
    if(lErr) {
        goto init_failed;
    }


    lErr = s_sampleQueueInit();
    if(lErr) {
        goto init_failed;
    }


    for(uint8_t i = 0; i < RC_CHANNEL_MAX; i++) {
        lErr = s_captureChannelInit(&s_sChannels[i]);

        if(lErr) {
            goto init_failed;
        }
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


void rcInputDeinit(void) {
    s_cleanup();
}
