#include "limit_switch.h"
#include "state_machine.h"
#include "device_config.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define DEBOUNCE_TIME_MS            30
#define LIMIT_SWITCH_ACTIVE         0
#define LIMIT_SWITCH_RELEASED       1
#define UPPER_LIMIT_NOTIFY_BIT      (1UL << 0)
#define LOWER_LIMIT_NOTIFY_BIT      (1UL << 1)
#define LIMIT_SWITCH_TASK_STACK     2048
#define LIMIT_SWITCH_TASK_PRIORITY  5


static const char* TAG = "limit_switch";


static TaskHandle_t s_pTaskHandle = NULL;
static bool s_isInitialized = false;
static bool s_isInvalidLimitState = false;
static bool s_isGpioConfigured = false;
static bool s_isUpperIsrAdded = false;
static bool s_isLowerIsrAdded = false;
static bool s_isShuttingDown = false;

static int s_lUpperStableLevel = LIMIT_SWITCH_RELEASED;
static int s_lLowerStableLevel = LIMIT_SWITCH_RELEASED;


/* process both switches active state */
static esp_err_t s_postFaultEvent(void) {
    esp_err_t lErr = stateMachinePostEvent(SM_EVENT_FAULT);

    if(lErr) {
        ESP_LOGE(TAG, "Failed to post fault event. Code: 0x%X", lErr);
    }

    return lErr;
}


/* process stable upper limit switch change */
static void s_processUpperLimit(void) {
    int lLevel = gpio_get_level(CONFIG_PIN_UPPER_LIMIT);

    if(lLevel == s_lUpperStableLevel) {
        return;
    }

    s_lUpperStableLevel = lLevel;

    if(LIMIT_SWITCH_ACTIVE == lLevel) {
        ESP_LOGI(TAG, "Upper limit active");

        esp_err_t lPostErr = stateMachinePostEvent(
            SM_EVENT_UPPER_LIMIT_ACTIVE
        );

        if(lPostErr) {
            ESP_LOGE(TAG, 
                "Failed to post upper limit active event. Code: 0x%X",
                lPostErr
            );
        }
    } else {
        ESP_LOGI(TAG, "Upper limit released");

        esp_err_t lPostErr = stateMachinePostEvent(
            SM_EVENT_UPPER_LIMIT_RELEASED
        );

        if(lPostErr) {
            ESP_LOGE(TAG, 
                "Failed to post upper limit released event. Code: 0x%X",
                lPostErr
            );
        }
    }
}


/* process stable lower limit switch change */
static void s_processLowerLimit(void) {
    int lLevel = gpio_get_level(CONFIG_PIN_LOWER_LIMIT);

    if(lLevel == s_lLowerStableLevel) {
        return;
    }

    s_lLowerStableLevel = lLevel;

    if(LIMIT_SWITCH_ACTIVE == lLevel) {
        ESP_LOGI(TAG, "Lower limit active");

        esp_err_t lPostErr = stateMachinePostEvent(
            SM_EVENT_LOWER_LIMIT_ACTIVE
        );

        if(lPostErr) {
            ESP_LOGE(TAG, 
                "Failed to post lower limit active event. Code: 0x%X",
                lPostErr
            );
        }

    } else {
        ESP_LOGI(TAG, "Lower limit released");

        esp_err_t lPostErr = stateMachinePostEvent(SM_EVENT_LOWER_LIMIT_RELEASED);

        if(lPostErr) {
            ESP_LOGE(TAG, 
                "Failed to post lower limit released event. Code: 0x%X",
                lPostErr
            );
        }
    }
}


/* upper limit interrupt */
static void IRAM_ATTR s_upperLimitISR(void* pArg) {
    (void)pArg;

    if(s_isShuttingDown || (NULL == s_pTaskHandle)) {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xTaskNotifyFromISR(
        s_pTaskHandle,
        UPPER_LIMIT_NOTIFY_BIT,
        eSetBits,
        &xHigherPriorityTaskWoken
    );

    if(pdTRUE == xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}


/* lower limit interrupt */
static void IRAM_ATTR s_lowerLimitISR(void* pArg) {
    (void)pArg;

    if(s_isShuttingDown || (NULL == s_pTaskHandle)) {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xTaskNotifyFromISR(
        s_pTaskHandle,
        LOWER_LIMIT_NOTIFY_BIT,
        eSetBits,
        &xHigherPriorityTaskWoken
    );

    if(pdTRUE == xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}


/* limit switch debounce task */
static void s_limitSwitchTask(void* pArgs) {
    uint32_t ulNotificationValue = 0;

    while(true) {
        xTaskNotifyWait(
            0,
            UINT32_MAX,
            &ulNotificationValue,
            portMAX_DELAY
        );

        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME_MS));

        int lUpperLevel = gpio_get_level(CONFIG_PIN_UPPER_LIMIT);
        int lLowerLevel = gpio_get_level(CONFIG_PIN_LOWER_LIMIT);

        if((LIMIT_SWITCH_ACTIVE == lUpperLevel) &&
            (LIMIT_SWITCH_ACTIVE == lLowerLevel)) {

            s_lUpperStableLevel = lUpperLevel;
            s_lLowerStableLevel = lLowerLevel;

            if(!s_isInvalidLimitState) {
                ESP_LOGE(TAG, "Invalid limit state: both switches active");

                s_isInvalidLimitState = true;
                s_postFaultEvent();
            }

            continue;
        }

        s_isInvalidLimitState = false;

        if(ulNotificationValue & UPPER_LIMIT_NOTIFY_BIT) {
            s_processUpperLimit();
        }

        if(ulNotificationValue & LOWER_LIMIT_NOTIFY_BIT) {
            s_processLowerLimit();
        }
    }
}


static void s_cleanup(void) {
    
}


/* initialize limit switch driver */
esp_err_t limitSwitchInit(void) {
    esp_err_t lErr = ESP_OK;

    if(s_isInitialized) {
        ESP_LOGW(TAG, "Limit switches already initialized");
        lErr = ESP_ERR_INVALID_STATE;

        goto end_init;
    }


    // configure both switches as active-low
    gpio_config_t sConfig = {
        .pin_bit_mask = (1ULL << CONFIG_PIN_UPPER_LIMIT) |
                        (1ULL << CONFIG_PIN_LOWER_LIMIT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };

    lErr = gpio_config(&sConfig);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to configure limit switches. Code: 0x%X", lErr);
        goto end_init;
    }


    // record switch state at startup
    s_lUpperStableLevel = gpio_get_level(CONFIG_PIN_UPPER_LIMIT);
    s_lLowerStableLevel = gpio_get_level(CONFIG_PIN_LOWER_LIMIT);


    // create debounce task
    BaseType_t xResult = xTaskCreate(
        s_limitSwitchTask,
        "limit_switch",
        LIMIT_SWITCH_TASK_STACK,
        NULL,
        LIMIT_SWITCH_TASK_PRIORITY,
        &s_pTaskHandle
    );

    if(pdPASS != xResult) {
        ESP_LOGE(TAG, "Failed to create limit switch task");
        s_pTaskHandle = NULL;

        lErr = ESP_FAIL;

        goto end_init;
    }


    // install ISR service
    lErr = gpio_install_isr_service(0);

    if((ESP_OK != lErr) && (ESP_ERR_INVALID_STATE != lErr)) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service. Code: 0x%X", lErr);

        vTaskDelete(s_pTaskHandle);
        s_pTaskHandle = NULL;

        goto end_init;
    }

    lErr = gpio_isr_handler_add(CONFIG_PIN_UPPER_LIMIT, s_upperLimitISR, NULL);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to add upper limit ISR. Code: 0x%X", lErr);

        vTaskDelete(s_pTaskHandle);
        s_pTaskHandle = NULL;

        goto end_init;
    }


    lErr = gpio_isr_handler_add(CONFIG_PIN_LOWER_LIMIT, s_lowerLimitISR, NULL);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to add lower limit ISR. Code: 0x%X", lErr);

        gpio_isr_handler_remove(CONFIG_PIN_UPPER_LIMIT);

        vTaskDelete(s_pTaskHandle);
        s_pTaskHandle = NULL;

        goto end_init;
    }

    s_isInitialized = true;

    ESP_LOGI(TAG, "Limit switches initialized. Upper: %d | Lower: %d",
        s_lUpperStableLevel,
        s_lLowerStableLevel
    );

end_init:

    return lErr;
}


esp_err_t limitSwitchSyncState(void) {
    esp_err_t lErr = ESP_OK;

    if(!s_isInitialized) {
        ESP_LOGE(TAG, "Limit switches not initialized");
        lErr = ESP_ERR_INVALID_STATE;

        goto end_sync;
    }

    // allow switch contact to settle
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME_MS));

    int lUpperLevel = gpio_get_level(CONFIG_PIN_UPPER_LIMIT);
    int lLowerLevel = gpio_get_level(CONFIG_PIN_LOWER_LIMIT);

    s_lUpperStableLevel = lUpperLevel;
    s_lLowerStableLevel = lLowerLevel;

    if((LIMIT_SWITCH_ACTIVE == lUpperLevel) && 
        (LIMIT_SWITCH_ACTIVE == lLowerLevel)) {
            ESP_LOGE(TAG, "Invalid limit state: both switches active");
            
            lErr = s_postFaultEvent();
    } else if(LIMIT_SWITCH_ACTIVE == lUpperLevel) {
        ESP_LOGI(TAG, "Initial position: upper limit active");

        lErr = stateMachinePostEvent(SM_EVENT_UPPER_LIMIT_ACTIVE);
    } else if(LIMIT_SWITCH_ACTIVE == lLowerLevel) {
        ESP_LOGI(TAG, "Initial position: lower limit active");

        lErr = stateMachinePostEvent(SM_EVENT_LOWER_LIMIT_ACTIVE);
    } else {
        ESP_LOGI(TAG, "Initial position unknown");
    }

    if(lErr) {
        ESP_LOGE(TAG, "Failed to sync limit switch state. Code: 0x%X", lErr);
    }

end_sync:

    return lErr;
}

