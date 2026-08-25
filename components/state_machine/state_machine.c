#include "state_machine.h"
#include "state_machine_state.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define SM_QUEUE_LENGTH     10
#define SM_TASK_STACK       4096
#define SM_TASK_PRIORITY    5

static const char* TAG = "SM";

static QueueHandle_t s_pQueue = NULL;

static StateMachineState_t s_eCurrentState = STATE_MACHINE_INIT;


static void s_stateMachineTransitionTo(StateMachineState_t eNewState) {
    if(eNewState == s_eCurrentState) {
        ESP_LOGW(TAG, "Already in state %s", StateMachineStateName(eNewState));
        return;
    }

    ESP_LOGI(TAG, "State transition:\t%s -> %s", 
        StateMachineStateName(s_eCurrentState), 
        StateMachineStateName(eNewState));

    s_eCurrentState = eNewState;

}


static void s_stateMachineProcessEvent(const StateMachineEvent_t* pEvent) {

    if(SM_EVENT_HALT == pEvent->eId) {
        s_stateMachineTransitionTo(STATE_MACHINE_IDLE);
        return;
    }

    if(SM_EVENT_FAULT == pEvent->eId) {
        s_stateMachineTransitionTo(STATE_MACHINE_FAULT);
        return;
    }

    switch (s_eCurrentState) {
        case STATE_MACHINE_INIT:
            switch(pEvent->eId) {
                case SM_EVENT_SYSTEM_READY:
                    s_stateMachineTransitionTo(STATE_MACHINE_IDLE);
                    break;

                default:
                    break;
            }
            break;

        case STATE_MACHINE_IDLE:
            switch (pEvent->eId) {
                case SM_EVENT_PUMP_ON:
                    s_stateMachineTransitionTo(STATE_MACHINE_PUMPING);
                    break;

                case SM_EVENT_LOWER_NOZZLE:
                    s_stateMachineTransitionTo(STATE_MACHINE_LOWERING);
                    break;

                case SM_EVENT_RAISE_NOZZLE:
                    s_stateMachineTransitionTo(STATE_MACHINE_RAISING);
                    break;

                default:
                    break;
            }

            break;

        case STATE_MACHINE_LOWERING:
            switch(pEvent->eId) {
                case SM_EVENT_STOP_SPOOL:
                case SM_EVENT_LIMIT_ACTIVE:
                    s_stateMachineTransitionTo(STATE_MACHINE_IDLE);
                    break;
                
                case SM_EVENT_RAISE_NOZZLE:
                    s_stateMachineTransitionTo(STATE_MACHINE_RAISING);
                    break;

                default:
                    ESP_LOGW(TAG, "Not allowed. Nozzle is lowering");
                    break;
            }
            break;

        case STATE_MACHINE_PUMPING:
            switch (pEvent->eId) {
                case SM_EVENT_PUMP_OFF:
                case SM_EVENT_RC_SIGNAL_LOST:
                    s_stateMachineTransitionTo(STATE_MACHINE_IDLE);
                    break;

                default:
                    ESP_LOGW(TAG, "Not allowed. Pump is active");
                    break;
            }
            break;

        case STATE_MACHINE_RAISING:
            switch(pEvent->eId) {
                case SM_EVENT_STOP_SPOOL:
                case SM_EVENT_LIMIT_ACTIVE:
                    s_stateMachineTransitionTo(STATE_MACHINE_IDLE);
                    break;

                default:
                    ESP_LOGW(TAG, "Not allowed. Nozzle is raising");
                    break;
            }
            break;

        case STATE_MACHINE_FAULT:
            switch(pEvent->eId) {
                case SM_EVENT_RESET:
                    s_stateMachineTransitionTo(STATE_MACHINE_IDLE);
                    break;

                default:
                    ESP_LOGW(TAG, "Not allowed. System is in fault state");
                    break;
            }
            break;

        default:
            break;
    }

}


static void s_pTask(void* arg) {
    StateMachineEvent_t s_sEvent;

    while(true) {
        if(pdTRUE == xQueueReceive(s_pQueue, &s_sEvent, portMAX_DELAY)){
            ESP_LOGW(TAG, "Event: %s", stateMachineEventName(s_sEvent));

            s_stateMachineProcessEvent(&s_sEvent);
        }
    }
}


esp_err_t stateMachineInit(void) {
    esp_err_t lErr = ESP_OK;

    if(NULL != s_pQueue) {
        ESP_LOGW(TAG, "State machine already initialized");
        lErr = ESP_ERR_INVALID_STATE;
        goto end_sm_init;
    }

    s_pQueue = xQueueCreate(SM_QUEUE_LENGTH, sizeof(StateMachineEvent_t));

    if(NULL == s_pQueue) {
        ESP_LOGE(TAG, "Failed to create event queue");
        lErr = ESP_ERR_NO_MEM;
        goto end_sm_init;
    }

    ESP_LOGI(TAG, "Event queue initialized");

    BaseType_t ubResult = xTaskCreate(
        s_pTask,
        "SM",
        SM_TASK_STACK,
        NULL,
        SM_TASK_PRIORITY,
        NULL
    );

    if(pdPASS != ubResult) {
        ESP_LOGE(TAG, "Failed to create state machine task");
        vQueueDelete(s_pQueue);
        s_pQueue = NULL;

        lErr = ESP_ERR_NO_MEM;
        goto end_sm_init;
    }

end_sm_init:
    return lErr;
}


esp_err_t stateMachinePostEvent(StateMachineEventId_t eEventId) {
    esp_err_t lErr = ESP_OK;
    
    if(NULL == s_pQueue) {
        lErr = ESP_ERR_INVALID_STATE;
        goto end_post_event;
    }

    StateMachineEvent_t sEvent = {
        .eId = eEventId,
        .ulData = 0
    };

    if(pdTRUE != xQueueSend(s_pQueue, &sEvent, pdMS_TO_TICKS(20))) {
        lErr = ESP_ERR_TIMEOUT;
    }

end_post_event:
    return lErr;
}