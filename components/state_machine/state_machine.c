#include <stdio.h>
#include "state_machine.h"
#include "state_machine_state.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

#define SM_QUEUE_LENGTH     10
#define SM_TASK_STACK       4096
#define SM_TASK_PRIORITY    5

static const char* TAG = "SM";

static QueueHandle_t s_pQueue = NULL;

static StateMachineState_t s_eCurrentState = STATE_MACHINE_INIT;



static void s_pTask(void* arg) {
    StateMachineEvent_t s_sEvent;

    while(true) {
        if(pdTRUE == xQueueReceive(s_pQueue, &s_sEvent, portMAX_DELAY)){
            ESP_LOGW(TAG, "Event: %s", stateMachineEventName(s_sEvent));

            s_stateMachineProcessEvent(&s_sEvent);
        }
    }
}


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

    if(SM_EVENT_HALT == pEvent->id || SM_EVENT_FAULT == pEvent->id) {
        stateMachineTransitionTo(STATE_MACHINE_IDLE);
        return;
    }

    switch (s_eCurrentState) {
        case STATE_MACHINE_INIT:
            break;

        case STATE_MACHINE_IDLE:
            switch (pEvent->id) {
                case SM_EVENT_PUMP_ON:
                    stateMachineTransitionTo(STATE_MACHINE_PUMPING);
                    break;

                case SM_EVENT_LOWER_NOZZLE:
                    stateMachineTransitionTo(STATE_MACHINE_LOWERING);
                    break;

                case SM_EVENT_RAISE_NOZZLE:
                    stateMachineTransitionTo(STATE_MACHINE_RAISING);
                    break;

                default:
                    break;
            }

            break;

        case STATE_MACHINE_LOWERING:
            break;

        case STATE_MACHINE_PUMPING:
            switch (pEvent->id) {
                case SM_EVENT_PUMP_OFF:
                case SM_EVENT_RC_SIGNAL_LOST:
                    stateMachineTransitionTo(STATE_MACHINE_IDLE);
                    break;

                default:
                    ESP_LOGE(TAG, "Not allowed. Pump is active");
                    break;
            }
            break;

        case STATE_MACHINE_RAISING:
            break;

        case STATE_MACHINE_FAULT:
            break;

        default:
            break;
    }

}


esp_err_t stateMachineInit(void) {
    esp_err_t lErr = ESP_OK;
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
        .id = eEventId,
        .data = 0
    };

    if(pdTRUE != xQueueSend(s_pQueue, &sEvent, pdMS_TO_TICKS(20))) {
        lErr = ESP_ERR_TIMEOUT;
    }

end_post_event:
    return lErr;
}