#include "state_machine.h"
#include "state_machine_common.h"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


#define SM_QUEUE_LENGTH     10
#define SM_TASK_STACK       4096
#define SM_TASK_PRIORITY    5


static const char *TAG = "SM";


/* Static Variables */
static QueueHandle_t s_pQueue = NULL;
static StateMachineStateId_t s_eCurrentState = STATE_MACHINE_STATE_MAX;
static StateMachineStateId_t s_ePrevState = STATE_MACHINE_STATE_MAX;
static StateMachineState_t* s_pStates[STATE_MACHINE_STATE_MAX] = {0};


/* Static Functions */
static StateMachineState_t* s_getState(StateMachineStateId_t eState) {
    if((uint32_t)eState < (uint32_t)STATE_MACHINE_STATE_MAX) {
        return s_pStates[eState];
    }

    return NULL;
}


static esp_err_t s_transitionTo(StateMachineStateId_t eNewState) {
    esp_err_t lErr = ESP_OK;

    StateMachineState_t* pCurrentState = s_getState(s_eCurrentState);
    StateMachineState_t* pNextState = s_getState(eNewState);

    if(NULL == pNextState) {
        ESP_LOGE(TAG, "State %s not registered", stateMachineStateName(eNewState));
        lErr = ESP_ERR_INVALID_STATE;
        goto end_transition;
    }

    if(eNewState == s_eCurrentState) {
        goto end_transition;
    }


    // Leave current state

    if((NULL != pCurrentState) && (NULL != pCurrentState->cbDeinit)) {

        lErr = pCurrentState->cbDeinit();
        if(lErr) {
            ESP_LOGE(TAG, "Failed to deinit state %s. Code: 0x%X", stateMachineStateName(s_eCurrentState), lErr);

            goto end_transition;
        }
    }


    // Change state
    s_ePrevState = s_eCurrentState;
    s_eCurrentState = eNewState;

    ESP_LOGI(TAG, "%s -> %s", stateMachineStateName(s_ePrevState), stateMachineStateName(s_eCurrentState));


    // Enter new state

    if(NULL != pNextState->cbInit) {

        lErr = pNextState->cbInit();
        if(lErr) {
            ESP_LOGE(TAG, "Failed to init state %s. Code: 0x%X", stateMachineStateName(eNewState), lErr);

            // TODO: transition to fault/error state on state callback failure

            goto end_transition;
        }
    }
    

end_transition:

    return lErr;
}


/* -------------------------------------------------------------------------- */
/* Event processing                                                           */
/* -------------------------------------------------------------------------- */

static void s_processEvent(StateMachineEvent_t *pEvent) {

    // check null argument
    if(NULL == pEvent) {
        ESP_LOGE(TAG, "Invalid event pointer");
        return;
    }

    StateMachineState_t* pState = s_getState(s_eCurrentState);

    // check if current state and its callbacks are valid
    if(NULL == pState) {
        ESP_LOGE(TAG, "Current state %s not registered", stateMachineStateName(s_eCurrentState));
        return;
    }

    if(NULL == pState->cbProcess) {
        ESP_LOGE(TAG, "State %s has no process callback", stateMachineStateName(s_eCurrentState));
        return;
    }

    if(NULL == pState->cbNextState) {
        ESP_LOGE(TAG, "%s has no next-state callback", stateMachineStateName(s_eCurrentState));
        return;
    }

    // Process the event
    pState->cbProcess(pEvent);


    // Determine the next state
    StateMachineStateId_t eNextState = pState->cbNextState(pEvent);


    // Transition to the next state if it's different from the current state
    if(eNextState != s_eCurrentState) {
        StateMachineStateId_t eOldState = s_eCurrentState;

        esp_err_t lErr = s_transitionTo(eNextState);
        if(lErr) {
            ESP_LOGE(TAG, "Failed to transition from %s to %s. Code: 0x%X",
                stateMachineStateName(eOldState),
                stateMachineStateName(eNextState),
                lErr
            );
        }
    }
}


/* -------------------------------------------------------------------------- */
/* State machine task                                                         */
/* -------------------------------------------------------------------------- */

static void s_pTask(void *pArg) {
    (void)pArg;

    StateMachineEvent_t sEvent;

    while(true) {
        if(pdTRUE == xQueueReceive(s_pQueue, &sEvent, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Event:\t%s", stateMachineEventName(sEvent));
            s_processEvent(&sEvent);
        }
    }
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t stateMachineInit(void) {
    esp_err_t lErr = ESP_OK;

    // check if event queue already exists
    if (NULL != s_pQueue) {
        ESP_LOGW(TAG, "State machine already initialized");
        lErr = ESP_ERR_INVALID_STATE;

        goto end_sm_init;
    }

    // get initial state
    StateMachineState_t* pInitState = s_getState(s_eCurrentState);


    // check if initial state is registered
    if(NULL == pInitState) {
        ESP_LOGE(TAG, "No initial state registered");
        lErr = ESP_ERR_INVALID_STATE;
        goto end_sm_init;
    }


    // create event queue
    s_pQueue = xQueueCreate(SM_QUEUE_LENGTH, sizeof(StateMachineEvent_t));

    if (NULL == s_pQueue) {
        ESP_LOGE(TAG, "Failed to create state machine event queue");
        lErr = ESP_ERR_NO_MEM;

        goto end_sm_init;
    }

    ESP_LOGI(TAG, "Event queue initialized");


    // enter initial state
    if(NULL != pInitState->cbInit) {
        lErr = pInitState->cbInit();
        if(lErr) {
            ESP_LOGE(TAG, "Failed to initialize init state %s. Code: 0x%X",
                stateMachineStateName(s_eCurrentState), 
                lErr
            );

            goto cleanup_queue;
        }
    }


    // create state machine task
    BaseType_t ubResult = xTaskCreate(
        s_pTask,
        "SM",
        SM_TASK_STACK,
        NULL,
        SM_TASK_PRIORITY,
        NULL
    );

    if (pdPASS != ubResult) {
        ESP_LOGE(TAG, "Failed to create state machine task");

        lErr = ESP_ERR_NO_MEM;

        goto cleanup_state;
    }

    ESP_LOGI(TAG, "State machine initialized in state %s", 
        stateMachineStateName(s_eCurrentState)
    );

    goto end_sm_init;

cleanup_state:

    if(NULL != pInitState->cbDeinit) {
        pInitState->cbDeinit();
    }

cleanup_queue:

    vQueueDelete(s_pQueue);
    s_pQueue = NULL;

end_sm_init:

    return lErr;
}


/* -------------------------------------------------------------------------- */
/* State registration                                                         */
/* -------------------------------------------------------------------------- */

esp_err_t stateMachineRegisterState(StateMachineState_t* pState, bool isInitialState) {
    esp_err_t lErr = ESP_OK;

    if(NULL != s_pQueue) {
        ESP_LOGE(TAG, "Cannot register state after state machine initialization!");

        lErr = ESP_ERR_INVALID_STATE;

        goto end_register_state;
    }


    if((NULL == pState) || ((uint32_t)pState->eState >= (uint32_t)STATE_MACHINE_STATE_MAX)) {
        lErr = ESP_ERR_INVALID_ARG;

        goto end_register_state;
    }


    if((NULL == pState->cbProcess) || (NULL == pState->cbNextState)) {
        ESP_LOGE(TAG, "%s has invalid callbacks", stateMachineStateName(pState->eState));

        lErr = ESP_ERR_INVALID_ARG;

        goto end_register_state;
    }

    if(NULL != s_pStates[pState->eState]) {
        ESP_LOGE(TAG, "%s already registered", stateMachineStateName(pState->eState));

        lErr = ESP_ERR_INVALID_STATE;

        goto end_register_state;
    }

    // Already registered initial state
    if(isInitialState && (STATE_MACHINE_STATE_MAX != s_eCurrentState)) {
        ESP_LOGE(TAG, "Initial state already registered!");

        lErr = ESP_ERR_INVALID_STATE;

        goto end_register_state;
    }

    // register the state
    s_pStates[pState->eState] = pState;

    // register as initial state if indicated
    if(isInitialState) {
        s_eCurrentState = pState->eState;
    }

    ESP_LOGI(TAG, "Registered %s%s", stateMachineStateName(pState->eState), isInitialState ? " [init] " : "");

end_register_state:

    return lErr;
}


/* -------------------------------------------------------------------------- */
/* Current state                                                              */
/* -------------------------------------------------------------------------- */

StateMachineStateId_t stateMachineCurrentState(void)
{
    return s_eCurrentState;
}


/* -------------------------------------------------------------------------- */
/* Event posting                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t stateMachinePostEvent(StateMachineEventId_t eEventId) {
    esp_err_t lErr = ESP_OK;

    if (NULL == s_pQueue) {
        lErr = ESP_ERR_INVALID_STATE;

        goto end_post_event;
    }

    StateMachineEvent_t sEvent = {
        .eId = eEventId,
        .ulData = 0
    };

    if (pdTRUE != xQueueSend(s_pQueue, &sEvent, pdMS_TO_TICKS(20))) {
        ESP_LOGW(TAG, "Failed to post event %s: queue full",
            stateMachineEventName(sEvent)
        );

        lErr = ESP_ERR_TIMEOUT;
    }

end_post_event:

    return lErr;
}

