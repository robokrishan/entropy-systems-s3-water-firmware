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
static StateMachineStateId_t s_eFailureState = STATE_MACHINE_STATE_MAX;
static StateMachineState_t* s_pStates[STATE_MACHINE_STATE_MAX] = {0};
static TaskHandle_t s_pTaskHandle = NULL;


/* Static Functions */
static StateMachineState_t* s_getState(StateMachineStateId_t eState) {
    if((uint32_t)eState < (uint32_t)STATE_MACHINE_STATE_MAX) {
        return s_pStates[eState];
    }

    return NULL;
}


static esp_err_t s_enterFailureState(void) {

    esp_err_t lErr = ESP_OK;

    if(STATE_MACHINE_STATE_MAX == s_eFailureState) {
        ESP_LOGE(TAG, "No failure state configured");
        lErr = ESP_ERR_INVALID_STATE;

        goto end_enter_fail_state;
    }

    StateMachineState_t* pFailureState = s_getState(s_eFailureState);

    if(NULL == pFailureState) {
        ESP_LOGE(TAG, "Failure state not registered!");
        lErr = ESP_ERR_INVALID_STATE;

        goto end_enter_fail_state;
    }

    if(s_eCurrentState == s_eFailureState) {
        goto end_enter_fail_state;
    }

    StateMachineStateId_t eOldState = s_eCurrentState;
    
    s_ePrevState = s_eCurrentState;
    s_eCurrentState = s_eFailureState;

    ESP_LOGE(TAG, "Forced transition: %s -> %s", 
        stateMachineStateName(eOldState),
        stateMachineStateName(s_eCurrentState)
    );

    if(NULL != pFailureState->cbInit) {
        lErr = pFailureState->cbInit();

        if(lErr) {
            ESP_LOGE(TAG, "Failed to init failure state. Code: 0x%X", lErr);
        }
    }

end_enter_fail_state:

    return lErr;
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
            ESP_LOGE(TAG, "Failed to deinit state %s. Code: 0x%X", 
                stateMachineStateName(s_eCurrentState), 
                lErr
            );

            esp_err_t lFailErr = s_enterFailureState();

            if(lFailErr) {
                ESP_LOGE(TAG, "Failed to enter failure state. Code: 0x%X", lFailErr);
            }

            goto end_transition;
        }
    }


    // Change state
    s_ePrevState = s_eCurrentState;
    s_eCurrentState = eNewState;

    ESP_LOGI(TAG, "%s -> %s", stateMachineStateName(s_ePrevState), 
        stateMachineStateName(s_eCurrentState)
    );


    // Enter new state

    if(NULL != pNextState->cbInit) {

        lErr = pNextState->cbInit();
        if(lErr) {
            ESP_LOGE(TAG, "Failed to init state %s. Code: 0x%X", 
                stateMachineStateName(eNewState), 
                lErr
            );
            
            /*
                Try to cleanup state that we failed to enter
            */
            if(NULL != pNextState->cbDeinit) {
                esp_err_t lCleanupErr = pNextState->cbDeinit();

                if(lCleanupErr) {
                    ESP_LOGE(TAG, "Failed to clean up state %s. Code: 0x%X",
                        stateMachineStateName(eNewState),
                        lCleanupErr
                    );
                }
            }

            /*
            * If the failure state's own initialization failed,
            * do not recursively attempt to enter it again.
            */
            if(eNewState != s_eFailureState) {
                esp_err_t lFailureErr = s_enterFailureState();

                if(lFailureErr) {
                    ESP_LOGE(TAG, "Failed to enter failure state. Code: 0x%X", lFailureErr);
                }
            }

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

    // check if failure state is configured
    if(STATE_MACHINE_STATE_MAX == s_eFailureState) {
        ESP_LOGE(TAG, "No failure state configured");
        lErr = ESP_ERR_INVALID_ARG;

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

            esp_err_t lFailError = s_enterFailureState();
            if(lFailError) {
                ESP_LOGE(TAG, "Failed to enter failure state. Code: 0x%X", lFailError);
            }

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
        &s_pTaskHandle
    );

    if(pdPASS != ubResult) {
        ESP_LOGE(TAG, "Failed to create state machine task");

        s_pTaskHandle = NULL;
        lErr = ESP_ERR_NO_MEM;

        goto cleanup_state;
    }

    ESP_LOGI(TAG, "State machine initialized in state %s", 
        stateMachineStateName(s_eCurrentState)
    );

    goto end_sm_init;

cleanup_state:

    if(NULL != pInitState->cbDeinit) {
        esp_err_t lCleanupErr = pInitState->cbDeinit();

        if(lCleanupErr) {
            ESP_LOGE(
                TAG,
                "Failed to deinit initial state during cleanup. Code: 0x%X",
                lCleanupErr
            );
        }
    }

cleanup_queue:

    if(NULL != s_pQueue) {
        vQueueDelete(s_pQueue);
        s_pQueue = NULL;
    }

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

StateMachineStateId_t stateMachineGetCurrentState(void)
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


/* -------------------------------------------------------------------------- */
/* Fail State Setter                                                          */
/* -------------------------------------------------------------------------- */

esp_err_t stateMachineSetFailureState(StateMachineStateId_t eState) {
    
    esp_err_t lErr = ESP_OK;

    if(NULL != s_pQueue) {
        ESP_LOGE(TAG, "Cannot set failure state after SM initialization");
        lErr = ESP_ERR_INVALID_STATE;

        goto end_fail_state_reg;
    }

    if(NULL == s_getState(eState)) {
        ESP_LOGE(TAG, "Failure state %s not registered!", stateMachineStateName(eState));
        lErr = ESP_ERR_INVALID_ARG;

        goto end_fail_state_reg;
    }

    if(STATE_MACHINE_STATE_MAX != s_eFailureState) {
        ESP_LOGE(TAG, "Failure state already configured");
        lErr = ESP_ERR_INVALID_STATE;

        goto end_fail_state_reg;
    }

    s_eFailureState = eState;

    ESP_LOGI(TAG, "Failure state set to %s", stateMachineStateName(s_eFailureState));

end_fail_state_reg:

    return lErr;
}


/* -------------------------------------------------------------------------- */
/* Deinitialization                                                           */
/* -------------------------------------------------------------------------- */

void stateMachineDeinit(void) {
    esp_err_t lErr = ESP_OK;

    /*
     * Record whether the state machine was actually running.
     *
     * Registered states may exist even if stateMachineInit() was never
     * successfully completed, so we should only invoke the current state's
     * deinit callback if the state machine task was successfully created.
     */
    bool bWasRunning = (NULL != s_pTaskHandle);


    /*
     * Stop state-machine processing first so that no further events can
     * modify state while cleanup is in progress.
     */
    if(NULL != s_pTaskHandle) {
        vTaskDelete(s_pTaskHandle);
        s_pTaskHandle = NULL;
    }


    /*
     * Deinitialize the currently active state.
     *
     * Do not stop cleanup if the callback fails.
     */
    if(bWasRunning) {
        StateMachineState_t* pCurrentState = s_getState(s_eCurrentState);

        if((NULL != pCurrentState) && (NULL != pCurrentState->cbDeinit)) {
            lErr = pCurrentState->cbDeinit();

            if(lErr) {
                ESP_LOGE(
                    TAG,
                    "Failed to deinit state %s during state machine cleanup. Code: 0x%X",
                    stateMachineStateName(s_eCurrentState),
                    lErr
                );
            }
        }
    }


    /*
     * Delete the event queue.
     */
    if(NULL != s_pQueue) {
        vQueueDelete(s_pQueue);
        s_pQueue = NULL;
    }


    /*
     * Remove all registered states so that the state table can be
     * populated again during a subsequent initialization.
     */
    for(uint32_t ulIndex = 0;
        ulIndex < (uint32_t)STATE_MACHINE_STATE_MAX;
        ulIndex++) {

        s_pStates[ulIndex] = NULL;
    }


    /*
     * Reset state-machine bookkeeping.
     */
    s_eCurrentState = STATE_MACHINE_STATE_MAX;
    s_ePrevState = STATE_MACHINE_STATE_MAX;
    s_eFailureState = STATE_MACHINE_STATE_MAX;

    ESP_LOGI(TAG, "State machine deinitialized");
}

