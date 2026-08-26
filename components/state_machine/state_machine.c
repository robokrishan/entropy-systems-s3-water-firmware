#include "state_machine.h"
#include "state_machine_common.h"
#include "state_machine_state_init.h"
#include "state_machine_state_stowed.h"
#include "state_machine_state_lowering.h"
#include "state_machine_state_deployed.h"
#include "state_machine_state_pumping.h"
#include "state_machine_state_raising.h"
#include "state_machine_state_pos_unknown.h"
#include "state_machine_state_fault.h"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


#define SM_QUEUE_LENGTH     10
#define SM_TASK_STACK       4096
#define SM_TASK_PRIORITY    5


static const char *TAG = "SM";


/* -------------------------------------------------------------------------- */
/* Types                                                                      */
/* -------------------------------------------------------------------------- */

typedef StateMachineState_t (*StateHandler_t)(
    const StateMachineEvent_t *pEvent
);


/* -------------------------------------------------------------------------- */
/* Static variables                                                           */
/* -------------------------------------------------------------------------- */

static QueueHandle_t s_pQueue = NULL;

static StateMachineState_t s_eCurrentState = STATE_MACHINE_INIT;


/*
 * Maps each state to the function responsible for processing events
 * while the state machine is in that state.
 */
static const StateHandler_t s_pStateHandlers[] = {
    [STATE_MACHINE_INIT] = stateInitProcessEvent,
    [STATE_MACHINE_STOWED] = stateStowedProcessEvent,
    [STATE_MACHINE_LOWERING] = stateLoweringProcessEvent,
    [STATE_MACHINE_DEPLOYED] = stateDeployedProcessEvent,
    [STATE_MACHINE_PUMPING] = statePumpingProcessEvent,
    [STATE_MACHINE_RAISING] = stateRaisingProcessEvent,
    [STATE_MACHINE_POSITION_UNKNOWN] = statePositionUnknownProcessEvent,
    [STATE_MACHINE_FAULT] = stateFaultProcessEvent
};


/* -------------------------------------------------------------------------- */
/* State transition                                                           */
/* -------------------------------------------------------------------------- */

static void s_stateMachineTransitionTo(StateMachineState_t eNewState)
{
    if (eNewState == s_eCurrentState) {
        return;
    }

    ESP_LOGI(
        TAG,
        "State transition:\t%s -> %s",
        StateMachineStateName(s_eCurrentState),
        StateMachineStateName(eNewState)
    );

    s_eCurrentState = eNewState;


    /*
     * State entry actions will eventually happen here.
     *
     * For now, the state machine only changes its logical state.
     *
     * Later examples:
     *
     * STATE_MACHINE_STOWED:
     *      spoolStop();
     *      pumpOff();
     *
     * STATE_MACHINE_LOWERING:
     *      spoolExtend();
     *
     * STATE_MACHINE_DEPLOYED:
     *      spoolStop();
     *      pumpOff();
     *
     * STATE_MACHINE_PUMPING:
     *      pumpOn();
     *
     * STATE_MACHINE_RAISING:
     *      spoolRetract();
     *
     * STATE_MACHINE_POSITION_UNKNOWN:
     *      spoolStop();
     *      pumpOff();
     *
     * STATE_MACHINE_FAULT:
     *      spoolStop();
     *      pumpOff();
     */
}


/* -------------------------------------------------------------------------- */
/* Event processing                                                           */
/* -------------------------------------------------------------------------- */

static void s_stateMachineProcessEvent(
    const StateMachineEvent_t *pEvent
)
{
    /*
     * A fault can occur from any state.
     *
     * This is deliberately handled before dispatching the event
     * to an individual state handler.
     */
    if (SM_EVENT_FAULT == pEvent->eId) {
        s_stateMachineTransitionTo(STATE_MACHINE_FAULT);
        return;
    }


    /*
     * Make sure the current state has a valid handler.
     */
    if (s_eCurrentState >= (sizeof(s_pStateHandlers) / sizeof(s_pStateHandlers[0]))) {
        ESP_LOGE(TAG, "Invalid state index: %d", s_eCurrentState);
        s_stateMachineTransitionTo(STATE_MACHINE_FAULT);
        return;
    }


    StateHandler_t pHandler = s_pStateHandlers[s_eCurrentState];


    if (NULL == pHandler) {

        ESP_LOGE(TAG, "No handler registered for state %s", 
            StateMachineStateName(s_eCurrentState)
        );

        s_stateMachineTransitionTo(STATE_MACHINE_FAULT);

        return;
    }


    /*
     * Ask the current state's handler what the next state should be.
     *
     * The handler itself does not modify s_eCurrentState.
     */
    StateMachineState_t eNextState = pHandler(pEvent);


    /*
     * If the handler requested a different state, perform the
     * transition here.
     */
    if (eNextState != s_eCurrentState) {
        s_stateMachineTransitionTo(eNextState);
    }
}


/* -------------------------------------------------------------------------- */
/* State machine task                                                         */
/* -------------------------------------------------------------------------- */

static void s_pTask(void *arg) {
    StateMachineEvent_t sEvent;

    while (true) {
        if (pdTRUE == xQueueReceive(s_pQueue, &sEvent, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Event: %s", stateMachineEventName(sEvent));

            s_stateMachineProcessEvent(&sEvent);
        }
    }
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t stateMachineInit(void) {
    esp_err_t lErr = ESP_OK;

    if (NULL != s_pQueue) {
        ESP_LOGW(TAG, "State machine already initialized");
        lErr = ESP_ERR_INVALID_STATE;

        goto end_sm_init;
    }

    s_pQueue = xQueueCreate(SM_QUEUE_LENGTH, sizeof(StateMachineEvent_t));

    if (NULL == s_pQueue) {
        ESP_LOGE(TAG, "Failed to create state machine event queue");
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

    if (pdPASS != ubResult) {
        ESP_LOGE(TAG, "Failed to create state machine task");
        vQueueDelete(s_pQueue);
        s_pQueue = NULL;
        lErr = ESP_ERR_NO_MEM;

        goto end_sm_init;
    }

    ESP_LOGI(TAG, "State machine initialized in state %s", 
        StateMachineStateName(s_eCurrentState)
    );

end_sm_init:

    return lErr;
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