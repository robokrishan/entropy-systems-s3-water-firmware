#include "tests.h"
#include "esp_err.h"
#include "esp_log.h"
#include "state_machine_common.h"
#include "state_machine.h"
#include "nozzle_servo.h"
#include "pump.h"
#include "limit_switch.h"
#include "motion_timeout.h"
#include "ina226.h"
#include "i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "TEST";


/* State Machine Test */
static bool s_checkState(StateMachineStateId_t eExpectedState) {
    StateMachineStateId_t eActualState = stateMachineGetCurrentState();

    if(eActualState != eExpectedState) {
        ESP_LOGE(TAG, "State check failed. Expected %s. Got %s",
            stateMachineStateName(eExpectedState),
            stateMachineStateName(eActualState)
        );
        return 0;

    } else {
        ESP_LOGI(TAG, "State check passed");
        return 1;
    }
}


static void s_postTestEvent(StateMachineEventId_t eEvent, uint32_t ulDelayMs) {
    StateMachineEvent_t temp = {
        .eId = eEvent,
        .ulData = 0
    };

    ESP_LOGI(TAG, "Posting test event: %s", stateMachineEventName(temp));

    esp_err_t lErr = ESP_OK;

    lErr = stateMachinePostEvent(eEvent);
    if(ESP_OK != lErr) {
        ESP_LOGE(TAG, "Failed to post event: %s", esp_err_to_name(lErr));
    }

    vTaskDelay(pdMS_TO_TICKS(ulDelayMs));
}


static uint8_t s_waitForState(StateMachineStateId_t eExpectedState) {
    StateMachineStateId_t eCurrentState = stateMachineGetCurrentState();
    StateMachineStateId_t ePreviousState = eCurrentState;

    ESP_LOGI(
        TAG,
        "Waiting for state: %s",
        stateMachineStateName(eExpectedState)
    );

    while(eCurrentState != eExpectedState) {

        /*
         * Don't hang forever if the mechanism enters FAULT
         * before reaching the expected state.
         */
        if(STATE_MACHINE_FAULT == eCurrentState) {
            ESP_LOGE(
                TAG,
                "Entered FAULT while waiting for %s",
                stateMachineStateName(eExpectedState)
            );

            return 0;
        }

        /*
         * Only print when the FSM actually changes state.
         */
        if(eCurrentState != ePreviousState) {
            ESP_LOGI(
                TAG,
                "Current state: %s",
                stateMachineStateName(eCurrentState)
            );

            ePreviousState = eCurrentState;
        }

        /*
         * This isn't an actuation delay. It just yields CPU time
         * while polling the FSM.
         */
        vTaskDelay(pdMS_TO_TICKS(50));

        eCurrentState = stateMachineGetCurrentState();
    }

    ESP_LOGI(
        TAG,
        "Reached state: %s",
        stateMachineStateName(eExpectedState)
    );

    return 1;
}


void testNormalSequence(void) {
    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 9;

    ubTestPassCount += s_checkState(STATE_MACHINE_INIT);

    s_postTestEvent(SM_EVENT_SYSTEM_READY, 3000);
    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);

    s_postTestEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 3000);
    ubTestPassCount += s_checkState(STATE_MACHINE_STOWED);

    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 3000);
    ubTestPassCount += s_checkState(STATE_MACHINE_LOWERING);

    s_postTestEvent(SM_EVENT_LOWER_LIMIT_ACTIVE, 3000);
    ubTestPassCount += s_checkState(STATE_MACHINE_DEPLOYED);

    s_postTestEvent(SM_EVENT_PUMP_ON, 3000);
    ubTestPassCount += s_checkState(STATE_MACHINE_PUMPING);

    s_postTestEvent(SM_EVENT_PUMP_OFF, 3000);
    ubTestPassCount += s_checkState(STATE_MACHINE_DEPLOYED);

    s_postTestEvent(SM_EVENT_NOZZLE_RETRACT, 3000);
    ubTestPassCount += s_checkState(STATE_MACHINE_RAISING);

    s_postTestEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 3000);
    ubTestPassCount += s_checkState(STATE_MACHINE_STOWED);

    ESP_LOGW(TAG, "Passed %d / %d tests", ubTestPassCount, ubTestCount);
    ESP_LOGW(TAG, "=== END TEST ===");
}


void testWrongSequence(void)
{
    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 15;

    /*
     * Establish known starting position.
     *
     * INIT -> POSITION_UNKNOWN -> STOWED
     */
    ubTestPassCount += s_checkState(STATE_MACHINE_INIT);
    s_postTestEvent(SM_EVENT_SYSTEM_READY, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);

    s_postTestEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_STOWED);


    /*
     * STOWED + PUMP_ON
     *
     * Must be rejected.
     * Remain in STOWED.
     */
    s_postTestEvent(SM_EVENT_PUMP_ON, 1000);
    ubTestPassCount += s_checkState(STATE_MACHINE_STOWED);


    /*
     * STOWED -> LOWERING
     */
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 1000);
    ubTestPassCount += s_checkState(STATE_MACHINE_LOWERING);


    /*
     * LOWERING + PUMP_ON
     *
     * Must be rejected.
     * Remain in LOWERING.
     */
    s_postTestEvent(SM_EVENT_PUMP_ON, 1000);
    ubTestPassCount += s_checkState(STATE_MACHINE_LOWERING);


    /*
     * WRONG LIMIT TEST:
     *
     * We are LOWERING, so the upper limit should not
     * complete the movement.
     *
     * Must remain in LOWERING.
     */
    s_postTestEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 1000);
    ubTestPassCount += s_checkState(STATE_MACHINE_LOWERING);


    /*
     * Stop before reaching lower limit.
     *
     * LOWERING -> POSITION_UNKNOWN
     */
    s_postTestEvent(SM_EVENT_STOP_SPOOL, 1000);
    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);

    /*
     * POSITION_UNKNOWN -> RAISING
     */
    s_postTestEvent(SM_EVENT_NOZZLE_RETRACT, 1000);
    ubTestPassCount += s_checkState(STATE_MACHINE_RAISING);


    /*
     * WRONG LIMIT TEST:
     *
     * We are RAISING, so the lower limit should not
     * complete the movement.
     *
     * Must remain in RAISING.
     */
    s_postTestEvent(SM_EVENT_LOWER_LIMIT_ACTIVE, 1000);
    ubTestPassCount += s_checkState(STATE_MACHINE_RAISING);


    /*
     * Correct upper limit.
     *
     * RAISING -> STOWED
     */
    s_postTestEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 1000);
    ubTestPassCount += s_checkState(STATE_MACHINE_STOWED);


    /*
     * STOWED -> LOWERING
     */
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 1000);
    ubTestPassCount += s_checkState(STATE_MACHINE_LOWERING);


    /*
     * Global fault.
     *
     * LOWERING -> FAULT
     */
    s_postTestEvent(SM_EVENT_FAULT, 1000);
    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);


    /*
     * Must be rejected while in FAULT.
     */
    s_postTestEvent(SM_EVENT_PUMP_ON, 1000);
    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);


    /*
     * FAULT -> POSITION_UNKNOWN
     */
    s_postTestEvent(SM_EVENT_RESET, 1000);
    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);

    ESP_LOGW(TAG, "Passed %d / %d tests", ubTestPassCount, ubTestCount);
    ESP_LOGW(TAG, "=== END TEST ===");
}


void testFaultSequence(void) {
    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 7;

    /* Get into a known STOWED state first */
    s_postTestEvent(SM_EVENT_SYSTEM_READY, 2000);
    s_postTestEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 1000);

    /* Start moving */
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 5000);
    ubTestPassCount += s_checkState(STATE_MACHINE_LOWERING);

    /*
     * Servo should currently be extending.
     * FAULT should stop the servo and enter FAULT.
     */
    s_postTestEvent(SM_EVENT_FAULT, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);

    /*
     * Normal commands should have no effect while faulted.
     */
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);
    s_postTestEvent(SM_EVENT_PUMP_ON, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);

    /*
     * RESET should clear the fault, but physical
     * nozzle position is no longer known.
     */
    s_postTestEvent(SM_EVENT_RESET, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /* Verify normal operation can resume*/
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 5000);
    ubTestPassCount += s_checkState(STATE_MACHINE_LOWERING);

    s_postTestEvent(SM_EVENT_LOWER_LIMIT_ACTIVE, 2000);
    ubTestPassCount += s_checkState(STATE_MACHINE_DEPLOYED);

    ESP_LOGW(TAG, "Passed %d / %d tests", ubTestPassCount, ubTestCount);
    ESP_LOGW(TAG, "=== END TEST ===");
}


void testStateInitFailure(void) {
    /*
        Force state_machine_state_lowering.s_stateInit() to
        return ESP_FAIL, then run this test.
    */
    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 7;

    ESP_LOGW(TAG, "=== TEST: State init failure ===");

    /* INIT -> POSITION_UNKNOWN */
    s_postTestEvent(SM_EVENT_SYSTEM_READY, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);

    /* POSITION_UNKNOWN -> STOWED */
    s_postTestEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_STOWED);

    /*
     * STOWED -> LOWERING
     *
     * LOWERING.cbInit() is deliberately returning ESP_FAIL,
     * so the state machine should force itself into FAULT.
     */
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);

    /*
     * Verify FAULT remains latched.
     */
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);

    s_postTestEvent(SM_EVENT_PUMP_ON, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);

    /*
     * Verify recovery still works.
     */
    s_postTestEvent(SM_EVENT_RESET, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);

    s_postTestEvent(SM_EVENT_NOZZLE_RETRACT, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_RAISING);

    ESP_LOGW(TAG, "Passed %d / %d tests", ubTestPassCount, ubTestCount);
    ESP_LOGW(TAG, "=== END TEST ===");
}


void testStateDeinitFailure(void) {
    /*
        Force state_machine_state_lowering.s_stateDeinit() to
        return ESP_FAIL, then run this test.
    */
    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 7;
    ESP_LOGW(TAG, "=== TEST: State deinit failure ===");

    /* INIT -> POSITION_UNKNOWN */
    s_postTestEvent(SM_EVENT_SYSTEM_READY, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);

    /* POSITION_UNKNOWN -> STOWED */
    s_postTestEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_STOWED);

    /* STOWED -> LOWERING */
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 1000);
    ubTestPassCount += s_checkState(STATE_MACHINE_LOWERING);

    /*
     * LOWER_LIMIT_ACTIVE normally causes:
     *
     * LOWERING -> DEPLOYED
     *
     * But LOWERING.cbDeinit() has been deliberately
     * made to fail. Therefore the state machine
     * should instead enter FAULT.
     */
    s_postTestEvent(SM_EVENT_LOWER_LIMIT_ACTIVE, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);

    /*
     * FAULT must remain latched.
     */
    s_postTestEvent(SM_EVENT_NOZZLE_RETRACT, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);

    /*
     * RESET should recover to POSITION_UNKNOWN.
     */
    s_postTestEvent(SM_EVENT_RESET, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);

    s_postTestEvent(SM_EVENT_NOZZLE_RETRACT, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_RAISING);

    ESP_LOGW(TAG, "Passed %d / %d tests", ubTestPassCount, ubTestCount);
    ESP_LOGW(TAG, "=== END TEST ===");
}


void testHaltSequence(void) {
    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 19;

    ESP_LOGW(TAG, "=== TEST: HALT sequence ===");


    /*
     * INIT + HALT
     *
     * Nothing is active yet.
     * Must remain in INIT.
     */
    ubTestPassCount += s_checkState(STATE_MACHINE_INIT);

    s_postTestEvent(SM_EVENT_HALT, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_INIT);


    /*
     * INIT -> POSITION_UNKNOWN
     */
    s_postTestEvent(SM_EVENT_SYSTEM_READY, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * POSITION_UNKNOWN + HALT
     *
     * Already stationary with unknown position.
     * Must remain in POSITION_UNKNOWN.
     */
    s_postTestEvent(SM_EVENT_HALT, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * Resolve position:
     * POSITION_UNKNOWN -> STOWED
     */
    s_postTestEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_STOWED);


    /*
     * STOWED + HALT
     *
     * Already stationary.
     * Must remain in STOWED.
     */
    s_postTestEvent(SM_EVENT_HALT, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_STOWED);


    /*
     * STOWED -> LOWERING
     */
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 1000);
    ubTestPassCount += s_checkState(STATE_MACHINE_LOWERING);


    /*
     * LOWERING + HALT
     *
     * Motion is interrupted before reaching a known
     * endpoint, so position becomes unknown.
     *
     * Servo should physically stop.
     */
    s_postTestEvent(SM_EVENT_HALT, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * Resume lowering from unknown position.
     */
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 1000);
    ubTestPassCount += s_checkState(STATE_MACHINE_LOWERING);


    /*
     * Simulate reaching lower limit.
     *
     * LOWERING -> DEPLOYED
     */
    s_postTestEvent(SM_EVENT_LOWER_LIMIT_ACTIVE, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_DEPLOYED);


    /*
     * DEPLOYED + HALT
     *
     * Already stationary and physical position is known.
     * Must remain DEPLOYED.
     */
    s_postTestEvent(SM_EVENT_HALT, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_DEPLOYED);


    /*
     * DEPLOYED -> PUMPING
     */
    s_postTestEvent(SM_EVENT_PUMP_ON, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_PUMPING);


    /*
     * PUMPING + HALT
     *
     * Pump operation stops, but nozzle remains
     * physically deployed.
     *
     * PUMPING -> DEPLOYED
     */
    s_postTestEvent(SM_EVENT_HALT, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_DEPLOYED);


    /*
     * DEPLOYED -> RAISING
     */
    s_postTestEvent(SM_EVENT_NOZZLE_RETRACT, 1000);
    ubTestPassCount += s_checkState(STATE_MACHINE_RAISING);


    /*
     * RAISING + HALT
     *
     * Motion is interrupted before reaching a known
     * endpoint, so position becomes unknown.
     *
     * Servo should physically stop.
     */
    s_postTestEvent(SM_EVENT_HALT, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * Enter FAULT.
     */
    s_postTestEvent(SM_EVENT_FAULT, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);


    /*
     * FAULT + HALT
     *
     * HALT must never clear a fault.
     */
    s_postTestEvent(SM_EVENT_HALT, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);


    /*
     * RESET clears the fault but position remains unknown.
     */
    s_postTestEvent(SM_EVENT_RESET, 500);
    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * Verify normal operation can resume after HALT/fault handling.
     */
    s_postTestEvent(SM_EVENT_NOZZLE_RETRACT, 1000);
    ubTestPassCount += s_checkState(STATE_MACHINE_RAISING);


    if(ubTestPassCount == ubTestCount) {
        ESP_LOGI(
            TAG,
            "TEST PASSED: %d / %d checks passed",
            ubTestPassCount,
            ubTestCount
        );
    } else {
        ESP_LOGE(
            TAG,
            "TEST FAILED: %d / %d checks passed",
            ubTestPassCount,
            ubTestCount
        );
    }

    ESP_LOGW(TAG, "=== END TEST ===");
}

/* Nozzle Servo Test */
void testNozzleServoSequence(void) {
    esp_err_t lErr = ESP_OK;

    ESP_LOGI(TAG, "Starting nozzle servo test");

    nozzleServoDeinit();

    lErr = nozzleServoInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init nozzle servo! Code: 0x%X", lErr);
        return;
    }

    ESP_LOGW(TAG, "Initialized");

    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGW(TAG, "Extending");

    lErr = nozzleServoExtend();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to extend servo! Code: 0x%X", lErr);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGW(TAG, "Stopping");

    lErr = nozzleServoStop();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to stop servo! Code: 0x%X", lErr);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGW(TAG, "Retracting");

    lErr = nozzleServoRetract();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to retract servo! Code: 0x%X", lErr);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGW(TAG, "Stopping");

    lErr = nozzleServoStop();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to stop servo! Code: 0x%X", lErr);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5000));

    nozzleServoDeinit();

    ESP_LOGW(TAG, "De-initialized");

    ESP_LOGI(TAG, "Nozzle servo test complete");
}


void testNozzlServoNeutral(void) {
    esp_err_t lErr = ESP_OK;

    ESP_LOGI(TAG, "Starting nozzle servo test");

    nozzleServoDeinit();

    lErr = nozzleServoInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init nozzle servo! Code: 0x%X", lErr);
        return;
    }

    ESP_LOGW(TAG, "Initialized");

    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGW(TAG, "Stopping");

    lErr = nozzleServoStop();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to stop servo! Code: 0x%X", lErr);
        return;
    }

    ESP_LOGW(TAG, "Servo should not move or jitter!");

    vTaskDelay(pdMS_TO_TICKS(30000));

    nozzleServoDeinit();

    ESP_LOGW(TAG, "De-initialized");

    ESP_LOGI(TAG, "Nozzle servo test complete");
}


void testPumpSequence(void) {
    esp_err_t lErr = ESP_OK;

    lErr = pumpInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to initialize pump. Code: 0x%X", lErr);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(3000));

    lErr = pumpOn();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to power on pump. Code: 0x%X", lErr);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(5000));

    lErr = pumpOff();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to power off pump. Code: 0x%X", lErr);
        return;
    }

    ESP_LOGI(TAG, "Pump test success.");
}


void testLimitSwitchSequence(void) {
    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 11;

    ESP_LOGW(TAG, "=== TEST: LIMIT SWITCH SEQUENCE ===");
    ESP_LOGW(TAG, "Ensure BOTH limit switches are RELEASED");
    vTaskDelay(pdMS_TO_TICKS(3000));


    /*
     * Complete system startup.
     *
     * Both switches are released, so synchronization should
     * leave the mechanism position unknown.
     */
    s_postTestEvent(SM_EVENT_SYSTEM_READY, 500);

    esp_err_t lErr = limitSwitchSyncState();
    if(lErr) {
        ESP_LOGE(
            TAG,
            "Failed to synchronize limit switches. Code: 0x%X",
            lErr
        );
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * TEST UPPER LIMIT ACTIVE
     *
     * Physically press the upper limit switch.
     *
     * POSITION_UNKNOWN + UPPER_LIMIT_ACTIVE -> STOWED
     */
    ESP_LOGW(TAG, "PRESS UPPER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_STOWED);


    /*
     * TEST UPPER LIMIT RELEASED
     *
     * Release the upper switch.
     * STOWED should remain STOWED.
     *
     * Also verify the log reports:
     * "Upper limit released"
     */
    ESP_LOGW(TAG, "RELEASE UPPER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_STOWED);


    /*
     * Begin lowering.
     *
     * STOWED -> LOWERING
     */
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 500);

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_LOWERING);


    /*
     * TEST LOWER LIMIT ACTIVE
     *
     * Physically press lower limit.
     *
     * LOWERING + LOWER_LIMIT_ACTIVE -> DEPLOYED
     *
     * Entering DEPLOYED should also stop the servo.
     */
    ESP_LOGW(TAG, "PRESS LOWER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_DEPLOYED);


    /*
     * TEST LOWER LIMIT RELEASED
     *
     * Release lower switch.
     * DEPLOYED should remain DEPLOYED.
     *
     * Verify log reports:
     * "Lower limit released"
     */
    ESP_LOGW(TAG, "RELEASE LOWER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_DEPLOYED);


    /*
     * Begin raising.
     *
     * DEPLOYED -> RAISING
     */
    s_postTestEvent(SM_EVENT_NOZZLE_RETRACT, 500);

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_RAISING);


    /*
     * TEST UPPER LIMIT ACTIVE DURING RAISING
     *
     * RAISING + UPPER_LIMIT_ACTIVE -> STOWED
     *
     * Servo should physically stop.
     */
    ESP_LOGW(TAG, "PRESS UPPER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_STOWED);


    /*
     * TEST INVALID PHYSICAL CONDITION
     *
     * Keep UPPER pressed and now press LOWER as well.
     *
     * Both switches active simultaneously must generate FAULT.
     */
    ESP_LOGW(
        TAG,
        "KEEP UPPER PRESSED and PRESS LOWER limit switch now"
    );

    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_FAULT);


    /*
     * Release both switches.
     *
     * Release events should not clear FAULT.
     */
    ESP_LOGW(TAG, "RELEASE BOTH limit switches now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_FAULT);


    /*
     * Reset fault.
     *
     * Neither physical endpoint is currently asserted,
     * therefore position returns to unknown.
     */
    s_postTestEvent(SM_EVENT_RESET, 500);

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    if(ubTestPassCount == ubTestCount) {
        ESP_LOGI(
            TAG,
            "TEST PASSED: %d / %d checks passed",
            ubTestPassCount,
            ubTestCount
        );
    } else {
        ESP_LOGE(
            TAG,
            "TEST FAILED: %d / %d checks passed",
            ubTestPassCount,
            ubTestCount
        );
    }

    ESP_LOGW(TAG, "=== END LIMIT SWITCH TEST ===");
}


void testMotionTimeoutSequence(void) {
    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 7;

    esp_err_t lErr = ESP_OK;

    ESP_LOGW(TAG, "=== TEST: MOTION TIMEOUT SEQUENCE ===");


    /*
     * Test starts in POSITION_UNKNOWN.
     */
    ubTestPassCount +=
        s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * TEST 1:
     * Start timer and allow it to expire.
     *
     * Expected:
     * POSITION_UNKNOWN -> FAULT
     */
    ESP_LOGW(TAG, "TEST 1: Allow motion timeout to expire");

    lErr = motionTimeoutStart();
    if(lErr) {
        ESP_LOGE(
            TAG,
            "Failed to start motion timeout. Code: 0x%X",
            lErr
        );
        return;
    }

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_POSITION_UNKNOWN);

    /*
     * Timeout is 10 seconds.
     * Give some additional margin for scheduling/event processing.
     */
    vTaskDelay(pdMS_TO_TICKS(11000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_FAULT);


    /*
     * Recover from FAULT.
     */
    s_postTestEvent(SM_EVENT_RESET, 500);

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * TEST 2:
     * Start timer and cancel it before expiration.
     *
     * Expected:
     * No FAULT occurs.
     */
    ESP_LOGW(TAG, "TEST 2: Stop motion timeout before expiry");

    lErr = motionTimeoutStart();
    if(lErr) {
        ESP_LOGE(
            TAG,
            "Failed to start motion timeout. Code: 0x%X",
            lErr
        );
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(3000));

    lErr = motionTimeoutStop();
    if(lErr) {
        ESP_LOGE(
            TAG,
            "Failed to stop motion timeout. Code: 0x%X",
            lErr
        );
        return;
    }

    /*
     * Wait longer than a complete timeout period.
     * If stop worked, we must remain POSITION_UNKNOWN.
     */
    vTaskDelay(pdMS_TO_TICKS(11000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * TEST 3:
     * Verify that calling Start() again while the timer is already
     * running restarts the countdown.
     */
    ESP_LOGW(TAG, "TEST 3: Restart active motion timeout");

    lErr = motionTimeoutStart();
    if(lErr) {
        ESP_LOGE(
            TAG,
            "Failed to start motion timeout. Code: 0x%X",
            lErr
        );
        return;
    }

    /*
     * Let 3 seconds pass.
     */
    vTaskDelay(pdMS_TO_TICKS(3000));

    /*
     * Restart the timer.
     *
     * The timeout should now occur 10 seconds from HERE,
     * rather than 10 seconds from the original start.
     */
    lErr = motionTimeoutStart();
    if(lErr) {
        ESP_LOGE(
            TAG,
            "Failed to restart motion timeout. Code: 0x%X",
            lErr
        );
        return;
    }

    /*
     * Wait 8 seconds.
     *
     * 11 seconds have passed since the first Start(), but only
     * 8 seconds since the reset. Therefore we should NOT be
     * in FAULT yet.
     */
    vTaskDelay(pdMS_TO_TICKS(8000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * Wait another 3 seconds.
     *
     * We are now 11 seconds beyond the reset point, so the timer
     * should have expired and generated FAULT.
     */
    vTaskDelay(pdMS_TO_TICKS(3000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_FAULT);


    if(ubTestPassCount == ubTestCount) {
        ESP_LOGI(
            TAG,
            "TEST PASSED: %d / %d checks passed",
            ubTestPassCount,
            ubTestCount
        );
    } else {
        ESP_LOGE(
            TAG,
            "TEST FAILED: %d / %d checks passed",
            ubTestPassCount,
            ubTestCount
        );
    }

    ESP_LOGW(TAG, "=== END MOTION TIMEOUT TEST ===");
}


void testMotionTimeoutIntegrationSequence(void) {
    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 16;

    ESP_LOGW(TAG, "=== TEST: MOTION TIMEOUT INTEGRATION ===");
    ESP_LOGW(TAG, "Ensure BOTH limit switches are RELEASED");

    vTaskDelay(pdMS_TO_TICKS(3000));


    /*
     * Test should begin in POSITION_UNKNOWN.
     */
    ubTestPassCount +=
        s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /* ============================================================
     * TEST 1: LOWERING timeout
     * ============================================================ */

    ESP_LOGW(TAG, "TEST 1: LOWERING timeout");
    ESP_LOGW(TAG, "PRESS UPPER limit switch now");

    vTaskDelay(pdMS_TO_TICKS(5000));

    /*
     * POSITION_UNKNOWN + UPPER_LIMIT_ACTIVE -> STOWED
     */
    ubTestPassCount +=
        s_checkState(STATE_MACHINE_STOWED);


    ESP_LOGW(TAG, "RELEASE UPPER limit switch now");

    vTaskDelay(pdMS_TO_TICKS(3000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_STOWED);


    /*
     * STOWED -> LOWERING
     *
     * LOWERING.cbInit() should:
     *   - start servo extension
     *   - start motion timeout
     */
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 500);

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_LOWERING);


    /*
     * DO NOT press the lower limit.
     *
     * After ~10 seconds the timer should expire and generate FAULT.
     */
    ESP_LOGW(
        TAG,
        "DO NOT PRESS LOWER limit - waiting for timeout..."
    );

    vTaskDelay(pdMS_TO_TICKS(11000));

    /*
     * Expected:
     *
     * motion timeout expires
     *      ↓
     * SM_EVENT_FAULT
     *      ↓
     * LOWERING.cbDeinit()
     *      ├── servo stop
     *      └── timeout stop
     *      ↓
     * FAULT
     */
    ubTestPassCount +=
        s_checkState(STATE_MACHINE_FAULT);


    /*
     * Recover.
     */
    s_postTestEvent(SM_EVENT_RESET, 500);

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /* ============================================================
     * TEST 2: RAISING timeout
     * ============================================================ */

    ESP_LOGW(TAG, "TEST 2: RAISING timeout");
    ESP_LOGW(TAG, "PRESS LOWER limit switch now");

    vTaskDelay(pdMS_TO_TICKS(5000));

    /*
     * POSITION_UNKNOWN + LOWER_LIMIT_ACTIVE -> DEPLOYED
     */
    ubTestPassCount +=
        s_checkState(STATE_MACHINE_DEPLOYED);


    ESP_LOGW(TAG, "RELEASE LOWER limit switch now");

    vTaskDelay(pdMS_TO_TICKS(3000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_DEPLOYED);


    /*
     * DEPLOYED -> RAISING
     *
     * RAISING.cbInit() should:
     *   - start servo retraction
     *   - start motion timeout
     */
    s_postTestEvent(SM_EVENT_NOZZLE_RETRACT, 500);

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_RAISING);


    /*
     * DO NOT press upper limit.
     */
    ESP_LOGW(
        TAG,
        "DO NOT PRESS UPPER limit - waiting for timeout..."
    );

    vTaskDelay(pdMS_TO_TICKS(11000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_FAULT);


    /*
     * Recover again.
     */
    s_postTestEvent(SM_EVENT_RESET, 500);

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /* ============================================================
     * TEST 3: Successful motion cancels timeout
     * ============================================================ */

    ESP_LOGW(TAG, "TEST 3: Successful motion cancels timeout");
    ESP_LOGW(TAG, "PRESS UPPER limit switch now");

    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_STOWED);


    ESP_LOGW(TAG, "RELEASE UPPER limit switch now");

    vTaskDelay(pdMS_TO_TICKS(3000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_STOWED);


    /*
     * Begin normal lowering.
     */
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 500);

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_LOWERING);


    /*
     * This time, activate the lower limit BEFORE the
     * 10-second timeout expires.
     */
    ESP_LOGW(
        TAG,
        "PRESS LOWER limit switch within 5 seconds"
    );

    vTaskDelay(pdMS_TO_TICKS(5000));

    /*
     * LOWER_LIMIT_ACTIVE should cause:
     *
     * LOWERING.cbDeinit()
     *      ├── servo stop
     *      └── motionTimeoutStop()
     *
     * then:
     *
     * LOWERING -> DEPLOYED
     */
    ubTestPassCount +=
        s_checkState(STATE_MACHINE_DEPLOYED);


    /*
     * Now deliberately wait LONGER than the full timeout.
     *
     * If cbDeinit() successfully cancelled the timer,
     * no delayed FAULT should appear.
     */
    ESP_LOGW(
        TAG,
        "Waiting 11 seconds to verify timeout was cancelled..."
    );

    vTaskDelay(pdMS_TO_TICKS(11000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_DEPLOYED);


    /* ============================================================
     * RESULT
     * ============================================================ */

    if(ubTestPassCount == ubTestCount) {
        ESP_LOGI(
            TAG,
            "TEST PASSED: %d / %d checks passed",
            ubTestPassCount,
            ubTestCount
        );
    } else {
        ESP_LOGE(
            TAG,
            "TEST FAILED: %d / %d checks passed",
            ubTestPassCount,
            ubTestCount
        );
    }

    ESP_LOGW(TAG, "=== END MOTION TIMEOUT INTEGRATION TEST ===");
}


void testRcSignalLossSequence(void) {
    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 5;

    ESP_LOGW(TAG, "=== TEST: RC SIGNAL LOSS ===");
    ESP_LOGW(TAG, "Ensure RC PWM signal is CONNECTED");

    /*
     * Give the RC input some time to receive valid pulses.
     *
     * RC_SIGNAL_TIMEOUT_MS is currently 2000 ms, so waiting
     * 3 seconds proves that valid pulses continuously reset
     * the signal-loss timer.
     */
    vTaskDelay(pdMS_TO_TICKS(5000));

    /*
     * We should still be in POSITION_UNKNOWN.
     */
    ubTestPassCount +=
        s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * Put the FSM into DEPLOYED using a synthetic endpoint event.
     */
    s_postTestEvent(
        SM_EVENT_LOWER_LIMIT_ACTIVE,
        500
    );

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_DEPLOYED);


    /*
     * Enter PUMPING.
     *
     * This makes RC signal loss observable through an actual
     * FSM transition:
     *
     * PUMPING + RC_SIGNAL_LOST -> DEPLOYED
     */
    s_postTestEvent(
        SM_EVENT_PUMP_ON,
        500
    );

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_PUMPING);


    /*
     * Disconnect the PWM signal wire.
     *
     * Do NOT disconnect the ESP32 or common ground.
     */
    ESP_LOGW(TAG, "DISCONNECT RC PWM SIGNAL NOW");

    vTaskDelay(pdMS_TO_TICKS(8000));

    /*
     * Expected after ~2 seconds:
     *
     * rc_input:
     *     RC signal lost
     *
     * state machine:
     *     SM_EVENT_RC_SIGNAL_LOST
     *     PUMPING -> DEPLOYED
     *
     * Pump should physically turn OFF.
     */
    ubTestPassCount +=
        s_checkState(STATE_MACHINE_DEPLOYED);


    /*
     * Reconnect PWM.
     */
    ESP_LOGW(TAG, "RECONNECT RC PWM SIGNAL NOW");

    vTaskDelay(pdMS_TO_TICKS(8000));

    /*
     * Expected logs:
     *
     * RC signal restored
     * RC input LOW
     *
     * OR
     *
     * RC signal restored
     * RC input HIGH
     *
     * depending on the switch position.
     *
     * The FSM should remain DEPLOYED because HIGH/LOW is not yet
     * mapped to state-machine commands.
     */
    ubTestPassCount +=
        s_checkState(STATE_MACHINE_DEPLOYED);


    if(ubTestPassCount == ubTestCount) {
        ESP_LOGI(
            TAG,
            "TEST PASSED: %d / %d checks passed",
            ubTestPassCount,
            ubTestCount
        );
    } else {
        ESP_LOGE(
            TAG,
            "TEST FAILED: %d / %d checks passed",
            ubTestPassCount,
            ubTestCount
        );
    }

    ESP_LOGW(TAG, "=== END RC SIGNAL LOSS TEST ===");
}


void testRcSignalLossFsmSequence(void) {
    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 12;

    ESP_LOGW(TAG, "=== TEST: RC SIGNAL LOSS FSM ===");


    /*
     * Test starts in POSITION_UNKNOWN.
     */
    ubTestPassCount +=
        s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /* ============================================================
     * TEST 1: Signal loss while LOWERING
     * ============================================================ */

    /*
     * Establish known STOWED position.
     */
    s_postTestEvent(
        SM_EVENT_UPPER_LIMIT_ACTIVE,
        500
    );

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_STOWED);


    /*
     * Begin lowering.
     *
     * Servo should begin extending and motion timeout should start.
     */
    s_postTestEvent(
        SM_EVENT_NOZZLE_EXTEND,
        500
    );

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_LOWERING);


    /*
     * Simulate RC signal loss during movement.
     *
     * Expected:
     * LOWERING -> POSITION_UNKNOWN
     *
     * LOWERING.cbDeinit() should:
     *   - stop servo
     *   - stop motion timeout
     */
    s_postTestEvent(
        SM_EVENT_RC_SIGNAL_LOST,
        500
    );

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /* ============================================================
     * TEST 2: Signal loss while PUMPING
     * ============================================================ */

    /*
     * Establish known DEPLOYED position.
     */
    s_postTestEvent(
        SM_EVENT_LOWER_LIMIT_ACTIVE,
        500
    );

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_DEPLOYED);


    /*
     * Start pump.
     */
    s_postTestEvent(
        SM_EVENT_PUMP_ON,
        500
    );

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_PUMPING);


    /*
     * Signal loss while pumping.
     *
     * Expected:
     * PUMPING -> DEPLOYED
     *
     * PUMPING.cbDeinit() should physically turn the pump OFF.
     */
    s_postTestEvent(
        SM_EVENT_RC_SIGNAL_LOST,
        500
    );

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_DEPLOYED);


    /* ============================================================
     * TEST 3: Signal loss while RAISING
     * ============================================================ */

    /*
     * Begin raising.
     *
     * Servo should retract and motion timeout should start.
     */
    s_postTestEvent(
        SM_EVENT_NOZZLE_RETRACT,
        500
    );

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_RAISING);


    /*
     * Signal loss during movement.
     *
     * Expected:
     * RAISING -> POSITION_UNKNOWN
     *
     * Servo and motion timeout should both stop.
     */
    s_postTestEvent(
        SM_EVENT_RC_SIGNAL_LOST,
        500
    );

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /* ============================================================
     * TEST 4: Signal loss while FAULTED
     * ============================================================ */

    /*
     * Enter FAULT.
     */
    s_postTestEvent(
        SM_EVENT_FAULT,
        500
    );

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_FAULT);


    /*
     * RC signal loss must NEVER clear a fault.
     */
    s_postTestEvent(
        SM_EVENT_RC_SIGNAL_LOST,
        500
    );

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_FAULT);


    /*
     * Recover normally using RESET.
     */
    s_postTestEvent(
        SM_EVENT_RESET,
        500
    );

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /* ============================================================
     * RESULT
     * ============================================================ */

    if(ubTestPassCount == ubTestCount) {
        ESP_LOGI(
            TAG,
            "TEST PASSED: %d / %d checks passed",
            ubTestPassCount,
            ubTestCount
        );
    } else {
        ESP_LOGE(
            TAG,
            "TEST FAILED: %d / %d checks passed",
            ubTestPassCount,
            ubTestCount
        );
    }

    ESP_LOGW(TAG, "=== END RC SIGNAL LOSS FSM TEST ===");
}


void testRcPumpIntegrationSequence(void) {
    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 10;

    ESP_LOGW(TAG, "=== TEST: RC PUMP INTEGRATION ===");
    ESP_LOGW(TAG, "Ensure RC PWM is connected");
    ESP_LOGW(TAG, "Set pump RC switch to LOW / OFF");

    /*
     * Give the RC input time to establish its initial LOW state.
     */
    vTaskDelay(pdMS_TO_TICKS(3000));


    /*
     * Test should begin in POSITION_UNKNOWN.
     */
    ubTestPassCount +=
        s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /* ============================================================
     * TEST 1: PUMP_ON must be rejected while STOWED
     * ============================================================ */

    /*
     * Establish STOWED position.
     */
    s_postTestEvent(
        SM_EVENT_UPPER_LIMIT_ACTIVE,
        500
    );

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_STOWED);


    /*
     * Physically move RC switch HIGH.
     *
     * rc_input should generate:
     *
     * RC_INPUT_STATE_HIGH
     *      ↓
     * SM_EVENT_PUMP_ON
     *
     * But the FSM must reject PUMP_ON while STOWED.
     */
    ESP_LOGW(TAG, "MOVE PUMP RC SWITCH HIGH / ON");

    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_STOWED);

    /*
     * Pump must physically remain OFF.
     */


    /*
     * Return the RC switch LOW.
     *
     * This should generate SM_EVENT_PUMP_OFF, which is harmless
     * while STOWED.
     */
    ESP_LOGW(TAG, "MOVE PUMP RC SWITCH LOW / OFF");

    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_STOWED);


    /* ============================================================
     * TEST 2: PUMP_ON while DEPLOYED
     * ============================================================ */

    /*
     * Begin normal lowering.
     */
    s_postTestEvent(
        SM_EVENT_NOZZLE_EXTEND,
        500
    );

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_LOWERING);


    /*
     * Simulate reaching the lower endpoint.
     */
    s_postTestEvent(
        SM_EVENT_LOWER_LIMIT_ACTIVE,
        500
    );

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_DEPLOYED);


    /*
     * Now physically switch HIGH.
     *
     * Expected:
     *
     * RC HIGH
     *     ↓
     * SM_EVENT_PUMP_ON
     *     ↓
     * DEPLOYED -> PUMPING
     *     ↓
     * pumpOn()
     */
    ESP_LOGW(TAG, "MOVE PUMP RC SWITCH HIGH / ON");

    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_PUMPING);

    /*
     * Pump should now physically be running.
     */


    /*
     * Leave the switch HIGH briefly.
     *
     * Duplicate suppression should prevent repeated PUMP_ON events.
     * The FSM should simply remain PUMPING.
     */
    ESP_LOGW(TAG, "KEEP SWITCH HIGH");

    vTaskDelay(pdMS_TO_TICKS(3000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_PUMPING);


    /*
     * Move switch LOW.
     *
     * Expected:
     *
     * RC LOW
     *     ↓
     * SM_EVENT_PUMP_OFF
     *     ↓
     * PUMPING -> DEPLOYED
     *     ↓
     * pumpOff()
     */
    ESP_LOGW(TAG, "MOVE PUMP RC SWITCH LOW / OFF");

    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_DEPLOYED);

    /*
     * Pump should now physically be OFF.
     */


    /*
     * Leave LOW briefly to verify no duplicate commands
     * cause any unexpected state changes.
     */
    ESP_LOGW(TAG, "KEEP SWITCH LOW");

    vTaskDelay(pdMS_TO_TICKS(3000));

    ubTestPassCount +=
        s_checkState(STATE_MACHINE_DEPLOYED);


    /* ============================================================
     * RESULT
     * ============================================================ */

    if(ubTestPassCount == ubTestCount) {
        ESP_LOGI(
            TAG,
            "TEST PASSED: %d / %d checks passed",
            ubTestPassCount,
            ubTestCount
        );
    } else {
        ESP_LOGE(
            TAG,
            "TEST FAILED: %d / %d checks passed",
            ubTestPassCount,
            ubTestCount
        );
    }

    ESP_LOGW(TAG, "=== END RC PUMP INTEGRATION TEST ===");
}


void testRcNozzleIntegrationSequence(void) {
    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 5;

    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, "RC NOZZLE INTEGRATION TEST");
    ESP_LOGW(TAG, "========================================");

    /*
     * Test prerequisites:
     *
     * - RC transmitter powered and connected
     * - nozzle servo connected
     * - upper and lower limit switches connected
     * - nozzle starts physically at the upper/stowed limit
     * - nozzle RC switch starts in RETRACT position
     */

    ESP_LOGW(TAG, "Place nozzle at UPPER/STOWED limit");
    ESP_LOGW(TAG, "Set nozzle RC switch to RETRACT");


    /* ============================================================
     * INITIAL STATE
     * ============================================================ */

    ubTestPassCount +=
        s_waitForState(STATE_MACHINE_STOWED);


    /* ============================================================
     * EXTEND NOZZLE
     * ============================================================ */

    ESP_LOGW(TAG, "----------------------------------------");
    ESP_LOGW(TAG, "MOVE NOZZLE RC SWITCH TO EXTEND");
    ESP_LOGW(TAG, "----------------------------------------");

    /*
     * Expected:
     *
     * RC switch
     *      ↓
     * RC_INPUT_STATE_LOW/HIGH
     *      ↓
     * SM_EVENT_NOZZLE_EXTEND
     *      ↓
     * STOWED -> LOWERING
     *      ↓
     * servo physically extends
     */
    ubTestPassCount +=
        s_waitForState(STATE_MACHINE_LOWERING);


    /*
     * Now do nothing.
     *
     * The servo should continue lowering until the physical
     * lower limit switch activates.
     *
     * Expected:
     *
     * lower limit
     *      ↓
     * SM_EVENT_LOWER_LIMIT_ACTIVE
     *      ↓
     * LOWERING -> DEPLOYED
     *      ↓
     * servo stops
     */
    ubTestPassCount +=
        s_waitForState(STATE_MACHINE_DEPLOYED);


    ESP_LOGW(TAG, "Nozzle successfully DEPLOYED");


    /* ============================================================
     * RETRACT NOZZLE
     * ============================================================ */

    ESP_LOGW(TAG, "----------------------------------------");
    ESP_LOGW(TAG, "MOVE NOZZLE RC SWITCH TO RETRACT");
    ESP_LOGW(TAG, "----------------------------------------");

    /*
     * Expected:
     *
     * RC switch
     *      ↓
     * SM_EVENT_NOZZLE_RETRACT
     *      ↓
     * DEPLOYED -> RAISING
     *      ↓
     * servo physically retracts
     */
    ubTestPassCount +=
        s_waitForState(STATE_MACHINE_RAISING);


    /*
     * Again, do nothing.
     *
     * The physical upper limit should stop the movement.
     *
     * Expected:
     *
     * upper limit
     *      ↓
     * SM_EVENT_UPPER_LIMIT_ACTIVE
     *      ↓
     * RAISING -> STOWED
     *      ↓
     * servo stops
     */
    ubTestPassCount +=
        s_waitForState(STATE_MACHINE_STOWED);


    ESP_LOGW(TAG, "Nozzle successfully STOWED");


    /* ============================================================
     * RESULT
     * ============================================================ */

    ESP_LOGW(TAG, "========================================");

    if(ubTestPassCount == ubTestCount) {
        ESP_LOGI(
            TAG,
            "TEST PASSED: %u / %u checks passed",
            ubTestPassCount,
            ubTestCount
        );
    } else {
        ESP_LOGE(
            TAG,
            "TEST FAILED: %u / %u checks passed",
            ubTestPassCount,
            ubTestCount
        );
    }

    ESP_LOGW(TAG, "========================================");
}


void testIna226Basic(void) {
    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 3;

    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, "INA226 BASIC TEST");
    ESP_LOGW(TAG, "========================================");


    /* ============================================================
     * TEST 1: I2C BUS INITIALIZATION
     * ============================================================ */

    esp_err_t lErr = i2cBusInit();

    if(ESP_OK == lErr) {
        ESP_LOGI(TAG, "I2C bus initialization PASSED");
        ubTestPassCount++;
    } else {
        ESP_LOGE(
            TAG,
            "I2C bus initialization FAILED. Code: 0x%X",
            lErr
        );

        goto test_complete;
    }


    /* ============================================================
     * TEST 2: INA226 INITIALIZATION / DEVICE IDENTIFICATION
     * ============================================================ */

    lErr = ina226Init();

    if(ESP_OK == lErr) {
        ESP_LOGI(TAG, "INA226 initialization PASSED");
        ubTestPassCount++;
    } else {
        ESP_LOGE(
            TAG,
            "INA226 initialization FAILED. Code: 0x%X",
            lErr
        );

        goto test_complete;
    }


    /* ============================================================
     * TEST 3: BUS VOLTAGE REGISTER READ
     * ============================================================ */

    float fBusVoltage = 0.0f;

    lErr = ina226ReadBusVoltage(
        &fBusVoltage
    );

    if(ESP_OK == lErr) {
        ESP_LOGI(
            TAG,
            "INA226 bus voltage: %.3f V",
            fBusVoltage
        );

        ESP_LOGI(TAG, "Bus voltage read PASSED");
        ubTestPassCount++;
    } else {
        ESP_LOGE(
            TAG,
            "Bus voltage read FAILED. Code: 0x%X",
            lErr
        );
    }


test_complete:

    ESP_LOGW(TAG, "========================================");

    if(ubTestPassCount == ubTestCount) {
        ESP_LOGI(
            TAG,
            "TEST PASSED: %u / %u checks passed",
            ubTestPassCount,
            ubTestCount
        );
    } else {
        ESP_LOGE(
            TAG,
            "TEST FAILED: %u / %u checks passed",
            ubTestPassCount,
            ubTestCount
        );
    }

    ESP_LOGW(TAG, "========================================");
}
