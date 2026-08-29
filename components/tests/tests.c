#include "tests.h"
#include "esp_err.h"
#include "esp_log.h"
#include "state_machine_common.h"
#include "state_machine.h"
#include "nozzle_servo.h"
#include "pump.h"
#include "limit_switch.h"
#include "motion_timeout.h"

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