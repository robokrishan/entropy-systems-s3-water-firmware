#include "tests.h"
#include "test_helpers.h"

#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"

#include "nozzle_servo.h"
#include "pump.h"
#include "motion_timeout.h"

#include "state_machine_common.h"
#include "state_machine.h"
#include "state_machine_states.h"


static const char* TAG = "TEST_STATE_MACHINE";


/**
 * @brief Deinitialize components used by state-machine tests.
 *
 * Deinitializes test dependencies in reverse initialization order.
 * Each component deinitialization is null-safe and performs best-effort
 * cleanup, allowing this function to be called after partial initialization.
 */
static void s_testDeinit(void) {
    stateMachineDeinit();
    motionTimeoutDeinit();
    pumpDeinit();
    nozzleServoDeinit();
}


/**
 * @brief Initialize components required by state-machine tests.
 *
 * Initializes only the actuator, timeout, state registration, and
 * state-machine components required by this test module.
 *
 * @return ESP_OK on success, otherwise an appropriate error code.
 */
static esp_err_t s_testInit(void) {
    esp_err_t lErr = ESP_OK;

    lErr = nozzleServoInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init nozzle servo! Code: 0x%X", lErr);

        goto init_fail;
    }

    lErr = pumpInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init pump! Code: 0x%X", lErr);

        goto init_fail;
    }

    lErr = motionTimeoutInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init motion timeout! Code: 0x%X", lErr);

        goto init_fail;
    }

    lErr = stateMachineStatesInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init state machine states! Code: 0x%X", lErr);

        goto init_fail;
    }

    lErr = stateMachineInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init state machine! Code: 0x%X", lErr);

        goto init_fail;
    }

    return ESP_OK;

init_fail:

    s_testDeinit();
    return lErr;
}


void testNormalSequence(void) {
    esp_err_t lErr = ESP_OK;

    lErr = s_testInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init test components. Code: 0x%X", lErr);

        return;
    }

    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 9;

    ubTestPassCount += testCheckState(STATE_MACHINE_INIT);

    testPostEvent(SM_EVENT_SYSTEM_READY, 3000);
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);

    testPostEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 3000);
    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);

    testPostEvent(SM_EVENT_NOZZLE_EXTEND, 3000);
    ubTestPassCount += testCheckState(STATE_MACHINE_LOWERING);

    testPostEvent(SM_EVENT_LOWER_LIMIT_ACTIVE, 3000);
    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);

    testPostEvent(SM_EVENT_PUMP_ON, 3000);
    ubTestPassCount += testCheckState(STATE_MACHINE_PUMPING);

    testPostEvent(SM_EVENT_PUMP_OFF, 3000);
    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);

    testPostEvent(SM_EVENT_NOZZLE_RETRACT, 3000);
    ubTestPassCount += testCheckState(STATE_MACHINE_RAISING);

    testPostEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 3000);
    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);

    ESP_LOGW(TAG, "Passed %d / %d tests", ubTestPassCount, ubTestCount);

    s_testDeinit();

    ESP_LOGW(TAG, "=== END TEST ===");
}


void testWrongSequence(void) {
    esp_err_t lErr = ESP_OK;

    lErr = s_testInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init test components. Code: 0x%X", lErr);

        return;
    }

    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 15;

    /*
     * Establish known starting position.
     *
     * INIT -> POSITION_UNKNOWN -> STOWED
     */
    ubTestPassCount += testCheckState(STATE_MACHINE_INIT);
    testPostEvent(SM_EVENT_SYSTEM_READY, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);

    testPostEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);

    /*
     * STOWED + PUMP_ON
     *
     * Must be rejected.
     * Remain in STOWED.
     */
    testPostEvent(SM_EVENT_PUMP_ON, 1000);
    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);

    /*
     * STOWED -> LOWERING
     */
    testPostEvent(SM_EVENT_NOZZLE_EXTEND, 1000);
    ubTestPassCount += testCheckState(STATE_MACHINE_LOWERING);

    /*
     * LOWERING + PUMP_ON
     *
     * Must be rejected.
     * Remain in LOWERING.
     */
    testPostEvent(SM_EVENT_PUMP_ON, 1000);
    ubTestPassCount += testCheckState(STATE_MACHINE_LOWERING);

    /*
     * WRONG LIMIT TEST:
     *
     * We are LOWERING, so the upper limit should not
     * complete the movement.
     *
     * Must remain in LOWERING.
     */
    testPostEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 1000);
    ubTestPassCount += testCheckState(STATE_MACHINE_LOWERING);

    /*
     * Stop before reaching lower limit.
     *
     * LOWERING -> POSITION_UNKNOWN
     */
    testPostEvent(SM_EVENT_STOP_SPOOL, 1000);
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);

    /*
     * POSITION_UNKNOWN -> RAISING
     */
    testPostEvent(SM_EVENT_NOZZLE_RETRACT, 1000);
    ubTestPassCount += testCheckState(STATE_MACHINE_RAISING);

    /*
     * WRONG LIMIT TEST:
     *
     * We are RAISING, so the lower limit should not
     * complete the movement.
     *
     * Must remain in RAISING.
     */
    testPostEvent(SM_EVENT_LOWER_LIMIT_ACTIVE, 1000);
    ubTestPassCount += testCheckState(STATE_MACHINE_RAISING);

    /*
     * Correct upper limit.
     *
     * RAISING -> STOWED
     */
    testPostEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 1000);
    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);

    /*
     * STOWED -> LOWERING
     */
    testPostEvent(SM_EVENT_NOZZLE_EXTEND, 1000);
    ubTestPassCount += testCheckState(STATE_MACHINE_LOWERING);

    /*
     * Global fault.
     *
     * LOWERING -> FAULT
     */
    testPostEvent(SM_EVENT_FAULT, 1000);
    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);

    /*
     * Must be rejected while in FAULT.
     */
    testPostEvent(SM_EVENT_PUMP_ON, 1000);
    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);

    /*
     * FAULT -> POSITION_UNKNOWN
     */
    testPostEvent(SM_EVENT_RESET, 1000);
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);

    ESP_LOGW(TAG, "Passed %d / %d tests", ubTestPassCount, ubTestCount);

    s_testDeinit();

    ESP_LOGW(TAG, "=== END TEST ===");
}


void testFaultSequence(void) {
    esp_err_t lErr = ESP_OK;

    lErr = s_testInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init test components. Code: 0x%X", lErr);

        return;
    }

    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 7;

    /* Get into a known STOWED state first */
    testPostEvent(SM_EVENT_SYSTEM_READY, 2000);
    testPostEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 1000);

    /* Start moving */
    testPostEvent(SM_EVENT_NOZZLE_EXTEND, 5000);
    ubTestPassCount += testCheckState(STATE_MACHINE_LOWERING);

    /*
     * Servo should currently be extending.
     * FAULT should stop the servo and enter FAULT.
     */
    testPostEvent(SM_EVENT_FAULT, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);

    /*
     * Normal commands should have no effect while faulted.
     */
    testPostEvent(SM_EVENT_NOZZLE_EXTEND, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);
    testPostEvent(SM_EVENT_PUMP_ON, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);

    /*
     * RESET should clear the fault, but physical
     * nozzle position is no longer known.
     */
    testPostEvent(SM_EVENT_RESET, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);


    /* Verify normal operation can resume*/
    testPostEvent(SM_EVENT_NOZZLE_EXTEND, 5000);
    ubTestPassCount += testCheckState(STATE_MACHINE_LOWERING);

    testPostEvent(SM_EVENT_LOWER_LIMIT_ACTIVE, 2000);
    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);

    ESP_LOGW(TAG, "Passed %d / %d tests", ubTestPassCount, ubTestCount);

    s_testDeinit();

    ESP_LOGW(TAG, "=== END TEST ===");
}


void testStateInitFailure(void) {
    /*
        Force state_machine_state_lowering.s_stateInit() to
        return ESP_FAIL, then run this test.
    */

    esp_err_t lErr = ESP_OK;

    lErr = s_testInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init test components. Code: 0x%X", lErr);

        return;
    }

    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 7;

    ESP_LOGW(TAG, "=== TEST: State init failure ===");

    /* INIT -> POSITION_UNKNOWN */
    testPostEvent(SM_EVENT_SYSTEM_READY, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);

    /* POSITION_UNKNOWN -> STOWED */
    testPostEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);

    /*
     * STOWED -> LOWERING
     *
     * LOWERING.cbInit() is deliberately returning ESP_FAIL,
     * so the state machine should force itself into FAULT.
     */
    testPostEvent(SM_EVENT_NOZZLE_EXTEND, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);

    /*
     * Verify FAULT remains latched.
     */
    testPostEvent(SM_EVENT_NOZZLE_EXTEND, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);

    testPostEvent(SM_EVENT_PUMP_ON, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);

    /*
     * Verify recovery still works.
     */
    testPostEvent(SM_EVENT_RESET, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);

    testPostEvent(SM_EVENT_NOZZLE_RETRACT, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_RAISING);

    ESP_LOGW(TAG, "Passed %d / %d tests", ubTestPassCount, ubTestCount);

    s_testDeinit();

    ESP_LOGW(TAG, "=== END TEST ===");
}


void testStateDeinitFailure(void) {
    esp_err_t lErr = ESP_OK;

    lErr = s_testInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init test components. Code: 0x%X", lErr);

        return;
    }

    /*
        Force state_machine_state_lowering.s_stateDeinit() to
        return ESP_FAIL, then run this test.
    */

    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 7;
    ESP_LOGW(TAG, "=== TEST: State deinit failure ===");

    /* INIT -> POSITION_UNKNOWN */
    testPostEvent(SM_EVENT_SYSTEM_READY, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);

    /* POSITION_UNKNOWN -> STOWED */
    testPostEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);

    /* STOWED -> LOWERING */
    testPostEvent(SM_EVENT_NOZZLE_EXTEND, 1000);
    ubTestPassCount += testCheckState(STATE_MACHINE_LOWERING);

    /*
     * LOWER_LIMIT_ACTIVE normally causes:
     *
     * LOWERING -> DEPLOYED
     *
     * But LOWERING.cbDeinit() has been deliberately
     * made to fail. Therefore the state machine
     * should instead enter FAULT.
     */
    testPostEvent(SM_EVENT_LOWER_LIMIT_ACTIVE, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);

    /*
     * FAULT must remain latched.
     */
    testPostEvent(SM_EVENT_NOZZLE_RETRACT, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);

    /*
     * RESET should recover to POSITION_UNKNOWN.
     */
    testPostEvent(SM_EVENT_RESET, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);

    testPostEvent(SM_EVENT_NOZZLE_RETRACT, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_RAISING);

    ESP_LOGW(TAG, "Passed %d / %d tests", ubTestPassCount, ubTestCount);

    s_testDeinit();

    ESP_LOGW(TAG, "=== END TEST ===");
}


void testHaltSequence(void) {
    esp_err_t lErr = ESP_OK;

    lErr = s_testInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init test components. Code: 0x%X", lErr);

        return;
    }


    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 19;

    ESP_LOGW(TAG, "=== TEST: HALT sequence ===");


    /*
     * INIT + HALT
     *
     * Nothing is active yet.
     * Must remain in INIT.
     */
    ubTestPassCount += testCheckState(STATE_MACHINE_INIT);

    testPostEvent(SM_EVENT_HALT, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_INIT);


    /*
     * INIT -> POSITION_UNKNOWN
     */
    testPostEvent(SM_EVENT_SYSTEM_READY, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * POSITION_UNKNOWN + HALT
     *
     * Already stationary with unknown position.
     * Must remain in POSITION_UNKNOWN.
     */
    testPostEvent(SM_EVENT_HALT, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * Resolve position:
     * POSITION_UNKNOWN -> STOWED
     */
    testPostEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);


    /*
     * STOWED + HALT
     *
     * Already stationary.
     * Must remain in STOWED.
     */
    testPostEvent(SM_EVENT_HALT, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);


    /*
     * STOWED -> LOWERING
     */
    testPostEvent(SM_EVENT_NOZZLE_EXTEND, 1000);
    ubTestPassCount += testCheckState(STATE_MACHINE_LOWERING);


    /*
     * LOWERING + HALT
     *
     * Motion is interrupted before reaching a known
     * endpoint, so position becomes unknown.
     *
     * Servo should physically stop.
     */
    testPostEvent(SM_EVENT_HALT, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * Resume lowering from unknown position.
     */
    testPostEvent(SM_EVENT_NOZZLE_EXTEND, 1000);
    ubTestPassCount += testCheckState(STATE_MACHINE_LOWERING);


    /*
     * Simulate reaching lower limit.
     *
     * LOWERING -> DEPLOYED
     */
    testPostEvent(SM_EVENT_LOWER_LIMIT_ACTIVE, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);


    /*
     * DEPLOYED + HALT
     *
     * Already stationary and physical position is known.
     * Must remain DEPLOYED.
     */
    testPostEvent(SM_EVENT_HALT, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);


    /*
     * DEPLOYED -> PUMPING
     */
    testPostEvent(SM_EVENT_PUMP_ON, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_PUMPING);


    /*
     * PUMPING + HALT
     *
     * Pump operation stops, but nozzle remains
     * physically deployed.
     *
     * PUMPING -> DEPLOYED
     */
    testPostEvent(SM_EVENT_HALT, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);


    /*
     * DEPLOYED -> RAISING
     */
    testPostEvent(SM_EVENT_NOZZLE_RETRACT, 1000);
    ubTestPassCount += testCheckState(STATE_MACHINE_RAISING);


    /*
     * RAISING + HALT
     *
     * Motion is interrupted before reaching a known
     * endpoint, so position becomes unknown.
     *
     * Servo should physically stop.
     */
    testPostEvent(SM_EVENT_HALT, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * Enter FAULT.
     */
    testPostEvent(SM_EVENT_FAULT, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);


    /*
     * FAULT + HALT
     *
     * HALT must never clear a fault.
     */
    testPostEvent(SM_EVENT_HALT, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);


    /*
     * RESET clears the fault but position remains unknown.
     */
    testPostEvent(SM_EVENT_RESET, 500);
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * Verify normal operation can resume after HALT/fault handling.
     */
    testPostEvent(SM_EVENT_NOZZLE_RETRACT, 1000);
    ubTestPassCount += testCheckState(STATE_MACHINE_RAISING);


    if(ubTestPassCount == ubTestCount) {
        ESP_LOGI(TAG, "TEST PASSED: %d / %d checks passed",
            ubTestPassCount,
            ubTestCount
        );
    } else {
        ESP_LOGE(TAG, "TEST FAILED: %d / %d checks passed",
            ubTestPassCount,
            ubTestCount
        );
    }

    s_testDeinit();

    ESP_LOGW(TAG, "=== END TEST ===");
}
