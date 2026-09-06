#include "tests.h"
#include "test_helpers.h"

#include <stdint.h>

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nozzle_servo.h"
#include "pump.h"
#include "motion_timeout.h"
#include "limit_switch.h"

#include "state_machine.h"
#include "state_machine_common.h"
#include "state_machine_states.h"


static const char* TAG = "TEST_MOTION_TIMEOUT";


static void s_testBaseDeinit(void) {
    stateMachineDeinit();
    motionTimeoutDeinit();
    pumpDeinit();
    nozzleServoDeinit();
}


static esp_err_t s_testBaseInit(void) {
    esp_err_t lErr = ESP_OK;

    lErr = nozzleServoInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init nozzle servo. Code: 0x%X", lErr);

        goto init_fail;
    }

    lErr = pumpInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init pump. Code: 0x%X", lErr);

        goto init_fail;
    }

    lErr = motionTimeoutInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init motion timeout. Code: 0x%X", lErr);

        goto init_fail;
    }

    lErr = stateMachineStatesRegister();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to register states. Code: 0x%X", lErr);

        goto init_fail;
    }

    lErr = stateMachineInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init state machine. Code: 0x%X", lErr);

        goto init_fail;
    }

    return ESP_OK;

init_fail:

    s_testBaseDeinit();

    return lErr;
}


void testMotionTimeoutSequence(void) {
    esp_err_t lErr = ESP_OK;

    lErr = s_testBaseInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init test components. Code: 0x%X", lErr);

        return;
    }

    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 7;

    ESP_LOGW(TAG, "=== TEST: MOTION TIMEOUT SEQUENCE ===");

    /*
    *   complete the state machine startup
    *   
    *   INIT -> POSITION_UNKNOWN
    */
    testPostEvent(SM_EVENT_SYSTEM_READY, 500);

    /*
     * Test starts in POSITION_UNKNOWN.
     */
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);

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
        ESP_LOGE(TAG, "Failed to start motion timeout. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);

    /*
     * Timeout is 10 seconds.
     * Give some additional margin for scheduling/event processing.
     */
    vTaskDelay(pdMS_TO_TICKS(11000));

    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);


    /*
     * Recover from FAULT.
     */
    testPostEvent(SM_EVENT_RESET, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);


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
        ESP_LOGE(TAG, "Failed to start motion timeout. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    vTaskDelay(pdMS_TO_TICKS(3000));

    lErr = motionTimeoutStop();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to stop motion timeout. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    /*
     * Wait longer than a complete timeout period.
     * If stop worked, we must remain POSITION_UNKNOWN.
     */
    vTaskDelay(pdMS_TO_TICKS(11000));

    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * TEST 3:
     * Verify that calling Start() again while the timer is already
     * running restarts the countdown.
     */
    ESP_LOGW(TAG, "TEST 3: Restart active motion timeout");

    lErr = motionTimeoutStart();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to start motion timeout. Code: 0x%X", lErr);

        goto test_cleanup;
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
        ESP_LOGE(TAG, "Failed to restart motion timeout. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    /*
     * Wait 8 seconds.
     *
     * 11 seconds have passed since the first Start(), but only
     * 8 seconds since the reset. Therefore we should NOT be
     * in FAULT yet.
     */
    vTaskDelay(pdMS_TO_TICKS(8000));

    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * Wait another 3 seconds.
     *
     * We are now 11 seconds beyond the reset point, so the timer
     * should have expired and generated FAULT.
     */
    vTaskDelay(pdMS_TO_TICKS(3000));

    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);


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

test_cleanup:

    s_testBaseDeinit();

    ESP_LOGW(TAG, "=== END MOTION TIMEOUT TEST ===");
}


void testMotionTimeoutIntegrationSequence(void) {
    esp_err_t lErr = s_testBaseInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init test components. Code: 0x%X", lErr);

        return;
    }
    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 16;

    ESP_LOGW(TAG, "=== TEST: MOTION TIMEOUT INTEGRATION ===");
    ESP_LOGW(TAG, "Ensure BOTH limit switches are RELEASED");

    vTaskDelay(pdMS_TO_TICKS(3000));

    /*
    * Complete FSM startup before enabling the external
    * limit-switch event source.
    *
    * INIT -> POSITION_UNKNOWN
    */
    testPostEvent(SM_EVENT_SYSTEM_READY, 500);

    // initialize limit switches
    lErr = limitSwitchInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init limit switches. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    lErr = limitSwitchSyncState();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to sync limit switches. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    /*
     * Test should begin in POSITION_UNKNOWN.
     */
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);


    /* ============================================================
     * TEST 1: LOWERING timeout
     * ============================================================ */

    ESP_LOGW(TAG, "TEST 1: LOWERING timeout");
    ESP_LOGW(TAG, "PRESS UPPER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    /*
     * POSITION_UNKNOWN + UPPER_LIMIT_ACTIVE -> STOWED
     */
    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);


    ESP_LOGW(TAG, "RELEASE UPPER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(3000));

    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);


    /*
     * STOWED -> LOWERING
     *
     * LOWERING.cbInit() should:
     *   - start servo extension
     *   - start motion timeout
     */
    testPostEvent(SM_EVENT_NOZZLE_EXTEND, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_LOWERING);


    /*
     * DO NOT press the lower limit.
     *
     * After ~10 seconds the timer should expire and generate FAULT.
     */
    ESP_LOGW(TAG, "DO NOT PRESS LOWER limit - waiting for timeout...");

    vTaskDelay(pdMS_TO_TICKS(11000));

    /*
    * Expected:
    *
    * motion timeout expires
    *      ↓
    * SM_EVENT_FAULT
    *      ↓
    * LOWERING.cbDeinit()
    *      ├── disable servo PWM output
    *      └── stop motion timeout
    *      ↓
    * FAULT
    */
    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);


    /*
     * Recover.
     */
    testPostEvent(SM_EVENT_RESET, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);


    /* ============================================================
     * TEST 2: RAISING timeout
     * ============================================================ */

    ESP_LOGW(TAG, "TEST 2: RAISING timeout");
    ESP_LOGW(TAG, "PRESS LOWER limit switch now");

    vTaskDelay(pdMS_TO_TICKS(5000));

    /*
     * POSITION_UNKNOWN + LOWER_LIMIT_ACTIVE -> DEPLOYED
     */
    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);


    ESP_LOGW(TAG, "RELEASE LOWER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(3000));

    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);


    /*
     * DEPLOYED -> RAISING
     *
     * RAISING.cbInit() should:
     *   - start servo retraction
     *   - start motion timeout
     */
    testPostEvent(SM_EVENT_NOZZLE_RETRACT, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_RAISING);


    /*
     * DO NOT press upper limit.
     */
    ESP_LOGW(TAG, "DO NOT PRESS UPPER limit - waiting for timeout...");

    vTaskDelay(pdMS_TO_TICKS(11000));

    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);


    /*
     * Recover again.
     */
    testPostEvent(SM_EVENT_RESET, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);


    /* ============================================================
     * TEST 3: Successful motion cancels timeout
     * ============================================================ */

    ESP_LOGW(TAG, "TEST 3: Successful motion cancels timeout");
    ESP_LOGW(TAG, "PRESS UPPER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);

    ESP_LOGW(TAG, "RELEASE UPPER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(3000));

    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);


    /*
     * Begin normal lowering.
     */
    testPostEvent(SM_EVENT_NOZZLE_EXTEND, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_LOWERING);


    /*
     * This time, activate the lower limit BEFORE the
     * 10-second timeout expires.
     */
    ESP_LOGW(TAG, "PRESS LOWER limit switch within 5 seconds");

    vTaskDelay(pdMS_TO_TICKS(5000));

    /*
     * LOWER_LIMIT_ACTIVE should cause:
     *
     * LOWERING.cbDeinit()
     *   ├── disable servo PWM output
     *   └── stop motion timeout
     *
     * then:
     *
     * LOWERING -> DEPLOYED
     */
    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);


    /*
     * Now deliberately wait LONGER than the full timeout.
     *
     * If cbDeinit() successfully cancelled the timer,
     * no delayed FAULT should appear.
     */
    ESP_LOGW(TAG,"Waiting 11 seconds to verify timeout was cancelled...");
    vTaskDelay(pdMS_TO_TICKS(11000));

    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);


    /* ============================================================
     * RESULT
     * ============================================================ */

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

test_cleanup:

    limitSwitchDeinit();
    s_testBaseDeinit();

    ESP_LOGW(TAG, "=== END MOTION TIMEOUT INTEGRATION TEST ===");
}

