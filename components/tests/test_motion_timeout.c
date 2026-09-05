#include "tests.h"
#include "esp_log.h"
#include "esp_err.h"

#include "state_machine.h"
#include "motion_timeout.h"


static const char* TAG = "TEST_LIMIT_SWITCH";



void testMotionTimeoutSequence(void) {
    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 7;

    esp_err_t lErr = ESP_OK;

    ESP_LOGW(TAG, "=== TEST: MOTION TIMEOUT SEQUENCE ===");

    /*
     * Test starts in POSITION_UNKNOWN.
     */
    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);

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

        return;
    }

    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);

    /*
     * Timeout is 10 seconds.
     * Give some additional margin for scheduling/event processing.
     */
    vTaskDelay(pdMS_TO_TICKS(11000));

    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);


    /*
     * Recover from FAULT.
     */
    s_postTestEvent(SM_EVENT_RESET, 500);

    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


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

        return;
    }

    vTaskDelay(pdMS_TO_TICKS(3000));

    lErr = motionTimeoutStop();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to stop motion timeout. Code: 0x%X", lErr);

        return;
    }

    /*
     * Wait longer than a complete timeout period.
     * If stop worked, we must remain POSITION_UNKNOWN.
     */
    vTaskDelay(pdMS_TO_TICKS(11000));

    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * TEST 3:
     * Verify that calling Start() again while the timer is already
     * running restarts the countdown.
     */
    ESP_LOGW(TAG, "TEST 3: Restart active motion timeout");

    lErr = motionTimeoutStart();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to start motion timeout. Code: 0x%X", lErr);

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
        ESP_LOGE(TAG, "Failed to restart motion timeout. Code: 0x%X", lErr);

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

    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * Wait another 3 seconds.
     *
     * We are now 11 seconds beyond the reset point, so the timer
     * should have expired and generated FAULT.
     */
    vTaskDelay(pdMS_TO_TICKS(3000));

    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);


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
    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /* ============================================================
     * TEST 1: LOWERING timeout
     * ============================================================ */

    ESP_LOGW(TAG, "TEST 1: LOWERING timeout");
    ESP_LOGW(TAG, "PRESS UPPER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    /*
     * POSITION_UNKNOWN + UPPER_LIMIT_ACTIVE -> STOWED
     */
    ubTestPassCount += s_checkState(STATE_MACHINE_STOWED);


    ESP_LOGW(TAG, "RELEASE UPPER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(3000));

    ubTestPassCount += s_checkState(STATE_MACHINE_STOWED);


    /*
     * STOWED -> LOWERING
     *
     * LOWERING.cbInit() should:
     *   - start servo extension
     *   - start motion timeout
     */
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 500);

    ubTestPassCount += s_checkState(STATE_MACHINE_LOWERING);


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
     *      ├── servo stop
     *      └── timeout stop
     *      ↓
     * FAULT
     */
    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);


    /*
     * Recover.
     */
    s_postTestEvent(SM_EVENT_RESET, 500);

    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /* ============================================================
     * TEST 2: RAISING timeout
     * ============================================================ */

    ESP_LOGW(TAG, "TEST 2: RAISING timeout");
    ESP_LOGW(TAG, "PRESS LOWER limit switch now");

    vTaskDelay(pdMS_TO_TICKS(5000));

    /*
     * POSITION_UNKNOWN + LOWER_LIMIT_ACTIVE -> DEPLOYED
     */
    ubTestPassCount +=s_checkState(STATE_MACHINE_DEPLOYED);


    ESP_LOGW(TAG, "RELEASE LOWER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(3000));

    ubTestPassCount += s_checkState(STATE_MACHINE_DEPLOYED);


    /*
     * DEPLOYED -> RAISING
     *
     * RAISING.cbInit() should:
     *   - start servo retraction
     *   - start motion timeout
     */
    s_postTestEvent(SM_EVENT_NOZZLE_RETRACT, 500);

    ubTestPassCount += s_checkState(STATE_MACHINE_RAISING);


    /*
     * DO NOT press upper limit.
     */
    ESP_LOGW(TAG, "DO NOT PRESS UPPER limit - waiting for timeout...");

    vTaskDelay(pdMS_TO_TICKS(11000));

    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);


    /*
     * Recover again.
     */
    s_postTestEvent(SM_EVENT_RESET, 500);

    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /* ============================================================
     * TEST 3: Successful motion cancels timeout
     * ============================================================ */

    ESP_LOGW(TAG, "TEST 3: Successful motion cancels timeout");
    ESP_LOGW(TAG, "PRESS UPPER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount += s_checkState(STATE_MACHINE_STOWED);

    ESP_LOGW(TAG, "RELEASE UPPER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(3000));

    ubTestPassCount += s_checkState(STATE_MACHINE_STOWED);


    /*
     * Begin normal lowering.
     */
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 500);

    ubTestPassCount += s_checkState(STATE_MACHINE_LOWERING);


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
     *      ├── servo stop
     *      └── motionTimeoutStop()
     *
     * then:
     *
     * LOWERING -> DEPLOYED
     */
    ubTestPassCount += s_checkState(STATE_MACHINE_DEPLOYED);


    /*
     * Now deliberately wait LONGER than the full timeout.
     *
     * If cbDeinit() successfully cancelled the timer,
     * no delayed FAULT should appear.
     */
    ESP_LOGW(TAG,"Waiting 11 seconds to verify timeout was cancelled...");
    vTaskDelay(pdMS_TO_TICKS(11000));

    ubTestPassCount += s_checkState(STATE_MACHINE_DEPLOYED);


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

    ESP_LOGW(TAG, "=== END MOTION TIMEOUT INTEGRATION TEST ===");
}

