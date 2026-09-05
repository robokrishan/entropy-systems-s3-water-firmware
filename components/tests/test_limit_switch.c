#include "tests.h"
#include "esp_log.h"

#include "limit_switch.h"


const char* TAG = "TEST_LIMIT_SWITCH";



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
        ESP_LOGE(TAG, "Failed to synchronize limit switches. Code: 0x%X", lErr);

        return;
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * TEST UPPER LIMIT ACTIVE
     *
     * Physically press the upper limit switch.
     *
     * POSITION_UNKNOWN + UPPER_LIMIT_ACTIVE -> STOWED
     */
    ESP_LOGW(TAG, "PRESS UPPER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount += s_checkState(STATE_MACHINE_STOWED);


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

    ubTestPassCount += s_checkState(STATE_MACHINE_STOWED);

    /*
     * Begin lowering.
     *
     * STOWED -> LOWERING
     */
    s_postTestEvent(SM_EVENT_NOZZLE_EXTEND, 500);

    ubTestPassCount += s_checkState(STATE_MACHINE_LOWERING);

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

    ubTestPassCount += s_checkState(STATE_MACHINE_DEPLOYED);

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

    ubTestPassCount += s_checkState(STATE_MACHINE_DEPLOYED);

    /*
     * Begin raising.
     *
     * DEPLOYED -> RAISING
     */
    s_postTestEvent(SM_EVENT_NOZZLE_RETRACT, 500);

    ubTestPassCount += s_checkState(STATE_MACHINE_RAISING);

    /*
     * TEST UPPER LIMIT ACTIVE DURING RAISING
     *
     * RAISING + UPPER_LIMIT_ACTIVE -> STOWED
     *
     * Servo should physically stop.
     */
    ESP_LOGW(TAG, "PRESS UPPER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount += s_checkState(STATE_MACHINE_STOWED);

    /*
     * TEST INVALID PHYSICAL CONDITION
     *
     * Keep UPPER pressed and now press LOWER as well.
     *
     * Both switches active simultaneously must generate FAULT.
     */
    ESP_LOGW(TAG, "KEEP UPPER PRESSED and PRESS LOWER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);

    /*
     * Release both switches.
     *
     * Release events should not clear FAULT.
     */
    ESP_LOGW(TAG, "RELEASE BOTH limit switches now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount += s_checkState(STATE_MACHINE_FAULT);

    /*
     * Reset fault.
     *
     * Neither physical endpoint is currently asserted,
     * therefore position returns to unknown.
     */
    s_postTestEvent(SM_EVENT_RESET, 500);

    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);


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

    ESP_LOGW(TAG, "=== END LIMIT SWITCH TEST ===");
}


