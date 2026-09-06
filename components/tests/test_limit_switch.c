#include "tests.h"
#include "test_helpers.h"

#include <stdint.h>

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nozzle_servo.h"
#include "limit_switch.h"
#include "pump.h"
#include "motion_timeout.h"

#include "state_machine.h"
#include "state_machine_states.h"
#include "state_machine_common.h"


static const char* TAG = "TEST_LIMIT_SWITCH";


static void s_testDeinit(void) {
    limitSwitchDeinit();
    stateMachineDeinit();
    motionTimeoutDeinit();
    pumpDeinit();
    nozzleServoDeinit();
}


static esp_err_t s_testInit(void) {
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

    s_testDeinit();

    return lErr;
}


void testLimitSwitchSequence(void) {
    esp_err_t lErr = s_testInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init test components. Code: 0x%X", lErr);

        return;
    }

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
    testPostEvent(SM_EVENT_SYSTEM_READY, 500);

    lErr = limitSwitchInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init limit switches. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    lErr = limitSwitchSyncState();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to synchronize limit switches. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);


    /*
     * TEST UPPER LIMIT ACTIVE
     *
     * Physically press the upper limit switch.
     *
     * POSITION_UNKNOWN + UPPER_LIMIT_ACTIVE -> STOWED
     */
    ESP_LOGW(TAG, "PRESS UPPER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);


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

    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);

    /*
     * Begin lowering.
     *
     * STOWED -> LOWERING
     */
    testPostEvent(SM_EVENT_NOZZLE_EXTEND, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_LOWERING);

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

    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);

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

    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);

    /*
     * Begin raising.
     *
     * DEPLOYED -> RAISING
     */
    testPostEvent(SM_EVENT_NOZZLE_RETRACT, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_RAISING);

    /*
     * TEST UPPER LIMIT ACTIVE DURING RAISING
     *
     * RAISING + UPPER_LIMIT_ACTIVE -> STOWED
     *
     * Servo should physically stop.
     */
    ESP_LOGW(TAG, "PRESS UPPER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);

    /*
     * TEST INVALID PHYSICAL CONDITION
     *
     * Keep UPPER pressed and now press LOWER as well.
     *
     * Both switches active simultaneously must generate FAULT.
     */
    ESP_LOGW(TAG, "KEEP UPPER PRESSED and PRESS LOWER limit switch now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);

    /*
     * Release both switches.
     *
     * Release events should not clear FAULT.
     */
    ESP_LOGW(TAG, "RELEASE BOTH limit switches now");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);

    /*
     * Reset fault.
     *
     * Neither physical endpoint is currently asserted,
     * therefore position returns to unknown.
     */
    testPostEvent(SM_EVENT_RESET, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);


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

    s_testDeinit();

    ESP_LOGW(TAG, "=== END LIMIT SWITCH TEST ===");
}


