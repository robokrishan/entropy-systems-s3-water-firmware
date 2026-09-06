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
#include "rc_input.h"

#include "state_machine.h"
#include "state_machine_common.h"
#include "state_machine_states.h"


static const char* TAG = "TEST_RC_INPUT";


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


void testRcSignalLossSequence(void) {
    esp_err_t lErr = s_testBaseInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init test components. Code: 0x%X", lErr);

        return;
    }

    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 5;

    testPostEvent(SM_EVENT_SYSTEM_READY, 500);
    /*
     * We should still be in POSITION_UNKNOWN.
     */
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);

    /*
     * Put the FSM into DEPLOYED using a synthetic endpoint event.
     */
    testPostEvent(SM_EVENT_LOWER_LIMIT_ACTIVE, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);

    ESP_LOGW(TAG, "=== TEST: RC SIGNAL LOSS ===");
    ESP_LOGW(TAG, "Ensure BOTH RC PWM signals are CONNECTED");
    ESP_LOGW(TAG, "Set pump RC switch to OFF");
    ESP_LOGW(TAG, "Set nozzle RC switch to EXTEND");

    vTaskDelay(pdMS_TO_TICKS(5000));


    lErr = rcInputInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init RC input. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    vTaskDelay(pdMS_TO_TICKS(3000));

    /*
     * Enter PUMPING.
     *
     * This makes RC signal loss observable through an actual
     * FSM transition:
     *
     * PUMPING + RC_SIGNAL_LOST -> DEPLOYED
     */
    testPostEvent(SM_EVENT_PUMP_ON, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_PUMPING);

    /*
    * Disconnect both RC PWM signal inputs.
    *
    * The signal-loss timer is shared by both RC channels, so either
    * channel continuing to receive valid PWM will keep the timer alive.
    *
    * Do NOT disconnect ESP32 power or common ground.
    */
    ESP_LOGW(TAG, "DISCONNECT BOTH RC PWM SIGNALS NOW");
    vTaskDelay(pdMS_TO_TICKS(8000));

    /*
     * Expected after ~500 milliseconds:
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
    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);


     /*
     * Reconnect both PWM inputs.
     *
     * Keep:
     *   pump   -> OFF
     *   nozzle -> EXTEND
     *
     * After signal restoration the RC states are treated as new
     * commands, but both commands are harmless while DEPLOYED.
     */
    ESP_LOGW(TAG, "RECONNECT BOTH RC PWM SIGNALS NOW");
    ESP_LOGW(TAG, "Keep pump switch OFF");
    ESP_LOGW(TAG, "Keep nozzle switch at EXTEND");

    vTaskDelay(pdMS_TO_TICKS(8000));

    /*
    * Expected logs should indicate that the RC signal was restored
    * and that the current pump and nozzle commands were processed
    * as new RC states.
    *
    * With:
    *   pump   -> OFF
    *   nozzle -> EXTEND
    *
    * both commands are harmless while DEPLOYED, so the FSM should
    * remain in DEPLOYED.
    */
    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);


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

    rcInputDeinit();
    s_testBaseDeinit();

    ESP_LOGW(TAG, "=== END RC SIGNAL LOSS TEST ===");
}

