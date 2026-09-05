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
#include "rc_input.h"

#include "state_machine.h"
#include "state_machine_states.h"
#include "state_machine_common.h"


static const char* TAG = "TEST_RC_INTEGRATION";


/**
 * @brief Deinitialize the base components used by RC integration tests.
 */
static void s_testBaseDeinit(void) {
    stateMachineDeinit();
    motionTimeoutDeinit();
    pumpDeinit();
    nozzleServoDeinit();
}


/**
 * @brief Initialize the base components required by RC integration tests.
 *
 * Does not initialize RC input or limit switches, allowing each test to
 * initialize only the external-input components it requires.
 *
 * @return ESP_OK on success, otherwise an appropriate error code.
 */
static esp_err_t s_testBaseInit(void) {
    esp_err_t lErr = ESP_OK;

    lErr = nozzleServoInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init nozzle servo! Code: 0x%X", lErr);
        
        goto init_fail;
    }

    lErr = pumpInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init pump. Code: 0x%X", lErr);
        goto init_fail;
    }

    lErr = motionTimeoutInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init motion timeout! Code: 0x%X", lErr);
        goto init_fail;
    }

    lErr = stateMachineStatesInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to register states! Code: 0x%X", lErr);
        goto init_fail;
    }

    lErr = stateMachineInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init state machine! Code: 0x%X", lErr);
        goto init_fail;
    }

    return ESP_OK;

init_fail:

    s_testBaseDeinit();

    return lErr;
}


void testRcSignalLossFsmSequence(void) {
    esp_err_t lErr = s_testBaseInit();

    if(lErr) {
        ESP_LOGE(TAG, "Failed to init test components. Code: 0x%X", lErr);
        return;
    }

    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 12;

    ESP_LOGW(TAG, "=== TEST: RC SIGNAL LOSS FSM ===");

    testPostEvent(SM_EVENT_SYSTEM_READY, 500);

    /*
     * Check POSITION_UNKNOWN.
     */
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);

    /* ============================================================
     * TEST 1: Signal loss while LOWERING
     * ============================================================ */

    /*
     * Establish known STOWED position.
     */
    testPostEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);

    /*
     * Begin lowering.
     *
     * Servo should begin extending and motion timeout should start.
     */
    testPostEvent(SM_EVENT_NOZZLE_EXTEND, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_LOWERING);

    /*
     * Simulate RC signal loss during movement.
     *
     * Expected:
     * LOWERING -> POSITION_UNKNOWN
     *
     * LOWERING.cbDeinit() should:
     *   - disable servo PWM output
     *   - stop motion timeout
     */
    testPostEvent(SM_EVENT_RC_SIGNAL_LOST, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);

    /* ============================================================
     * TEST 2: Signal loss while PUMPING
     * ============================================================ */

    /*
     * Establish known DEPLOYED position.
     */
    testPostEvent(SM_EVENT_LOWER_LIMIT_ACTIVE, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);

    /*
     * Start pump.
     */
    testPostEvent(SM_EVENT_PUMP_ON, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_PUMPING);

    /*
     * Signal loss while pumping.
     *
     * Expected:
     * PUMPING -> DEPLOYED
     *
     * PUMPING.cbDeinit() should physically turn the pump OFF.
     */
    testPostEvent(SM_EVENT_RC_SIGNAL_LOST, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);

    /* ============================================================
     * TEST 3: Signal loss while RAISING
     * ============================================================ */

    /*
     * Begin raising.
     *
     * Servo should retract and motion timeout should start.
     */
    testPostEvent(SM_EVENT_NOZZLE_RETRACT, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_RAISING);

    /*
     * Signal loss during movement.
     *
     * Expected:
     * RAISING -> POSITION_UNKNOWN
     *
     * Servo and motion timeout should both stop.
     */
    testPostEvent(SM_EVENT_RC_SIGNAL_LOST, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);

    /* ============================================================
     * TEST 4: Signal loss while FAULTED
     * ============================================================ */

    /*
     * Enter FAULT.
     */
    testPostEvent(SM_EVENT_FAULT, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);


    /*
     * RC signal loss must NEVER clear a fault.
     */
    testPostEvent(SM_EVENT_RC_SIGNAL_LOST, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_FAULT);


    /*
     * Recover normally using RESET.
     */
    testPostEvent(SM_EVENT_RESET, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);


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

    s_testBaseDeinit();

    ESP_LOGW(TAG, "=== END RC SIGNAL LOSS FSM TEST ===");
}


void testRcPumpIntegrationSequence(void) {
    esp_err_t lErr = s_testBaseInit();

    if(lErr) {
        ESP_LOGE(TAG, "Failed to init test components. Code: 0x%X", lErr);
        return;
    }

    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 10;

    ESP_LOGW(TAG, "=== TEST: RC PUMP INTEGRATION ===");
    ESP_LOGW(TAG, "Ensure RC PWM is connected");
    ESP_LOGW(TAG, "Set pump RC switch to OFF");

    /*
    * Move INIT -> POSITION_UNKNOWN before enabling external RC input.
    */
    testPostEvent(SM_EVENT_SYSTEM_READY, 500);

    lErr = rcInputInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init RC input. Code: 0x%X", lErr);

        goto test_cleanup;
    }

    /*
     * Give the RC input time to establish its initial OFF state.
     */
    vTaskDelay(pdMS_TO_TICKS(3000));


    /*
     * Test should begin in POSITION_UNKNOWN.
     */
    ubTestPassCount += testCheckState(STATE_MACHINE_POSITION_UNKNOWN);


    /* ============================================================
     * TEST 1: PUMP_ON must be rejected while STOWED
     * ============================================================ */

    /*
     * Establish STOWED position.
     */
    testPostEvent(SM_EVENT_UPPER_LIMIT_ACTIVE, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);

    /*
     * Physically move RC switch ON.
     *
     * rc_input should generate:
     *
     * RC_INPUT_STATE_LOW
     *      ↓
     * SM_EVENT_PUMP_ON
     *
     * But the FSM must reject PUMP_ON while STOWED.
     */
    ESP_LOGW(TAG, "MOVE PUMP RC SWITCH ON");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);

    /*
     * Pump must physically remain OFF.
     */

    /*
     * Return the RC switch OFF.
     *
     * This should generate SM_EVENT_PUMP_OFF, which is harmless
     * while STOWED.
     */
    ESP_LOGW(TAG, "MOVE PUMP RC SWITCH OFF");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount += testCheckState(STATE_MACHINE_STOWED);

    /* ============================================================
     * TEST 2: PUMP_ON while DEPLOYED
     * ============================================================ */

    /*
     * Begin normal lowering.
     */
    testPostEvent(SM_EVENT_NOZZLE_EXTEND,500);

    ubTestPassCount += testCheckState(STATE_MACHINE_LOWERING);

    /*
     * Simulate reaching the lower endpoint.
     */
    testPostEvent(SM_EVENT_LOWER_LIMIT_ACTIVE, 500);

    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);

    /*
     * Now physically switch ON.
     *
     * Expected:
     *
     * RC ON
     *     ↓
     * SM_EVENT_PUMP_ON
     *     ↓
     * DEPLOYED -> PUMPING
     *     ↓
     * pumpOn()
     */
    ESP_LOGW(TAG, "MOVE PUMP RC SWITCH ON");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount += testCheckState(STATE_MACHINE_PUMPING);

    /*
     * Pump should now physically be running.
     */

    /*
     * Leave the switch ON briefly.
     *
     * Duplicate suppression should prevent repeated PUMP_ON events.
     * The FSM should simply remain PUMPING.
     */
    ESP_LOGW(TAG, "KEEP SWITCH ON");
    vTaskDelay(pdMS_TO_TICKS(3000));

    ubTestPassCount += testCheckState(STATE_MACHINE_PUMPING);

    /*
     * Move switch OFF.
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
    ESP_LOGW(TAG, "MOVE PUMP RC SWITCH OFF");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ubTestPassCount += testCheckState(STATE_MACHINE_DEPLOYED);

    /*
     * Pump should now physically be OFF.
     */


    /*
     * Leave OFF briefly to verify no duplicate commands
     * cause any unexpected state changes.
     */
    ESP_LOGW(TAG, "KEEP SWITCH OFF");
    vTaskDelay(pdMS_TO_TICKS(3000));

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

    rcInputDeinit();
    s_testBaseDeinit();

    ESP_LOGW(TAG, "=== END RC PUMP INTEGRATION TEST ===");
}


void testRcNozzleIntegrationSequence(void) {
    esp_err_t lErr = s_testBaseInit();

    if(lErr) {
        ESP_LOGE(TAG, "Failed to init test components. Code: 0x%X", lErr);
        return;
    }

    uint8_t ubTestPassCount = 0;
    const uint8_t ubTestCount = 5;

    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, "RC NOZZLE INTEGRATION TEST");
    ESP_LOGW(TAG, "========================================");

    ESP_LOGW(TAG, "Prerequisites:");
    ESP_LOGW(TAG, " - Place nozzle at UPPER/STOWED limit");
    ESP_LOGW(TAG, " - Set nozzle RC switch to RETRACT");
    ESP_LOGW(TAG, " - Ensure transmitter and receiver are powered");

    vTaskDelay(pdMS_TO_TICKS(3000));

    testPostEvent(SM_EVENT_SYSTEM_READY, 500);

    lErr = limitSwitchInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init limit switches. Code: 0x%X", lErr);
        goto test_cleanup;
    }


    /* ============================================================
     * INITIAL STATE
     * ============================================================ */

    bool bReachedState = testWaitForState(STATE_MACHINE_STOWED, 5000);
    ubTestPassCount += bReachedState;

    if(!bReachedState) {
        ESP_LOGE(TAG, "Failed to reach STOWED state. Aborting test.");
        goto test_cleanup;
    }


    lErr = rcInputInit();
    if(lErr) {
        ESP_LOGE(TAG, "Failed to init RC input. Code: 0x%X", lErr);
        goto test_cleanup;
    }


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
    
     bReachedState = testWaitForState(STATE_MACHINE_LOWERING, 30000);
    ubTestPassCount += bReachedState;

    if(!bReachedState) {
        ESP_LOGE(TAG, "Failed to reach LOWERING state. Aborting test.");
        goto test_cleanup;
    }

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
    bReachedState = testWaitForState(STATE_MACHINE_DEPLOYED, 12000);
    ubTestPassCount += bReachedState;

    if(!bReachedState) {
        ESP_LOGE(TAG, "Failed to reach DEPLOYED state. Aborting test.");
        goto test_cleanup;
    }

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
    bReachedState = testWaitForState(STATE_MACHINE_RAISING, 30000);
    ubTestPassCount += bReachedState;

    if(!bReachedState) {
        ESP_LOGE(TAG, "Failed to reach RAISING state. Aborting test.");
        goto test_cleanup;
    }

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
    bReachedState = testWaitForState(STATE_MACHINE_STOWED, 12000);
    ubTestPassCount += bReachedState;

    if(!bReachedState) {
        ESP_LOGE(TAG, "Failed to reach STOWED state. Aborting test.");
        goto test_cleanup;
    }

    ESP_LOGW(TAG, "Nozzle successfully STOWED");


    /* ============================================================
     * RESULT
     * ============================================================ */

    ESP_LOGW(TAG, "========================================");

    if(ubTestPassCount == ubTestCount) {
        ESP_LOGI(TAG, "TEST PASSED: %u / %u checks passed",
            ubTestPassCount,
            ubTestCount
        );
    } else {
        ESP_LOGE(TAG, "TEST FAILED: %u / %u checks passed",
            ubTestPassCount,
            ubTestCount
        );
    }
    ESP_LOGW(TAG, "========================================");

test_cleanup:

    rcInputDeinit();
    limitSwitchDeinit();
    s_testBaseDeinit();

    
    ESP_LOGW(TAG, "=== END RC NOZZLE INTEGRATION TEST ===");
}

