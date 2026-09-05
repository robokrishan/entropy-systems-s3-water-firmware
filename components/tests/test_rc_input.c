#include "tests.h"
#include "esp_log.h"
#include "esp_err.h"

#include "rc_input.h"


static const char* TAG = "TEST_RC_INPUT";


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
    ubTestPassCount += s_checkState(STATE_MACHINE_POSITION_UNKNOWN);

    /*
     * Put the FSM into DEPLOYED using a synthetic endpoint event.
     */
    s_postTestEvent(SM_EVENT_LOWER_LIMIT_ACTIVE, 500);

    ubTestPassCount += s_checkState(STATE_MACHINE_DEPLOYED);

    /*
     * Enter PUMPING.
     *
     * This makes RC signal loss observable through an actual
     * FSM transition:
     *
     * PUMPING + RC_SIGNAL_LOST -> DEPLOYED
     */
    s_postTestEvent(SM_EVENT_PUMP_ON, 500);

    ubTestPassCount += s_checkState(STATE_MACHINE_PUMPING);

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
    ubTestPassCount += s_checkState(STATE_MACHINE_DEPLOYED);


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
    ubTestPassCount += s_checkState(STATE_MACHINE_DEPLOYED);


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

    ESP_LOGW(TAG, "=== END RC SIGNAL LOSS TEST ===");
}