#include "nozzle_servo.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "device_config.h"

#define NOZZLE_SERVO_STOP_US            1450
#define NOZZLE_SERVO_EXTEND_US          1800
#define NOZZLE_SERVO_RETRACT_US         1200

#define NOZZLE_SERVO_RESOLUTION_HZ      1000000
#define NOZZLE_SERVO_PERIOD_TICKS       20000


static const char* TAG = "nozzle_servo";


static mcpwm_timer_handle_t s_pTimer = NULL;
static mcpwm_oper_handle_t s_pOperator = NULL;
static mcpwm_cmpr_handle_t s_pComparator = NULL;
static mcpwm_gen_handle_t s_pGenerator = NULL;

static bool s_isInitialized = false;

static esp_err_t s_setPulseWidth(uint32_t ulPulseWidthUs) {
    esp_err_t lErr = ESP_OK;
    

    // check if nozzle servo initialized
    if(!s_isInitialized) {
        ESP_LOGE(TAG, "Nozzle servo not initialized. Cannot set pulse width!");
        lErr = ESP_ERR_INVALID_STATE;

        goto end_set_pulse_width;
    }


    lErr = mcpwm_comparator_set_compare_value(s_pComparator, ulPulseWidthUs);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to set pulse width to %u. Code: 0x%X", ulPulseWidthUs, lErr);
        goto end_set_pulse_width;
    }

    ESP_LOGI(TAG, "Set pulse width to %u", ulPulseWidthUs);

end_set_pulse_width:
    return lErr;
}


static void s_cleanup(void) {
    if (NULL != s_pGenerator) {
        mcpwm_generator_set_force_level(
            s_pGenerator,
            0,
            true
        );
    }

    if (NULL != s_pTimer) {
        mcpwm_timer_start_stop(
            s_pTimer,
            MCPWM_TIMER_STOP_EMPTY
        );

        mcpwm_timer_disable(
            s_pTimer
        );
    }

    if (NULL != s_pGenerator) {
        mcpwm_del_generator(s_pGenerator);
        s_pGenerator = NULL;
    }

    if (NULL != s_pComparator) {
        mcpwm_del_comparator(s_pComparator);
        s_pComparator = NULL;
    }

    if (NULL != s_pOperator) {
        mcpwm_del_operator(s_pOperator);
        s_pOperator = NULL;
    }

    if (NULL != s_pTimer) {
        mcpwm_del_timer(s_pTimer);
        s_pTimer = NULL;
    }
}


esp_err_t nozzleServoInit(void) {
    esp_err_t lErr = ESP_OK;

    if(NULL != s_pTimer) {
        ESP_LOGW(TAG, "Nozzle servo already initialized");
        lErr = ESP_ERR_INVALID_STATE;
        goto end_init;
    }


    // configure pwm timer
    mcpwm_timer_config_t sTimerConfig = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = NOZZLE_SERVO_RESOLUTION_HZ,
        .period_ticks = NOZZLE_SERVO_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP
    };

    lErr = mcpwm_new_timer(&sTimerConfig, &s_pTimer);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to create MCPWM timer: 0x%X", lErr);
        goto end_init;
    }


    // configure pwm operator
    mcpwm_operator_config_t sOperatorConfig = {
        .group_id = 0
    };

    lErr = mcpwm_new_operator(&sOperatorConfig, &s_pOperator);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to create MCPWM operator: 0x%X", lErr);
        goto end_init;
    }


    // connect operator to timer
    lErr = mcpwm_operator_connect_timer(s_pOperator, s_pTimer);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to connect operator to timer: 0x%X", lErr);
        goto end_init;
    }


    // configure pwm comparator
    mcpwm_comparator_config_t sComparatorConfig = {
        .flags.update_cmp_on_tez = true
    };

    lErr = mcpwm_new_comparator(s_pOperator, &sComparatorConfig, &s_pComparator);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to create MCPWM comparator: 0x%X", lErr);
        goto end_init;
    }


    // configure pwm generator
    mcpwm_generator_config_t sGeneratorConfig = {
        .gen_gpio_num = CONFIG_PIN_NOZZLE_SERVO
    };

    lErr = mcpwm_new_generator(s_pOperator, &sGeneratorConfig, &s_pGenerator);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to create MCPWM generator: 0x%X", lErr);
        goto end_init;
    }


    // define waveform
    lErr = mcpwm_generator_set_action_on_timer_event(
        s_pGenerator,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            MCPWM_TIMER_EVENT_EMPTY,
            MCPWM_GEN_ACTION_HIGH
        )
    );

    if(lErr) {
        ESP_LOGE(TAG, "Failed to set generator action on pulse start: 0x%X", lErr);
        goto end_init;
    }


    lErr = mcpwm_generator_set_action_on_compare_event(
        s_pGenerator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            s_pComparator,
            MCPWM_GEN_ACTION_LOW
        )
    );

    if(lErr) {
        ESP_LOGE(TAG, "Failed to set generator action on pulse comparator: 0x%X", lErr);
        goto end_init;
    }


    // set initial pulse to neutral command
    lErr = mcpwm_comparator_set_compare_value(s_pComparator, NOZZLE_SERVO_STOP_US);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to set initial pulse width: 0x%X", lErr);
        goto end_init;
    }


    // enable timer
    lErr = mcpwm_timer_enable(s_pTimer);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to enable timer: 0x%X", lErr);
        goto end_init;
    }


    // start timer
    lErr = mcpwm_timer_start_stop(s_pTimer, MCPWM_TIMER_START_NO_STOP);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to start timer: 0x%X", lErr);
        goto end_init;
    }

    s_isInitialized = true;

    ESP_LOGI(TAG, "Nozzle servo initialized on pin %d", CONFIG_PIN_NOZZLE_SERVO);

end_init:

    if(lErr) {
        s_cleanup();
    }

    return lErr;
}


void nozzleServoDeinit(void) {
    s_cleanup();
}

esp_err_t nozzleServoExtend(void) {
    esp_err_t lErr = ESP_OK;

    lErr = s_setPulseWidth(NOZZLE_SERVO_EXTEND_US);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to extend. Code: 0x%X", lErr);
    } else {
        ESP_LOGI(TAG, "Nozzle servo extending...");
    }

    return lErr;
}


esp_err_t nozzleServoRetract(void) {
    esp_err_t lErr = ESP_OK;

    lErr = s_setPulseWidth(NOZZLE_SERVO_RETRACT_US);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to retract. Code: 0x%X", lErr);
    } else {
        ESP_LOGI(TAG, "Nozzle servo retracting...");
    }

    return lErr;
}


esp_err_t nozzleServoStop(void) {
    esp_err_t lErr = ESP_OK;

    lErr = s_setPulseWidth(NOZZLE_SERVO_STOP_US);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to stop nozzle servo. Code: 0x%X", lErr);
    } else {
        ESP_LOGI(TAG, "Nozzle servo stopped.");
    }

    return lErr;
}