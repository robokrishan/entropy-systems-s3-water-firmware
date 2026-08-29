#include <stdio.h>
#include "pump.h"
#include "stdbool.h"
#include "device_config.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define PUMP_ON         1
#define PUMP_OFF        0

static const char* TAG = "pump";

static bool s_isInitialized = false;

esp_err_t pumpInit(void) {

    esp_err_t lErr = ESP_OK;

    if(s_isInitialized) {
        ESP_LOGW(TAG, "Pump already initialized.");
        lErr = ESP_ERR_INVALID_STATE;

        goto end_init;
    }

    lErr = gpio_set_level(CONFIG_PIN_PUMP, PUMP_OFF);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to set initial pump level. Code: 0x%X", lErr);
        goto end_init;
    }

    lErr = gpio_set_direction(CONFIG_PIN_PUMP, GPIO_MODE_OUTPUT);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to configure pump GPIO. Code: 0x%X", lErr);
        goto end_init;
    }

    s_isInitialized = true;

    ESP_LOGI(TAG, "Pump initialized on pin %d", CONFIG_PIN_PUMP);

end_init:

    return lErr;
}


esp_err_t pumpOn(void) {
    
    esp_err_t lErr = ESP_OK;

    if(!s_isInitialized) {
        ESP_LOGE(TAG, "Pump not inititalized");
        lErr = ESP_ERR_INVALID_STATE;

        goto end_pump_on;
    }

    lErr = gpio_set_level(CONFIG_PIN_PUMP, PUMP_ON);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to power on pump. Code: 0x%X", lErr);
        goto end_pump_on;
    }

    ESP_LOGI(TAG, "PUMP POWER ON");

end_pump_on:

    return lErr;
}


esp_err_t pumpOff(void) {
    
    esp_err_t lErr = ESP_OK;

    if(!s_isInitialized) {
        ESP_LOGE(TAG, "Pump not inititalized");
        lErr = ESP_ERR_INVALID_STATE;

        goto end_pump_off;
    }

    lErr = gpio_set_level(CONFIG_PIN_PUMP, PUMP_OFF);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to power off pump. Code: 0x%X", lErr);
        goto end_pump_off;
    }

    ESP_LOGI(TAG, "PUMP POWER OFF");

end_pump_off:

    return lErr;
}
