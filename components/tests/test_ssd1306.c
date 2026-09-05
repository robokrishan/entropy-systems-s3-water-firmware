#include "tests.h"
#include "esp_log.h"
#include "esp_err.h"

#include "i2c.h"
#include "ssd1306.h"


static const char* TAG = "TEST_SSD1306";



void testSsd1306Basic(void) {
    ESP_ERROR_CHECK(i2cBusInit());
    ESP_ERROR_CHECK(ina226Init());
    ESP_ERROR_CHECK(ssd1306Init());
}


void testSsd1306WriteText(void) {
    ESP_ERROR_CHECK(i2cBusInit());
    ESP_ERROR_CHECK(ssd1306Init());

    ESP_ERROR_CHECK(
        ssd1306WriteText(0, "WATER SAMPLER")
    );

    ESP_ERROR_CHECK(
        ssd1306WriteText(2, "OLED TEST")
    );

    ESP_ERROR_CHECK(
        ssd1306WriteText(4, "HELLO ESP32")
    );
}



