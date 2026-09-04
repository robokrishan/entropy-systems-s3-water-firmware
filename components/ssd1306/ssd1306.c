#include "ssd1306.h"
#include "i2c.h"
#include "font_table.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"


/*        I2C Configuration        */
#define SSD1306_I2C_ADDRESS         0x3C
#define SSD1306_I2C_SPEED_HZ        400000


/*        I2C Control Bytes         */
#define SSD1306_CONTROL_COMMAND     0x00
#define SSD1306_CONTROL_DATA        0x40


static const char* TAG = "ssd1306";

static i2c_master_dev_handle_t s_pDeviceHandle = NULL;
static bool s_isInitialized = false;


/*          Helpers         */
static esp_err_t s_sendCommands(const uint8_t* pCommands, size_t ulCommandCount) {

    // check for invalid arguments
    if(NULL == pCommands || 0 == ulCommandCount) {
        return ESP_ERR_INVALID_ARG;
    }

    // check if i2c initialized for component
    if(NULL == s_pDeviceHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    // Command structure is one control byte followed by
    // command bytes
    uint8_t ubData[32] = {0};

    if((ulCommandCount + 1) > sizeof(ubData)) {
        return ESP_ERR_INVALID_SIZE;
    }

    // first byte is control byte
    ubData[0] = SSD1306_CONTROL_COMMAND;

    // rest of the structure is commands
    memcpy(&ubData[1], pCommands, ulCommandCount);

    return i2cBusWrite(s_pDeviceHandle, ubData, ulCommandCount+1);
}


static esp_err_t s_clearDisplay(void) {

    const uint8_t ubAddressCommands[] = {
        0x21, 0x00, 0x7F, 0x22, 0x00, 0x07
    };

    esp_err_t lErr = s_sendCommands(ubAddressCommands, sizeof(ubAddressCommands));
    if(lErr) {
        return lErr;
    }

    uint8_t ubData[17] = {0};
    
    ubData[0] = SSD1306_CONTROL_DATA;

    const uint16_t uwDisplayBytes = SSD1306_WIDTH * SSD1306_PAGE_COUNT;
    const uint8_t ubPayloadSize = sizeof(ubData) - 1;

    for(uint16_t uwOffset = 0; uwOffset < uwDisplayBytes; uwOffset += ubPayloadSize) {
        lErr = i2cBusWrite(s_pDeviceHandle, ubData, sizeof(ubData));
        if(lErr) {
            ESP_LOGE(TAG, "Failed to clear display. Code: 0x%X", lErr);
            
            return lErr;
        }
    }

    return ESP_OK;
}


static esp_err_t s_sendData(const uint8_t* pData, size_t ulDataSize) {
    if(NULL == pData || 0 == ulDataSize) {
        return ESP_ERR_INVALID_ARG;
    }

    if(NULL == s_pDeviceHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t ubBuffer[SSD1306_WIDTH + 1] = {0};

    if((ulDataSize + 1) > sizeof(ubBuffer)) {
        return ESP_ERR_INVALID_SIZE;
    }

    ubBuffer[0] = SSD1306_CONTROL_DATA;

    memcpy(&ubBuffer[1], pData, ulDataSize);

    return i2cBusWrite(s_pDeviceHandle, ubBuffer, ulDataSize + 1);
}


static esp_err_t s_setRow(uint8_t ubRow) {
    if(ubRow >= SSD1306_TEXT_ROWS) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t ubCommands[] = {
        0x21,               // Set column address
        0x00,               // Start column
        SSD1306_WIDTH - 1,  // End column

        0x22,               // Set page address
        ubRow,              // Start page
        ubRow               // End page
    };

    return s_sendCommands(ubCommands, sizeof(ubCommands));
}


/*      Public API      */
esp_err_t ssd1306Init(void) {
    if(s_isInitialized) {
        ESP_LOGW(TAG, "OLED already initialized!");

        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t lErr = i2cBusAddDevice(SSD1306_I2C_ADDRESS, SSD1306_I2C_SPEED_HZ, &s_pDeviceHandle);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to add SSD1306 to I2C bus. Code: 0x%X", lErr);

        return lErr;
    }

    // initialization sequence
    const uint8_t ubInitCommands[] = {
        0xAE,           // Display OFF

        0xD5, 0x80,     // Display clock divide ratio
        0xA8, 0x3F,     // Multiplex ratio: 1/64
        0xD3, 0x00,     // Display offset
        0x40,           // Display start line

        0x8D, 0x14,     // Enable charge pump

        0x20, 0x00,     // Horizontal addressing mode

        0xA1,           // Segment remap
        0xC8,           // COM output scan direction

        0xDA, 0x12,     // COM pins configuration
        0x81, 0x7F,     // Contrast

        0xD9, 0xF1,     // Pre-charge period
        0xDB, 0x40,     // VCOMH deselect level

        0xA4,           // Resume RAM display
        0xA6            // Normal display
    };


    lErr = s_sendCommands(ubInitCommands, sizeof(ubInitCommands));
    if(lErr) {
        ESP_LOGE(TAG, "Failed to configure SSD1306. Code: 0x%X", lErr);

        return lErr;
    }

    lErr = s_clearDisplay();
    if(lErr) {
        return lErr;
    }

    const uint8_t ubDisplayOn = 0xAF;
    
    lErr = s_sendCommands(&ubDisplayOn, sizeof(ubDisplayOn));
    if(lErr) {
        ESP_LOGE(TAG, "Failed to enable display. Code: 0x%X", lErr);

        return lErr;
    }

    s_isInitialized = true;

    ESP_LOGI(TAG, "SSD1306 initialized");

    return ESP_OK;
}


esp_err_t ssd1306Clear(void) {
    if(!s_isInitialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return s_clearDisplay();
}


esp_err_t ssd1306WriteText(uint8_t ubRow, const char* pText) {
    if(!s_isInitialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if(NULL == pText) {
        return ESP_ERR_INVALID_ARG;
    }

    if(ubRow >= SSD1306_TEXT_ROWS) {
        return ESP_ERR_INVALID_ARG;
    }


    // start with blank row
    uint8_t ubRowData[SSD1306_WIDTH] = {0};

    uint8_t ubColumn = 0;
    uint8_t ubCharacterCount = 0;

    while('\0' != *pText && ubCharacterCount < SSD1306_MAX_CHARS_PER_ROW) {
        uint8_t ubCharacter = (uint8_t)*pText;

        // replace unsupported characters with '?'
        if(ubCharacter < 0x20 || ubCharacter > 0x7E) {
            ubCharacter = '?';
        }

        uint8_t ubFontIndex = ubCharacter - 0x20;

        // copy 5 char columns
        for(uint8_t ubCol = 0; ubCol < SSD1306_FONT_WIDTH; ubCol++) {
            ubRowData[ubColumn++] = s_ubFont5x7[ubFontIndex][ubCol];
        }

        // add blank col between chars
        ubRowData[ubColumn++] = 0x00;
        pText++;
        ubCharacterCount++;
    }

    esp_err_t lErr = s_setRow(ubRow);
    if(lErr) {
        ESP_LOGE(TAG, "Failed to set display row %u. Code: 0x%X", ubRow, lErr);

        return lErr;
    }

    lErr = s_sendData(ubRowData, sizeof(ubRowData));
    if(lErr) {
        ESP_LOGE(TAG, "Failed to write display text. Code: 0x%X", lErr);

        return lErr;
    }

    return ESP_OK;
}
