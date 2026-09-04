#ifndef COMPONENTS_SSD1306_H_
#define COMPONENTS_SSD1306_H_

#include "esp_err.h"

/*      Display Dimensions      */
#define SSD1306_WIDTH               128
#define SSD1306_HEIGHT              64


/*        Display Configuration     */
#define SSD1306_PAGE_COUNT          (SSD1306_HEIGHT/8)
#define SSD1306_TEXT_ROWS           8
#define SSD1306_FONT_WIDTH          5
#define SSD1306_CHAR_WIDTH          6
#define SSD1306_MAX_CHARS_PER_ROW   (SSD1306_WIDTH / SSD1306_CHAR_WIDTH)


/**
 * @brief Initialize the SSD1306 OLED display.
 *
 * Registers the SSD1306 device on the I2C bus, configures the display,
 * and clears the display memory.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the SSD1306 is already initialized
 *      - Appropriate esp_err_t error code on I2C communication failure
 */
esp_err_t ssd1306Init(void);


/**
 * @brief Clear the entire OLED display.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the SSD1306 is not initialized
 *      - Appropriate esp_err_t error code on I2C communication failure
 */
esp_err_t ssd1306Clear(void);


/**
 * @brief Write text to a row of the OLED display.
 *
 * The SSD1306 128x64 display is divided into eight text rows when using
 * an 8-pixel-high font. Row 0 is the top row and row 7 is the bottom row.
 *
 * @param[in] ubRow Row on which to display the text, from 0 to 7.
 * @param[in] pText Null-terminated string to display.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the SSD1306 is not initialized
 *      - ESP_ERR_INVALID_ARG if the row is invalid or pText is NULL
 *      - Appropriate esp_err_t error code on I2C communication failure
 */
esp_err_t ssd1306WriteText(uint8_t ubRow, const char* pText);

#endif /* COMPONENTS_SSD1306_H_ */