/*
 * lcd16x2_i2c.h
 *
 *  Created on: Mar 28, 2020
 *      Author: Mohamed Yaqoob
 */

#ifndef LCD16X2_I2C_H_
#define LCD16X2_I2C_H_

#include "main.h"
#include <stdbool.h>

/* Function prototypes */
/**
 * @brief Initialise LCD16x2
 * @param[in] *pI2cHandle - pointer to HAL I2C handle
 */
bool lcd16x2_i2c_init(I2C_HandleTypeDef *pI2cHandle);

/**
 * @brief Set cursor position
 * @param[in] row - 0 or 1 for line1 or line2
 * @param[in] col - 0 - 15 (16 columns LCD)
 */
void lcd16x2_i2c_setCursor(uint8_t row, uint8_t col);
/**
 * @brief Move to beginning of 1st line
 */
void lcd16x2_i2c_1stLine(void);
/**
 * @brief Move to beginning of 2nd line
 */
void lcd16x2_i2c_2ndLine(void);

/**
 * @brief Select LCD Number of lines mode
 */
void lcd16x2_i2c_TwoLines(void);
void lcd16x2_i2c_OneLine(void);

/**
 * @brief Cursor ON/OFF
 */
void lcd16x2_i2c_cursorShow(bool state);

/**
 * @brief Display clear
 */
void lcd16x2_i2c_clear(void);

/**
 * @brief Display ON/OFF, to hide all characters, but not clear
 */
void lcd16x2_i2c_display(bool state);

/**
 * @brief Shift content to right
 */
void lcd16x2_i2c_shiftRight(uint8_t offset);

/**
 * @brief Shift content to left
 */
void lcd16x2_i2c_shiftLeft(uint8_t offset);

/**
 * @brief Print to display
 */
void lcd16x2_i2c_printf(const char* str, ...);

void lcd16x2_i2c_create_char(uint8_t location, uint8_t newChar[]);

void lcd_custom(unsigned char *Pattern, char Location);

void lcd16x2_i2c_print_custom_char(char customChar);

void lcd16x2_i2c_create_custom_chars();
void lcd16x2_i2c_create_init_custom_chars();

enum customChar {LIGHTBULB, THERMOMETER, DROP, CO2_1, CO2_2, CO2_3, PPM_1, PPM_2};
enum initCustomChar {CERO = 0, TWENTY = 1, FORTY = 2, SIXTY = 3, EIGHTY = 4, HUNDRED = 5};
enum charSize {CUSTOM_CHAR_ARRAY_SIZE = 8, CUSTOM_CHAR_ARRAY_BYTE_SIZE = 8, INIT_CUSTOM_CHAR_ARRAY_SIZE = 6};

//uint8_t initCustomChars[6][CUSTOM_CHAR_ARRAY_BYTE_SIZE];


#endif /* LCD16X2_I2C_H_ */