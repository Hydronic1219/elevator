/*
 * lcd.h
 *
 *  Created on: Jun 18, 2025
 *      Author: user21
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "main.h"
#include "string.h"

/* LCD는 4bit 통신
 *
 */
#define I2C_LCD_ADDRESS		(0x27<<1)
#define BACKLIGHT_ON		0x08

/*lcd command*/
#define  DISPLAY_ON 		0x0C
#define  DISPLAY_OFF		0x08
#define  CLEAR_DISPLAY		0x01
#define  RETURN_HOME		0x02


void lcdCommand(uint8_t command);
void lcdData(uint8_t);
void i2cLcd_Init();
void lcdString(char *str);
void moveCursor(uint8_t row, uint8_t col);

#endif /* INC_LCD_H_ */
