/*
 * lcd_blue.h
 *
 * 20 character 4 line I2C Display
 * Backpack Interface labelled "YwRobot Arduino LCM1602 IIC V1"
 *
 *  Created on: 12/03/2015
 *      Author: Victor
 */

#ifndef F5529___CC3000_WEBSERVER_I2C_LCD_LCD_BLUE_H_
#define F5529___CC3000_WEBSERVER_I2C_LCD_LCD_BLUE_H_

#include <inttypes.h>

#define LCD_ADDR 	0x27 // Endereço I2C
#define COLS		20
#define ROWS		4

/*!
 @defined
 @abstract   Enable bit of the LCD
 @discussion Defines the IO of the expander connected to the LCD Enable
 */
#define EN 2  // Enable bit

/*!
 @defined
 @abstract   Read/Write bit of the LCD
 @discussion Defines the IO of the expander connected to the LCD Rw pin
 */
#define RW 1  // Read/Write bit

/*!
 @defined
 @abstract   Register bit of the LCD
 @discussion Defines the IO of the expander connected to the LCD Register select pin
 */
#define RS 0  // Register select bit

/*!
 @defined
 @abstract   LCD dataline allocation this library only supports 4 bit LCD control
 mode.
 @discussion D4, D5, D6, D7 LCD data lines pin mapping of the extender module
 */
#define D4 4
#define D5 5
#define D6 6
#define D7 7

// PINO DO BACKLIGHT
#define BACKLIGHT_PIN 3

// flags for backlight control
/*!
 @defined
 @abstract   LCD_NOBACKLIGHT
 @discussion NO BACKLIGHT MASK
 */
#define LCD_NOBACKLIGHT 0x00

/*!
 @defined
 @abstract   LCD_BACKLIGHT
 @discussion BACKLIGHT MASK used when backlight is on
 */
#define LCD_BACKLIGHT   0xFF


uint8_t lcd_blue_detect(void); 		// Verificar a presença do LCD
void lcd_blue_config(void);			// Configuração do LCD

#endif /* F5529___CC3000_WEBSERVER_I2C_LCD_LCD_BLUE_H_ */
