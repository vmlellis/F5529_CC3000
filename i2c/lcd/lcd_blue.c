/*
 * lcd_blue.c
 *
 *  Created on: 12/03/2015
 *      Author: Victor
 */


#include "lcd_blue.h"
#include "lcd.h"
#include "../twi_master.h"

extern uint8_t lcd_displayfunction;
extern uint8_t lcd_polarity;

static uint8_t backlightPinMask;
static uint8_t backlightStsMask;
static uint8_t en;
static uint8_t rw;
static uint8_t rs;
static uint8_t data_pins[4];


uint8_t lcd_blue_detect(void) {
	uint8_t bytesRead = twi_master_readFrom(LCD_ADDR, 0, 1, 1);
	return (bytesRead);
}

void lcd_blue_write( uint8_t value ) {
	twi_master_writeTo(LCD_ADDR, &value, sizeof(value), 1);
}

/*
 * write4bits
 */
void lcd_blue_write4bits ( uint8_t value, uint8_t mode )  {
	uint8_t data = 0, i;

	// Map the value to LCD pin mapping
	// --------------------------------
	for (i = 0; i < 4; i++)
	{
	  if ( ( value & 0x1 ) == 1 )
	  {
		 data |= data_pins[i];
	  }
	  value = ( value >> 1 );
	}

	// Is it a command or data
	// -----------------------
	if ( mode == DATA )
	{
	  mode = rs;
	}

	data |= mode | backlightStsMask;

	lcd_blue_write(data | en);    // En HIGH
	lcd_blue_write(data & ~en);   // En LOW
}

/*
 * send - write either command or data
 */
void lcd_blue_send(uint8_t value, uint8_t mode)
{
   if ( mode == FOUR_BITS )
   {
	   lcd_blue_write4bits( (value & 0x0F), COMMAND );
   }
   else
   {
	   lcd_blue_write4bits( (value >> 4), mode );
	   lcd_blue_write4bits( (value & 0x0F), mode);
   }
}

void lcd_blue_setBacklight(uint8_t value)  {
	// Check for polarity to configure mask accordingly
	// ----------------------------------------------------------
	if  (((lcd_polarity == POSITIVE) && (value > 0)) ||
	   ((lcd_polarity == NEGATIVE ) && ( value == 0 )))
	{
		backlightStsMask = backlightPinMask & LCD_BACKLIGHT;
	}
	else
	{
		backlightStsMask = backlightPinMask & LCD_NOBACKLIGHT;
	}
	lcd_blue_write(backlightStsMask);
}

void lcd_blue_config(void) {
	backlightPinMask = ( 1 << BACKLIGHT_PIN );
	backlightStsMask = LCD_NOBACKLIGHT;
	lcd_polarity = POSITIVE;
	en = ( 1 << EN );
    rw = ( 1 << RW );
    rs = ( 1 << RS );

    // Initialise pin mapping
	data_pins[0] = ( 1 << D4 );
	data_pins[1] = ( 1 << D5 );
	data_pins[2] = ( 1 << D6 );
	data_pins[3] = ( 1 << D7 );

	lcd_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
	lcd_blue_write(0); // Set the entire port to LOW

	lcd_attachSendFunction(lcd_blue_send);
	lcd_attachSetBacklightFunction(lcd_blue_setBacklight);

	lcd_init(COLS, ROWS, 0);
}


