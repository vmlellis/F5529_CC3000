/*
 * lcd.c
 *
 *  Created on: 13/03/2015
 *      Author: Victor
 */

#include "lcd.h"
#include "../../setup.h"

// Internal LCD variables to control the LCD shared between all derived
// classes.
uint8_t lcd_displayfunction;  		 // LCD_5x10DOTS or LCD_5x8DOTS, LCD_4BITMODE or
						  	         // LCD_8BITMODE, LCD_1LINE or LCD_2LINE
static uint8_t lcd_displaycontrol;   // LCD base control command LCD on/off, blink, cursor
						  	         // all commands are "ored" to its contents.
static uint8_t lcd_displaymode;      // Text entry mode to the LCD
static uint8_t lcd_numlines;         // Number of lines of the LCD, initialized with begin()
static uint8_t lcd_cols;             // Number of columns in the LCD
uint8_t lcd_polarity;   	         // Backlight polarity



static void (*lcd_send)(uint8_t value, uint8_t mode);
static void (*lcd_setBacklight)(uint8_t value);

void lcd_attachSendFunction(void (*function)(uint8_t, uint8_t)) {
	lcd_send = function;
}

void lcd_attachSetBacklightFunction(void (*function)(uint8_t)) {
	lcd_setBacklight = function;
}

void lcd_init(uint8_t cols, uint8_t lines, uint8_t dotsize)  {

	if (lines > 1)
	{
	  lcd_displayfunction |= LCD_2LINE;
	}
	lcd_numlines = lines;
	lcd_cols = cols;

	// for some 1 line displays you can select a 10 pixel high font
	// ------------------------------------------------------------
	if ((dotsize != LCD_5x8DOTS) && (lines == 1))
	{
	  lcd_displayfunction |= LCD_5x10DOTS;
	}

	// SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
	// according to datasheet, we need at least 40ms after power rises above 2.7V
	// before sending commands. Arduino can turn on way before 4.5V so we'll wait
	// 50
	// ---------------------------------------------------------------------------
	delay (100); // 100ms delay

	//put the LCD into 4 bit or 8 bit mode
	// -------------------------------------
	if (! (lcd_displayfunction & LCD_8BITMODE))
	{
	  // this is according to the hitachi HD44780 datasheet
	  // figure 24, pg 46

	  // we start in 8bit mode, try to set 4 bit mode
	  lcd_send(0x03, FOUR_BITS);
	  delayMicroseconds(4500); // wait min 4.1ms

	  // second try
	  lcd_send( 0x03,FOUR_BITS);
	  delayMicroseconds(4500); // wait min 4.1ms

	  // third go!
	  lcd_send(0x03, FOUR_BITS);
	  delayMicroseconds(150);

	  // finally, set to 4-bit interface
	  lcd_send(0x02, FOUR_BITS);
	}
	else
	{
	  // this is according to the hitachi HD44780 datasheet
	  // page 45 figure 23

	  // Send function set command sequence
	  lcd_command(LCD_FUNCTIONSET | lcd_displayfunction);
	  delayMicroseconds(4500);  // wait more than 4.1ms

	  // second try
	  lcd_command(LCD_FUNCTIONSET | lcd_displayfunction);
	  delayMicroseconds(150);

	  // third go
	  lcd_command(LCD_FUNCTIONSET | lcd_displayfunction);
	}

	// finally, set # lines, font size, etc.
	lcd_command(LCD_FUNCTIONSET | lcd_displayfunction);

	// turn the display on with no cursor or blinking default
	lcd_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	lcd_display();

	// clear the LCD
	lcd_clear();

	// Initialize to default text direction (for roman languages)
	lcd_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	// set the entry mode
	lcd_command(LCD_ENTRYMODESET | lcd_displaymode);

	lcd_backlight();
}

// Common LCD Commands
// ---------------------------------------------------------------------------
void lcd_clear()
{
   lcd_command(LCD_CLEARDISPLAY);         // clear display, set cursor position to zero
   delayMicroseconds(HOME_CLEAR_EXEC);    // this command is time consuming
}

void lcd_home()
{
	lcd_command(LCD_RETURNHOME);          // set cursor position to zero
   delayMicroseconds(HOME_CLEAR_EXEC); 	  // This command is time consuming
}

void lcd_setCursor(uint8_t col, uint8_t row)
{
   const uint8_t row_offsetsDef[]   = { 0x00, 0x40, 0x14, 0x54 }; // For regular LCDs
   const uint8_t row_offsetsLarge[] = { 0x00, 0x40, 0x10, 0x50 }; // For 16x4 LCDs

   if ( row >= lcd_numlines )
   {
      row = lcd_numlines-1;    // rows start at 0
   }

   // 16x4 LCDs have special memory map layout
   // ----------------------------------------
   if ( lcd_cols == 16 && lcd_numlines == 4 )
   {
      lcd_command(LCD_SETDDRAMADDR | (col + row_offsetsLarge[row]));
   }
   else
   {
      lcd_command(LCD_SETDDRAMADDR | (col + row_offsetsDef[row]));
   }
}

// Turn the display on/off
void lcd_noDisplay()
{
   lcd_displaycontrol &= ~LCD_DISPLAYON;
   lcd_command(LCD_DISPLAYCONTROL | lcd_displaycontrol);
}

void lcd_display()
{
   lcd_displaycontrol |= LCD_DISPLAYON;
   lcd_command(LCD_DISPLAYCONTROL | lcd_displaycontrol);
}

// Turns the underline cursor on/off
void lcd_noCursor()
{
	lcd_displaycontrol &= ~LCD_CURSORON;
	lcd_command(LCD_DISPLAYCONTROL | lcd_displaycontrol);
}

void lcd_cursor()
{
	lcd_displaycontrol |= LCD_CURSORON;
	lcd_command(LCD_DISPLAYCONTROL | lcd_displaycontrol);
}

// Turns on/off the blinking cursor
void lcd_noBlink()
{
	lcd_displaycontrol &= ~LCD_BLINKON;
	lcd_command(LCD_DISPLAYCONTROL | lcd_displaycontrol);
}

void lcd_blink()
{
	lcd_displaycontrol |= LCD_BLINKON;
	lcd_command(LCD_DISPLAYCONTROL | lcd_displaycontrol);
}

// This is for text that flows Left to Right
void lcd_leftToRight(void)
{
   lcd_displaymode |= LCD_ENTRYLEFT;
   lcd_command(LCD_ENTRYMODESET | lcd_displaymode);
}

// This is for text that flows Right to Left
void lcd_rightToLeft(void)
{
   lcd_displaymode &= ~LCD_ENTRYLEFT;
   lcd_command(LCD_ENTRYMODESET | lcd_displaymode);
}

// This method moves the cursor one space to the right
void lcd_moveCursorRight(void)
{
   lcd_command(LCD_CURSORSHIFT | LCD_CURSORMOVE | LCD_MOVERIGHT);
}

// This method moves the cursor one space to the left
void lcd_moveCursorLeft(void)
{
   lcd_command(LCD_CURSORSHIFT | LCD_CURSORMOVE | LCD_MOVELEFT);
}


// This will 'right justify' text from the cursor
void lcd_autoscroll(void)
{
   lcd_displaymode |= LCD_ENTRYSHIFTINCREMENT;
   lcd_command(LCD_ENTRYMODESET | lcd_displaymode);
}

// This will 'left justify' text from the cursor
void lcd_noAutoscroll(void)
{
   lcd_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
   lcd_command(LCD_ENTRYMODESET | lcd_displaymode);
}

// Write to CGRAM of new characters
void lcd_createChar(uint8_t location, uint8_t charmap[])
{
	int i;

	location &= 0x7;            // we only have 8 locations 0-7

	lcd_command(LCD_SETCGRAMADDR | (location << 3));
	delayMicroseconds(30);

	for (i=0; i<8; i++)
	{
	  lcd_write(charmap[i]);      // call the virtual write method
	  delayMicroseconds(40);
	}
}

//
// Switch on the backlight
void lcd_backlight ( void )
{
   lcd_setBacklight(255);
}

//
// Switch off the backlight
void lcd_noBacklight ( void )
{
   lcd_setBacklight(0);
}

//
// Switch fully on the LCD (backlight and LCD)
void lcd_on ( void )
{
   lcd_display();
   lcd_backlight();
}

//
// Switch fully off the LCD (backlight and LCD)
void lcd_off ( void )
{
   lcd_noBacklight();
   lcd_noDisplay();
}

void lcd_print(char *s)
{
	char c;

  	// Loops through each character in string 's'
	while ((c = *s++)) {
	  lcd_write(c);
	}
}

// General LCD commands - generic methods used by the rest of the commands
// ---------------------------------------------------------------------------
void lcd_command(uint8_t value)
{
   lcd_send(value, COMMAND);
}


void lcd_write(uint8_t value)
{
   lcd_send(value, DATA);
}
