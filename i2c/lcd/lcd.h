/*
 * lcd.h
 *
 * Bilioteca para a implemetação básica do LCD
 *
 *  Created on: 13/03/2015
 *      Author: Victor
 */

#ifndef F5529___CC3000_WEBSERVER_I2C_LCD_LCD_H_
#define F5529___CC3000_WEBSERVER_I2C_LCD_LCD_H_

#include <inttypes.h>
#include <stdio.h> // for size_t

// LCD Commands
// ---------------------------------------------------------------------------
#define LCD_CLEARDISPLAY        0x01
#define LCD_RETURNHOME          0x02
#define LCD_ENTRYMODESET        0x04
#define LCD_DISPLAYCONTROL      0x08
#define LCD_CURSORSHIFT         0x10
#define LCD_FUNCTIONSET         0x20
#define LCD_SETCGRAMADDR        0x40
#define LCD_SETDDRAMADDR        0x80

// flags for display entry mode
// ---------------------------------------------------------------------------
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off and cursor control
// ---------------------------------------------------------------------------
#define LCD_DISPLAYON           0x04
#define LCD_DISPLAYOFF          0x00
#define LCD_CURSORON            0x02
#define LCD_CURSOROFF           0x00
#define LCD_BLINKON             0x01
#define LCD_BLINKOFF            0x00

// flags for display/cursor shift
// ---------------------------------------------------------------------------
#define LCD_DISPLAYMOVE         0x08
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x04
#define LCD_MOVELEFT            0x00

// flags for function set
// ---------------------------------------------------------------------------
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x04
#define LCD_5x8DOTS             0x00


// Define COMMAND and DATA LCD Rs (used by send method).
// ---------------------------------------------------------------------------
#define COMMAND                 0
#define DATA                    1
#define FOUR_BITS               2


/*!
 @defined
 @abstract   Defines the duration of the home and clear commands
 @discussion This constant defines the time it takes for the home and clear
 commands in the LCD - Time in microseconds.
 */
#define HOME_CLEAR_EXEC      2000

/*!
    @defined
    @abstract   Backlight off constant declaration
    @discussion Used in combination with the setBacklight to swith off the
 LCD backlight. @set setBacklight
*/
#define BACKLIGHT_OFF           0

/*!
 @defined
 @abstract   Backlight on constant declaration
 @discussion Used in combination with the setBacklight to swith on the
 LCD backlight. @set setBacklight
 */
#define BACKLIGHT_ON          255

/*!
 @typedef
 @abstract   Define backlight control polarity
 @discussion Backlight control polarity. @see setBacklightPin.
 */
#define POSITIVE 0
#define NEGATIVE 1


// Attach functions
void lcd_attachSendFunction(void (*function)(uint8_t, uint8_t));
void lcd_attachSetBacklightFunction(void (*function)(uint8_t));

/*!
 @function
 @abstract   LCD initialization.
 @discussion Initializes the LCD to a given size (col, row). This methods
 initializes the LCD, therefore, it MUST be called prior to using any other
 method from this class.

 This method is abstract, a base implementation is available common to all LCD
 drivers. Should it not be compatible with some other LCD driver, a derived
 implementation should be done on the driver specif class.

 @param      cols[in] the number of columns that the display has
 @param      rows[in] the number of rows that the display has
 @param      charsize[in] character size, default==LCD_5x8DOTS
 */
void lcd_init(uint8_t cols, uint8_t lines, uint8_t dotsize);

/*!
 @function
 @abstract   Clears the LCD.
 @discussion Clears the LCD screen and positions the cursor in the upper-left
 corner.

 This operation is time consuming for the LCD.

 @param      none
 */
void lcd_clear(void);

/*!
 @function
 @abstract   Sets the cursor to the upper-left corner.
 @discussion Positions the cursor in the upper-left of the LCD.
 That is, use that location in outputting subsequent text to the display.
 To also clear the display, use the clear() function instead.

 This operation is time consuming for the LCD.

 @param      none
 */
void lcd_home(void);

/*!
 @function
 @abstract   Turns off the LCD display.
 @discussion Turns off the LCD display, without losing the text currently
 being displayed on it.

 @param      none
 */
void lcd_noDisplay(void);

/*!
 @function
 @abstract   Turns on the LCD display.
 @discussion Turns on the LCD display, after it's been turned off with
 noDisplay(). This will restore the text (and cursor location) that was on
 the display prior to calling noDisplay().

 @param      none
 */
void lcd_display(void);

/*!
 @function
 @abstract   Turns off the blinking of the LCD cursor.

 @param      none
 */
void lcd_noBlink(void);

/*!
 @function
 @abstract   Display the cursor of the LCD.
 @discussion Display the blinking LCD cursor. If used in combination with
 the cursor() function, the result will depend on the particular display.

 @param      none
 */
void lcd_blink(void);

/*!
 @function
 @abstract   Hides the LCD cursor.

 @param      none
 */
void lcd_noCursor(void);

/*!
 @function
 @abstract   Display the LCD cursor.
 @discussion Display the LCD cursor: an underscore (line) at the location
 where the next character will be written.

 @param      none
 */
void lcd_cursor(void);

/*!
 @function
 @abstract   Scrolls the contents of the display (text and cursor) one space
 to the left.

 @param      none
 */
void lcd_scrollDisplayLeft(void);

/*!
 @function
 @abstract   Scrolls the contents of the display (text and cursor) one space
 to the right.

 @param      none
 */
void lcd_scrollDisplayRight(void);

/*!
 @function
 @abstract   Set the direction for text written to the LCD to left-to-right.
 @discussion Set the direction for text written to the LCD to left-to-right.
 All subsequent characters written to the display will go from left to right,
 but does not affect previously-output text.

 This is the default configuration.

 @param      none
 */
void lcd_leftToRight(void);

/*!
 @function
 @abstract   Set the direction for text written to the LCD to right-to-left.
 @discussion Set the direction for text written to the LCD to right-to-left.
 All subsequent characters written to the display will go from right to left,
 but does not affect previously-output text.

 left-to-right is the default configuration.

 @param      none
 */
void lcd_rightToLeft(void);

/*!
 @function
 @abstract   Moves the cursor one space to the left.
 @discussion
 @param      none
 */
void lcd_moveCursorLeft(void);

/*!
 @function
 @abstract   Moves the cursor one space to the right.

 @param      none
 */
void lcd_moveCursorRight(void);

/*!
 @function
 @abstract   Turns on automatic scrolling of the LCD.
 @discussion Turns on automatic scrolling of the LCD. This causes each
 character output to the display to push previous characters over by one
 space. If the current text direction is left-to-right (the default),
 the display scrolls to the left; if the current direction is right-to-left,
 the display scrolls to the right.
 This has the effect of outputting each new character to the same location on
 the LCD.

 @param      none
 */
void lcd_autoscroll(void);

/*!
 @function
 @abstract   Turns off automatic scrolling of the LCD.
 @discussion Turns off automatic scrolling of the LCD, this is the default
 configuration of the LCD.

 @param      none
 */
void lcd_noAutoscroll(void);

/*!
 @function
 @abstract   Creates a custom character for use on the LCD.
 @discussion Create a custom character (glyph) for use on the LCD.
 Most chipsets only support up to eight characters of 5x8 pixels. Therefore,
 this methods has been limited to locations (numbered 0 to 7).

 The appearance of each custom character is specified by an array of eight
 bytes, one for each row. The five least significant bits of each byte
 determine the pixels in that row. To display a custom character on screen,
 write()/print() its number, i.e. lcd.print (char(x)); // Where x is 0..7.

 @param      location[in] LCD memory location of the character to create
 (0 to 7)
 @param      charmap[in] the bitmap array representing each row of the character.
 */
void lcd_createChar(uint8_t location, uint8_t charmap[]);

/*!
 @function
 @abstract   Position the LCD cursor.
 @discussion Sets the position of the LCD cursor. Set the location at which
 subsequent text written to the LCD will be displayed.

 @param      col[in] LCD column
 @param      row[in] LCD row - line.
 */
void lcd_setCursor(uint8_t col, uint8_t row);

/*!
 @function
 @abstract   Switch-on the LCD backlight.
 @discussion Switch-on the LCD backlight.
 The setBacklightPin has to be called before setting the backlight for
 this method to work. @see setBacklightPin.
 */
void lcd_backlight(void);

/*!
 @function
 @abstract   Switch-off the LCD backlight.
 @discussion Switch-off the LCD backlight.
 The setBacklightPin has to be called before setting the backlight for
 this method to work. @see setBacklightPin.
 */
void lcd_noBacklight(void);

/*!
 @function
 @abstract   Switch on the LCD module.
 @discussion Switch on the LCD module, it will switch on the LCD controller
 and the backlight. This method has the same effect of calling display and
 backlight. @see display, @see backlight
 */
void lcd_on(void);

/*!
 @function
 @abstract   Switch off the LCD module.
 @discussion Switch off the LCD module, it will switch off the LCD controller
 and the backlight. This method has the same effect of calling noDisplay and
 noBacklight. @see display, @see backlight
 */
void lcd_off(void);

void lcd_print(char*);


/*!
 @function
 @abstract   Send a command to the LCD.
 @discussion This method sends a command to the LCD by setting the Register
 select line of the LCD.

 This command shouldn't be used to drive the LCD, only to implement any other
 feature that is not available on this library.

 @param      value[in] Command value to send to the LCD (COMMAND, DATA or
 FOUR_BITS).
 */
void lcd_command(uint8_t value);

/*!
 @function
 @abstract   Writes to the LCD.
 @discussion This method writes character to the LCD in the current cursor
 position.

 This is the virtual write method, implemented in the Print class, therefore
 all Print class methods will end up calling this method.

 @param      value[in] Value to write to the LCD.
 */
void lcd_write(uint8_t value);

#endif /* F5529___CC3000_WEBSERVER_I2C_LCD_LCD_H_ */
