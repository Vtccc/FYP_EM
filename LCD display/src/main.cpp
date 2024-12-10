
/**********************************************************************************
 *
 * Interfacing Arduino with ST7565 Monochrome LCD (128x64 Pixel)
 * This is a free software with NO WARRANTY - Use it at your own risk!
 * https://simple-circuit.com/
 *
***********************************************************************************
 Written by Limor Fried/Ladyada for Adafruit Industries,
 with contributions from the open source community.
 BSD license, check license.txt for more information
 All text above, and the splash screen below must be
 included in any redistribution.
************************************************************************************
 Modified to work with ST7565 Monochrome LCD.
 https://simple-circuit.com/
 **********************************************************************************/

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <ST7565_LCD.h>

// ST7565 LCD connection with Arduino board using software SPI
#define LCD_DIN 12
#define LCD_SCLK 11
#define LCD_A0 16
#define LCD_RESET 3
#define LCD_CS 8
ST7565_LCD display = ST7565_LCD(LCD_DIN, LCD_SCLK, LCD_A0, LCD_RESET, LCD_CS);

/*/ Comment out above, uncomment this block to use hardware SPI
// connect LCD 'DIN' & 'SCLK' to board's hardware SPI pins
#define LCD_A0     16
#define LCD_RESET  3
#define LCD_CS     8
ST7565 display = ST7565(LCD_A0, LCD_RESET, LCD_CS);
*/

#define NUMFLAKES 10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16
static const unsigned char PROGMEM logo_bmp[] =
	{0b00000000, 0b11000000,
	 0b00000001, 0b11000000,
	 0b00000001, 0b11000000,
	 0b00000011, 0b11100000,
	 0b11110011, 0b11100000,
	 0b11111110, 0b11111000,
	 0b01111110, 0b11111111,
	 0b00110011, 0b10011111,
	 0b00011111, 0b11111100,
	 0b00001101, 0b01110000,
	 0b00011011, 0b10100000,
	 0b00111111, 0b11100000,
	 0b00111111, 0b11110000,
	 0b01111100, 0b11110000,
	 0b01110000, 0b01110000,
	 0b00000000, 0b00110000};

void testscrolldisplay()
{
	Serial.println(F("Scrolling Display Test"));
	// scroll right
	for (uint8_t scroll = 0; scroll < display.width(); scroll++)
	{
		display.scrollRight(1); // scroll the screen to the right by '1' pixel
		display.display();
		delay(10); // wait 10 ms
	}
	delay(1000); // wait for 1 second
	// scroll left
	for (uint8_t scroll = 0; scroll < display.width(); scroll++)
	{
		display.scrollLeft(1); // scroll the screen to the left by '1' pixel
		display.display();
		delay(10); // wait 10 ms
	}
	delay(1000); // wait for 1 second
	// scroll up
	for (uint8_t scroll = 0; scroll < display.height(); scroll++)
	{
		display.scrollUp(1); // scroll the screen to the left by '1' pixel
		display.display();
		delay(10); // wait 10 ms
	}
	delay(1000); // wait for 1 second
	// scroll down
	for (uint8_t scroll = 0; scroll < display.height(); scroll++)
	{
		display.scrollDown(1); // scroll the screen to the left by '1' pixel
		display.display();
		delay(10); // wait 10 ms
	}
	delay(1000); // wait for 1 second
	// scroll diagonal right-up
	for (uint8_t scroll = 0; scroll < display.width(); scroll++)
	{
		display.scrollRight(1); // scroll the screen to the right by '1' pixel
		display.scrollUp(1);	// scroll the screen to the left by '1' pixel
		display.display();
		delay(10); // wait 10 ms
	}
	delay(1000); // wait for 1 second
	// scroll diagonal right-down
	for (uint8_t scroll = 0; scroll < display.width(); scroll++)
	{
		display.scrollRight(1); // scroll the screen to the right by '1' pixel
		display.scrollDown(1);	// scroll the screen to the left by '1' pixel
		display.display();
		delay(10); // wait 10 ms
	}
	delay(1000); // wait for 1 second
	// scroll diagonal left-up
	for (uint8_t scroll = 0; scroll < display.width(); scroll++)
	{
		display.scrollLeft(1); // scroll the screen to the right by '1' pixel
		display.scrollUp(1);   // scroll the screen to the left by '1' pixel
		display.display();
		delay(10); // wait 10 ms
	}
	delay(1000); // wait for 1 second
	// scroll diagonal left-down
	for (uint8_t scroll = 0; scroll < display.width(); scroll++)
	{
		display.scrollLeft(1); // scroll the screen to the right by '1' pixel
		display.scrollDown(1); // scroll the screen to the left by '1' pixel
		display.display();
		delay(10); // wait 10 ms
	}
	delay(1000); // wait for 1 second
}

void testdrawline()
{
	int16_t i;

	display.clearDisplay(); // Clear display buffer

	for (i = 0; i < display.width(); i += 4)
	{
		display.drawLine(0, 0, i, display.height() - 1, ST7565_ON);
		display.display(); // Update screen with each newly-drawn line
		delay(1);
	}
	for (i = 0; i < display.height(); i += 4)
	{
		display.drawLine(0, 0, display.width() - 1, i, ST7565_ON);
		display.display();
		delay(1);
	}
	delay(250);

	display.clearDisplay();

	for (i = 0; i < display.width(); i += 4)
	{
		display.drawLine(0, display.height() - 1, i, 0, ST7565_ON);
		display.display();
		delay(1);
	}
	for (i = display.height() - 1; i >= 0; i -= 4)
	{
		display.drawLine(0, display.height() - 1, display.width() - 1, i, ST7565_ON);
		display.display();
		delay(1);
	}
	delay(250);

	display.clearDisplay();

	for (i = display.width() - 1; i >= 0; i -= 4)
	{
		display.drawLine(display.width() - 1, display.height() - 1, i, 0, ST7565_ON);
		display.display();
		delay(1);
	}
	for (i = display.height() - 1; i >= 0; i -= 4)
	{
		display.drawLine(display.width() - 1, display.height() - 1, 0, i, ST7565_ON);
		display.display();
		delay(1);
	}
	delay(250);

	display.clearDisplay();

	for (i = 0; i < display.height(); i += 4)
	{
		display.drawLine(display.width() - 1, 0, 0, i, ST7565_ON);
		display.display();
		delay(1);
	}
	for (i = 0; i < display.width(); i += 4)
	{
		display.drawLine(display.width() - 1, 0, i, display.height() - 1, ST7565_ON);
		display.display();
		delay(1);
	}

	delay(2000); // Pause for 2 seconds
}

void testdrawrect(void)
{
	display.clearDisplay();

	for (int16_t i = 0; i < display.height() / 2; i += 2)
	{
		display.drawRect(i, i, display.width() - 2 * i, display.height() - 2 * i, ST7565_ON);
		display.display(); // Update screen with each newly-drawn rectangle
		delay(1);
	}

	delay(2000);
}

void testfillrect(void)
{
	display.clearDisplay();

	for (int16_t i = 0; i < display.height() / 2; i += 3)
	{
		// The INVERSE color is used so rectangles alternate white/ST7565_ON
		display.fillRect(i, i, display.width() - i * 2, display.height() - i * 2, ST7565_INVERSE);
		display.display(); // Update screen with each newly-drawn rectangle
		delay(1);
	}

	delay(2000);
}

void testdrawcircle(void)
{
	display.clearDisplay();

	for (int16_t i = 0; i < max(display.width(), display.height()) / 2; i += 2)
	{
		display.drawCircle(display.width() / 2, display.height() / 2, i, ST7565_ON);
		display.display();
		delay(1);
	}

	delay(2000);
}

void testfillcircle(void)
{
	display.clearDisplay();

	for (int16_t i = max(display.width(), display.height()) / 2; i > 0; i -= 3)
	{
		// The INVERSE color is used so circles alternate white/black
		display.fillCircle(display.width() / 2, display.height() / 2, i, ST7565_INVERSE);
		display.display(); // Update screen with each newly-drawn circle
		delay(1);
	}

	delay(2000);
}

void testdrawroundrect(void)
{
	display.clearDisplay();

	for (int16_t i = 0; i < display.height() / 2 - 2; i += 2)
	{
		display.drawRoundRect(i, i, display.width() - 2 * i, display.height() - 2 * i,
							  display.height() / 4, ST7565_ON);
		display.display();
		delay(1);
	}

	delay(2000);
}

void testfillroundrect(void)
{
	display.clearDisplay();

	for (int16_t i = 0; i < display.height() / 2 - 2; i += 2)
	{
		// The INVERSE color is used so round-rects alternate white/ST7565_ON
		display.fillRoundRect(i, i, display.width() - 2 * i, display.height() - 2 * i,
							  display.height() / 4, ST7565_INVERSE);
		display.display();
		delay(1);
	}

	delay(2000);
}

void testdrawtriangle(void)
{
	display.clearDisplay();

	for (int16_t i = 0; i < max(display.width(), display.height()) / 2; i += 5)
	{
		display.drawTriangle(
			display.width() / 2, display.height() / 2 - i,
			display.width() / 2 - i, display.height() / 2 + i,
			display.width() / 2 + i, display.height() / 2 + i, ST7565_ON);
		display.display();
		delay(1);
	}

	delay(2000);
}

void testfilltriangle(void)
{
	display.clearDisplay();

	for (int16_t i = max(display.width(), display.height()) / 2; i > 0; i -= 5)
	{
		// The INVERSE color is used so triangles alternate white/black
		display.fillTriangle(
			display.width() / 2, display.height() / 2 - i,
			display.width() / 2 - i, display.height() / 2 + i,
			display.width() / 2 + i, display.height() / 2 + i, ST7565_INVERSE);
		display.display();
		delay(1);
	}

	delay(2000);
}

void testdrawchar(void)
{
	display.clearDisplay();

	display.setTextSize(1);			 // Normal 1:1 pixel scale
	display.setTextColor(ST7565_ON); // Draw white text
	display.setCursor(0, 0);		 // Start at top-left corner
	display.cp437(true);			 // Use full 256 char 'Code Page 437' font

	// Not all the characters will fit on the display. This is normal.
	// Library will draw what it can and the rest will be clipped.
	for (int16_t i = 0; i < 256; i++)
	{
		if (i == '\n')
			display.write(' ');
		else
			display.write(i);
	}

	display.display();
	delay(2000);
}

void testdrawstyles(void)
{
	display.clearDisplay();

	display.setTextSize(1);			 // Normal 1:1 pixel scale
	display.setTextColor(ST7565_ON); // Draw white text
	display.setCursor(0, 0);		 // Start at top-left corner
	display.println(F("Hello, world!"));

	display.setTextColor(ST7565_OFF, ST7565_ON); // Draw 'inverse' text
	display.println(3.141592);

	display.setTextSize(2); // Draw 2X-scale text
	display.setTextColor(ST7565_ON);
	display.print(F("0x"));
	display.println(0xDEADBEEF, HEX);

	display.display();
	delay(2000);
}

void testscrolltext(void)
{
	display.clearDisplay();

	display.setTextSize(2); // Draw 2X-scale text
	display.setTextColor(ST7565_ON);
	display.setCursor(10, 0);
	display.println(F("scroll"));
	display.display(); // Show initial text
	delay(1000);

	// Scroll in various directions, pausing in-between:
	// scroll right
	for (uint8_t scroll = 0; scroll < 0x0F; scroll++)
	{
		display.scrollRight(1);
		display.display();
		delay(10);
	}
	delay(1000);
	// scroll left
	for (uint8_t scroll = 0; scroll < 0x0F; scroll++)
	{
		display.scrollLeft(1);
		display.display();
		delay(10);
	}
	delay(1000);
	// diagonal scroll right-up
	for (uint8_t scroll = 0; scroll < display.height() / 2; scroll++)
	{
		display.scrollRight(1);
		display.scrollUp(1);
		display.display();
		delay(10);
	}
	delay(1000);
	// diagonal scroll left-up
	for (uint8_t scroll = 0; scroll < display.height() / 2; scroll++)
	{
		display.scrollLeft(1);
		display.scrollUp(1);
		display.display();
		delay(10);
	}
	delay(1000);
}

void testdrawbitmap(void)
{
	display.clearDisplay();

	display.drawBitmap(
		(display.width() - LOGO_WIDTH) / 2,
		(display.height() - LOGO_HEIGHT) / 2,
		logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
	display.display();
	delay(1000);
}

#define XPOS 0 // Indexes into the 'icons' array in function below
#define YPOS 1
#define DELTAY 2

void testanimate(const uint8_t *bitmap, uint8_t w, uint8_t h)
{
	int8_t f, icons[NUMFLAKES][3];

	// Initialize 'snowflake' positions
	for (f = 0; f < NUMFLAKES; f++)
	{
		icons[f][XPOS] = random(1 - LOGO_WIDTH, display.width());
		icons[f][YPOS] = -LOGO_HEIGHT;
		icons[f][DELTAY] = random(1, 6);
		Serial.print(F("x: "));
		Serial.print(icons[f][XPOS], DEC);
		Serial.print(F(" y: "));
		Serial.print(icons[f][YPOS], DEC);
		Serial.print(F(" dy: "));
		Serial.println(icons[f][DELTAY], DEC);
	}

	for (;;)
	{							// Loop forever...
		display.clearDisplay(); // Clear the display buffer

		// Draw each snowflake:
		for (f = 0; f < NUMFLAKES; f++)
		{
			display.drawBitmap(icons[f][XPOS], icons[f][YPOS], bitmap, w, h, ST7565_ON);
		}

		display.display(); // Show the display buffer on the screen
		delay(200);		   // Pause for 1/10 second

		// Then update coordinates of each flake...
		for (f = 0; f < NUMFLAKES; f++)
		{
			icons[f][YPOS] += icons[f][DELTAY];
			// If snowflake is off the bottom of the screen...
			if (icons[f][YPOS] >= display.height())
			{
				// Reinitialize to a random position, just off the top
				icons[f][XPOS] = random(1 - LOGO_WIDTH, display.width());
				icons[f][YPOS] = -LOGO_HEIGHT;
				icons[f][DELTAY] = random(1, 6);
			}
		}
	}
}

// setup function
void setup()
{
	Serial.begin(115200);

	Serial.println(F("ST7565 LCD test"));

	// initialize the ST7565 LCD display with contrast = 12 (0 <= coontrast <= 63)
	display.begin(13);

	display.display();
	delay(2000); // Pause for 2 seconds

	testscrolldisplay(); // scroll whole display in different directions

	// Clear the buffer
	display.clearDisplay();

	// Draw a single pixel in white
	display.drawPixel(10, 10, ST7565_ON);

	// Show the display buffer on the screen. You MUST call display() after
	// drawing commands to make them visible on screen!
	display.display();
	delay(2000);

	// display.display() is NOT necessary after every single drawing command,
	// unless that's what you want...rather, you can batch up a bunch of
	// drawing operations and then update the screen all at once by calling
	// display.display(). These examples demonstrate both approaches...

	testdrawline(); // Draw many lines

	testdrawrect(); // Draw rectangles (outlines)

	testfillrect(); // Draw rectangles (filled)

	testdrawcircle(); // Draw circles (outlines)

	testfillcircle(); // Draw circles (filled)

	testdrawroundrect(); // Draw rounded rectangles (outlines)

	testfillroundrect(); // Draw rounded rectangles (filled)

	testdrawtriangle(); // Draw triangles (outlines)

	testfilltriangle(); // Draw triangles (filled)

	testdrawchar(); // Draw characters of the default font

	testdrawstyles(); // Draw 'stylized' characters

	testscrolltext(); // Draw scrolling text

	testdrawbitmap(); // Draw a small bitmap image

	// Invert and restore display, pausing in-between
	display.invertDisplay(true);
	delay(1000);
	display.invertDisplay(false);
	delay(1000);

	testanimate(logo_bmp, LOGO_WIDTH, LOGO_HEIGHT); // Animate bitmaps
}

// main loop (nothing here!)
void loop()
{
}