/**********************************************************************************
 * This is a library for ST7565 Monochrome LCD Display.
 * These displays use SPI to communicate, 4 or 5 pins are required to
 *   interface
 * This is a free library WITH NO WARRANTY, use it at your own risk!
 *
 * https://simple-circuit.com/
 *
 ***********************************************************************************
 * This library depends on Adafruit GFX library at
 *   https://github.com/adafruit/Adafruit-GFX-Library
 *   being present on your system. Please make sure you have installed the latest
 *   version before using this library.
 ***********************************************************************************/
#ifndef _ST7565_LCD_H
#define _ST7565_LCD_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

#include <SPI.h>

#ifdef __SAM3X8E__
typedef volatile RwReg PortReg;
typedef uint32_t PortMask;
#elif defined(__AVR__)
typedef volatile uint8_t PortReg;
typedef uint8_t PortMask;
#else
typedef volatile uint32_t PortReg;
typedef uint8_t PortMask;
#endif
#define ST7565_ON 1		 // ON pixel
#define ST7565_OFF 0	 // OFF pixel
#define ST7565_INVERSE 2 // inverse pixel

#define LCDWIDTH 128
#define LCDHEIGHT 64

#define CMD_DISPLAY_OFF 0xAE // lcd off
#define CMD_DISPLAY_ON 0xAF	 // lcd on

#define CMD_SET_DISP_START_LINE 0x40 // start line address, 0x40
#define CMD_SET_PAGE 0xB0			 // start page address

#define CMD_SET_COLUMN_UPPER 0x10
#define CMD_SET_COLUMN_LOWER 0x00

#define CMD_SET_ADC_NORMAL 0xA0
#define CMD_SET_ADC_REVERSE 0xA1

#define CMD_SET_DISP_NORMAL 0xA6
#define CMD_SET_DISP_REVERSE 0xA7

#define CMD_SET_ALLPTS_NORMAL 0xA4
#define CMD_SET_ALLPTS_ON 0xA5
#define CMD_SET_BIAS_9 0xA2
#define CMD_SET_BIAS_7 0xA3

#define CMD_RMW 0xE0
#define CMD_RMW_CLEAR 0xEE
#define CMD_INTERNAL_RESET 0xE2
#define CMD_SET_COM_NORMAL 0xC0
#define CMD_SET_COM_REVERSE 0xC8
#define CMD_SET_POWER_CONTROL 0x28
#define CMD_SET_RESISTOR_RATIO 0x20
#define CMD_SET_VOLUME_FIRST 0x81
#define CMD_SET_VOLUME_SECOND 0
#define CMD_SET_STATIC_OFF 0xAC
#define CMD_SET_STATIC_ON 0xAD
#define CMD_SET_STATIC_REG 0x0
#define CMD_SET_BOOSTER_FIRST 0xF8
#define CMD_SET_BOOSTER_234 0
#define CMD_SET_BOOSTER_5 1
#define CMD_SET_BOOSTER_6 3
#define CMD_NOP 0xE3
#define CMD_TEST 0xF0

// define hardware SPI clock frequency
#define ST7565_SPI_CLOCK_DIV SPI_CLOCK_DIV8

class ST7565_LCD : public Adafruit_GFX
{
public:
	// Software SPI with explicit CS pin.
	ST7565_LCD(int8_t SID, int8_t SCLK, int8_t A0, int8_t RST, int8_t CS);
	// Software SPI with CS tied to ground.  Saves a pin but other pins can't be shared with other hardware.
	ST7565_LCD(int8_t SID, int8_t SCLK, int8_t A0, int8_t RST);
	// Hardware SPI based on hardware controlled SCK (SCLK) and MOSI (DIN) pins. CS is still controlled by any IO pin.
	// NOTE: MISO and SS will be set as an input and output respectively, so be careful sharing those pins!
	ST7565_LCD(int8_t A0, int8_t RST, int8_t CS);

	void begin(uint8_t contrast);
	void command(uint8_t c);
	void data(uint8_t c);

	void setContrast(uint8_t val);
	void drawPixel(int16_t x, int16_t y, uint16_t color);
	void display();
	void clearDisplay(void);
	void scrollRight(uint8_t s);
	void scrollLeft(uint8_t s);
	void scrollDown(uint8_t s);
	void scrollUp(uint8_t s);

private:
	int8_t sid, sclk, a0, rst, cs;
	volatile PortReg *mosiport, *clkport;
	PortMask mosipinmask, clkpinmask;

	void spiwrite(uint8_t c);
	bool isHardwareSPI();
};

#endif
