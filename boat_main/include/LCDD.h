// LCDD.h
#ifndef LCDD_H
#define LCDD_H

#include <Arduino.h>

class LCDD {
public:
  LCDD(uint8_t te, uint8_t res, uint8_t dc, uint8_t cs, uint8_t sclk, uint8_t sdi);
  void Initial_ST7305();
  void Clear_Screen(uint16_t color);
  // void Draw_Center_Rectangle(uint16_t width, uint16_t height, uint16_t color);
  void Draw_Rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
  void Draw_Balance_Indicator(int value);
  void Draw_Line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
  // void Set_Partial_Refresh(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

private:
  const uint8_t TE_PIN;
  const uint8_t RES_PIN;
  const uint8_t DC_PIN;
  const uint8_t CS_PIN;
  const uint8_t SCLK_PIN;
  const uint8_t SDI_PIN;

  void Write_Command(uint8_t command);
  void Write_Data(uint8_t data);
};

#endif