// LCDD.cpp
#include "LCDD.h"

LCDD::LCDD(uint8_t te, uint8_t res, uint8_t dc, uint8_t cs, uint8_t sclk, uint8_t sdi)
  : TE_PIN(te), RES_PIN(res), DC_PIN(dc), CS_PIN(cs), SCLK_PIN(sclk), SDI_PIN(sdi) {
  pinMode(TE_PIN, OUTPUT);
  pinMode(RES_PIN, OUTPUT);
  pinMode(DC_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(SCLK_PIN, OUTPUT);
  pinMode(SDI_PIN, OUTPUT);
}

void LCDD::Initial_ST7305() 
{ // 复位屏幕
  digitalWrite(RES_PIN, HIGH);
  delay(100);
  digitalWrite(RES_PIN, LOW);
  delay(10000);
  digitalWrite(RES_PIN, HIGH);
  delay(10000);

  // 7305 初始化命令
  Write_Command(0xD6); // NVM Load Control
  Write_Data(0x17);
  Write_Data(0x02);

  Write_Command(0xD1); // Booster Enable
  Write_Data(0x01);

  Write_Command(0xC0); // Gate Voltage Setting
  Write_Data(0x08); // VGH 00:8V  04:10V  08:12V   0E:15V
  Write_Data(0x06); // VGL 00:-5V   04:-7V   0A:-10V

  Write_Command(0xC1); // VSHP Setting (4.8V)
  Write_Data(0x3C); // VSHP1
  Write_Data(0x3C); // VSHP2
  Write_Data(0x3C); // VSHP3
  Write_Data(0x3C); // VSHP4

  Write_Command(0xC2); // VSLP Setting (0.98V)
  Write_Data(0x23); // VSLP1
  Write_Data(0x23); // VSLP2
  Write_Data(0x23); // VSLP3
  Write_Data(0x23); // VSLP4

  Write_Command(0xC4); // VSHN Setting (-3.6V)
  Write_Data(0x5A); // VSHN1
  Write_Data(0x5A); // VSHN2
  Write_Data(0x5A); // VSHN3
  Write_Data(0x5A); // VSHN4

  Write_Command(0xC5); // VSLN Setting (0.22V)
  Write_Data(0x37); // VSLN1
  Write_Data(0x37); // VSLN2
  Write_Data(0x37); // VSLN3
  Write_Data(0x37); // VSLN4

  Write_Command(0xD8); // HPM=32Hz
  Write_Data(0xA6); // ~51Hz
  Write_Data(0xE9); // ~1Hz

  Write_Command(0xB2); // Frame Rate Control
  Write_Data(0x12); // HPM=32hz ; LPM=1hz

  Write_Command(0xB3); // Update Period Gate EQ Control in HPM
  Write_Data(0xE5);
  Write_Data(0xF6);
  Write_Data(0x05); // HPM EQ Control
  Write_Data(0x46);
  Write_Data(0x77);
  Write_Data(0x77);
  Write_Data(0x77);
  Write_Data(0x77);
  Write_Data(0x76);
  Write_Data(0x45);

  Write_Command(0xB4); // Update Period Gate EQ Control in LPM
  Write_Data(0x05); // LPM EQ Control
  Write_Data(0x46);
  Write_Data(0x77);
  Write_Data(0x77);
  Write_Data(0x77);
  Write_Data(0x77);
  Write_Data(0x76);
  Write_Data(0x45);

  Write_Command(0x62); // Gate Timing Control
  Write_Data(0x32);
  Write_Data(0x03);
  Write_Data(0x1F);

  Write_Command(0xB7); // Source EQ Enable
  Write_Data(0x13);

  Write_Command(0xB0); // Gate Line Setting
  Write_Data(0x60); // 384 line

  Write_Command(0x11); // Sleep out
  delay(255);

  Write_Command(0xC9); // Source Voltage Select
  Write_Data(0x00); // VSHP1; VSLP1 ; VSHN1 ; VSLN1

  Write_Command(0x36); // Memory Data Access Control
  Write_Data(0x48); // MX=1 ; DO=1

  Write_Command(0x3A); // Data Format Select
  Write_Data(0x10); // 10:4write for 24bit ; 11: 3write for 24bit

  Write_Command(0xB9); // Gamma Mode Setting
  Write_Data(0x20); // 20: Mono 00:4GS

  Write_Command(0xB8); // Panel Setting
  Write_Data(0x09); // Panel Setting Frame inversion  09:column 29:dot_1-Frame 25:dot_1-Line

  // WRITE RAM 168 * 384
  Write_Command(0x2A); // Column Address Setting
  Write_Data(0x17);
  Write_Data(0x24);

  Write_Command(0x2B); // Row Address Setting
  Write_Data(0x00);
  Write_Data(0xBF);

  Write_Command(0x35); // TE
  Write_Data(0x00);

  Write_Command(0xD0); // Auto power down
  Write_Data(0xFF);

  Write_Command(0x39); // LPM

  Write_Command(0x29); // DISPLAY ON
}


void LCDD::Write_Command(uint8_t command) {
  digitalWrite(DC_PIN, LOW);
  digitalWrite(CS_PIN, LOW);
  shiftOut(SDI_PIN, SCLK_PIN, MSBFIRST, command);
  digitalWrite(CS_PIN, HIGH);
}

void LCDD::Write_Data(uint8_t data) {
  digitalWrite(DC_PIN, HIGH);
  digitalWrite(CS_PIN, LOW);
  shiftOut(SDI_PIN, SCLK_PIN, MSBFIRST, data);
  digitalWrite(CS_PIN, HIGH);
}

void LCDD::Clear_Screen(uint16_t color) {
  Write_Command(0x2A); // 设置列地址
  Write_Data(0x17);    // 起始列
  Write_Data(0x24);    // 结束列 (168列)

  Write_Command(0x2B); // 设置行地址
  Write_Data(0x00);    // 起始行
  Write_Data(0xBF);    // 结束行 (384行)

  Write_Command(0x2C); // 开始写入数据

  for (uint16_t i=0;i<192;i++) {
    for (uint16_t j=0;j<14;j++) {
      Write_Data(color);   // 发送高字节
      Write_Data(color);   // 发送高字节
      Write_Data(color);   // 发送高字节
      Write_Data(color);   // 发送高字节

      // Write_Data(color & 0xFF); // 发送低字节
    }
  }
}


void LCDD::Draw_Rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) 
{
  Write_Command(0x2A); // 设置列地址
  Write_Data(x1); // 起始列高字节
  // Write_Data(x1 & 0xFF); // 起始列低字节
  Write_Data(x2); // 结束列高字节
  // Write_Data(x2 & 0xFF); // 结束列低字节

  Write_Command(0x2B); // 设置行地址
  Write_Data(y1); // 起始行高字节
  // Write_Data(y1 & 0xFF); // 起始行低字节
  Write_Data(y2); // 结束行高字节
  // Write_Data(y2 & 0xFF); // 结束行低字节

  Write_Command(0x2C); // 开始写入数据

  for (uint16_t i = y1; i <= y2; i++) {
    for (uint16_t j = x1; j <= x2; j++) {
      // Write_Data(color >> 8);   // 发送高字节
      // Write_Data(color & 0xFF); // 发送低字节
      Write_Data(color);
      Write_Data(color);
      Write_Data(color);
      Write_Data(color);
    }
  }
}



void LCDD::Draw_Balance_Indicator(int value) 
{
      const int centerX = (23+36)/2;           // Middle of screen x
    const int centerY = 191/2;           // Middle of screen y
    const int rectHeight = 1;        // Height of the rectangles
    const int maxValue = 50;          // Maximum input value (like maxAngle in original)
    const double k = 0.05;            // Non-linear scaling factor
    
    // Clamp the input value
    if (value < -maxValue) value = -maxValue;
    if (value > maxValue) value = maxValue;
    
    // Calculate scaled width using non-linear scaling (similar to original)
    double scaledValue = tanh(k * -value) / tanh(k * maxValue);
    int rectWidth = (int)(scaledValue * 60);  // 60 pixels maximum width
    
    // Draw center vertical line
    Draw_Rectangle(centerX-2, centerY, centerX+2, centerY, 0xFFFF);

    // Draw rectangles on both sides
    if (rectWidth > 0) {
        // Right side rectangle
        Draw_Rectangle(centerX - rectHeight, centerY, centerX + rectHeight, centerY+rectWidth, 0xFFFF);
    } else {
        // Left side rectangle
        Draw_Rectangle(centerX - rectHeight, centerY+rectWidth, centerX + rectHeight, centerY, 0xFFFF);
    }
}

void LCDD::Rotate90(const uint8_t src[8], uint8_t dst[8]) {
  memset(dst, 0, 8);
  for(int y=0; y<8; y++) 
    for(int x=0; x<8; x++) 
      dst[x] |= ((src[y] >> (7-x)) & 1) << y;
}

void LCDD::Draw_Rotated_Number(uint8_t num, uint16_t startRow, uint16_t startCol) {
  if(num > 10) return;
  uint8_t rotated[8];
  Rotate90(font[num], rotated);

  Write_Command(0x2A); Write_Data(startRow); Write_Data(startRow + 7); // 8列宽
  Write_Command(0x2B); Write_Data(startCol); Write_Data(startCol + 7);  // 8行高
  Write_Command(0x2C);
  
  for(uint8_t y=0; y<8; y++) {
    for(int x=7; x>=0; x--) {
      bool pixel = rotated[y] & (1 << x);
      for(uint8_t k=0;k<4;k++) 
        Write_Data(pixel ? 0xFF : 0x00);
    }
  }
}