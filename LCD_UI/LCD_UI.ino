#include <Arduino.h>

// 定义引脚和全局变量
#define TE_PIN   1
#define RES_PIN  2
#define DC_PIN   3
#define CS_PIN   4
#define SCLK_PIN 5
#define SDI_PIN  6

const int centerX = (23+36)/2;        // 屏幕水平中心
const int centerY = 191/2;            // 屏幕垂直中心
const int rectHeight = 1;             // 矩形固定高度
const int maxValue = 50;              // 最大输入值
const double k = 0.05;                // 非线性缩放因子

const uint16_t COL_START = 0x17;      // 列起始地址23
const uint16_t COL_END = 0x24;        // 列结束地址36 (14列宽)
const uint16_t ROW_START = 0x00;      // 行起始地址0
const uint16_t ROW_END = 0xBF;        // 行结束地址191 (192行高)

// 8x8字体数据（旋转用）
const uint8_t font[11][8] = {
  {0xFC, 0x8C, 0x8C, 0x8C, 0xFC, 0x00, 0x00, 0x00}, //0
  {0x70, 0x30, 0x30, 0x30, 0xFC, 0x00, 0x00, 0x00}, //1
  {0xFC, 0x0C, 0xFC, 0x80, 0xFC, 0x00, 0x00, 0x00}, //2
  {0xFC, 0x0C, 0xFC, 0x0C, 0xFC, 0x00, 0x00, 0x00}, //3
  {0x8C, 0x8C, 0xFC, 0x0C, 0x0C, 0x00, 0x00, 0x00}, //4
  {0xFC, 0x80, 0xFC, 0x0C, 0xFC, 0x00, 0x00, 0x00}, //5
  {0xF8, 0x80, 0xFC, 0x8C, 0xFC, 0x00, 0x00, 0x00}, //6
  {0xFC, 0x0C, 0x0C, 0x0C, 0x0C, 0x00, 0x00, 0x00}, //7
  {0xFC, 0x8C, 0xFC, 0x8C, 0xFC, 0x00, 0x00, 0x00}, //8
  {0xFC, 0x8C, 0xFC, 0x0C, 0xFC, 0x00, 0x00, 0x00}, //9
  {0x00, 0xC0, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00} //:
};

// 屏幕初始化函数
void Initial_ST7305() {
  // 复位屏幕
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

// 基础通信函数
void Write_Command(uint8_t command) {
  digitalWrite(DC_PIN, LOW);
  digitalWrite(CS_PIN, LOW);
  shiftOut(SDI_PIN, SCLK_PIN, MSBFIRST, command);
  digitalWrite(CS_PIN, HIGH);
}

void Write_Data(uint8_t data) {
  digitalWrite(DC_PIN, HIGH);
  digitalWrite(CS_PIN, LOW);
  shiftOut(SDI_PIN, SCLK_PIN, MSBFIRST, data);
  digitalWrite(CS_PIN, HIGH);
}

// 图形绘制函数
void Clear_Screen() {
  Write_Command(0x2A); Write_Data(0x17); Write_Data(0x24);
  Write_Command(0x2B); Write_Data(0x00); Write_Data(0xBF);
  Write_Command(0x2C);
  for(uint16_t i=0;i<192;i++) 
    for(uint16_t j=0;j<14;j++) 
      for(uint8_t k=0;k<4;k++) Write_Data(0x00);
}

void Draw_Rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, bool color) {
  Write_Command(0x2A); Write_Data(x1); Write_Data(x2);
  Write_Command(0x2B); Write_Data(y1); Write_Data(y2);
  Write_Command(0x2C);
  for(uint16_t i=y1; i<=y2; i++)
    for(uint16_t j=x1; j<=x2; j++)
      for(uint8_t k=0;k<4;k++) 
        Write_Data(color ? 0xFF : 0x00);
}

// 平衡指示器函数
void Draw_Balance_Indicator(int value) {
  static int lastWidth = 0;
  static int lastDir = 0;
  
  value = constrain(value, -maxValue, maxValue);
  double scaled = tanh(k * -value) / tanh(k * maxValue);
  int width = (int)(scaled * 60);
  int dir = (width > 0) ? 1 : (width < 0) ? -1 : 0;

  if(width == lastWidth && dir == lastDir) return;

  // 清除旧图形
  if(lastDir != 0) {
    if(lastDir == 1) {
      Draw_Rectangle(centerX-1, centerY, centerX+1, centerY+lastWidth, false);
    } else {
      Draw_Rectangle(centerX-1, centerY+lastWidth, centerX+1, centerY, false);
    }
  }

  // 绘制新图形
  if(dir != 0) {
    if(dir == 1) {
      Draw_Rectangle(centerX-1, centerY, centerX+1, centerY+width, true);
    } else {
      Draw_Rectangle(centerX-1, centerY+width, centerX+1, centerY, true);
    }
  }

  // 更新中心线
  Draw_Rectangle(centerX-2, centerY, centerX+2, centerY, true);
  
  lastWidth = width;
  lastDir = dir;
}

// 数字显示函数
void Rotate90(const uint8_t src[8], uint8_t dst[8]) {
  memset(dst, 0, 8);
  for(int y=0; y<8; y++) 
    for(int x=0; x<8; x++) 
      dst[x] |= ((src[y] >> (7-x)) & 1) << y;
}

void Draw_Rotated_Number(uint8_t num, uint16_t startRow, uint16_t startCol) {
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


// 主程序
unsigned long lastBalanceUpdate = 0;
unsigned long lastNumberUpdate = 0;
int currentValue = -50;
uint8_t currentNumber = 0;

void setup() {
  pinMode(TE_PIN, OUTPUT);
  pinMode(RES_PIN, OUTPUT);
  pinMode(DC_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(SCLK_PIN, OUTPUT);
  pinMode(SDI_PIN, OUTPUT);

  Initial_ST7305();
  Clear_Screen();
  Draw_Rectangle(centerX-2, centerY, centerX+2, centerY, true); // 中心线
  Draw_Rotated_Number(10, 23, 21);
}

void loop() {
  unsigned long now = millis();

  // 每500ms更新平衡指示
  if(now - lastBalanceUpdate >= 500) {
    lastBalanceUpdate = now;
    currentValue += 10;
    if(currentValue > 50) currentValue = -50;
    Draw_Balance_Indicator(currentValue);
  }

  // 每1000ms更新数字
  if(now - lastNumberUpdate >= 1000) {
    lastNumberUpdate = now;
    Draw_Rotated_Number(currentNumber, 23, 5);
    Draw_Rotated_Number((currentNumber + 3) % 10, 23, 13);
    
    Draw_Rotated_Number((currentNumber + 1) % 10, 23, 29);
    Draw_Rotated_Number((currentNumber + 2) % 10, 23, 37);
    Draw_Rotated_Number((currentNumber - 1) % 10, 23, 176);
    Draw_Rotated_Number((currentNumber - 2) % 10, 23, 184);
    currentNumber = (currentNumber + 1) % 10 ;
  }
}