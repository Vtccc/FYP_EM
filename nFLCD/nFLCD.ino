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


// 初始化屏幕
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

// 写命令
void Write_Command(uint8_t command) {
  digitalWrite(DC_PIN, LOW);  // DC = 0 表示命令
  digitalWrite(CS_PIN, LOW);  // 片选使能
  shiftOut(SDI_PIN, SCLK_PIN, MSBFIRST, command); // 发送命令
  digitalWrite(CS_PIN, HIGH); // 片选禁用
}

// 写数据
void Write_Data(uint8_t data) {
  digitalWrite(DC_PIN, HIGH); // DC = 1 表示数据
  digitalWrite(CS_PIN, LOW);  // 片选使能
  shiftOut(SDI_PIN, SCLK_PIN, MSBFIRST, data); // 发送数据
  digitalWrite(CS_PIN, HIGH); // 片选禁用
}

// 清屏
void Clear_Screen(uint16_t color) {
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



// 绘制长方形
void Draw_Rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
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

void Draw_Balance_Indicator(int value) {
    static int lastRectWidth = 0;         // 保存上一次的宽度
    static int lastDirection = 0;         // 保存上一次方向（0:无 1:右 -1:左）
    

    
    // 数值钳位
    value = constrain(value, -maxValue, maxValue);
    
    // 计算缩放宽度
    double scaledValue = tanh(k * -value) / tanh(k * maxValue);
    int rectWidth = (int)(scaledValue * 60);
    int currentDirection = (rectWidth > 0) ? 1 : (rectWidth < 0) ? -1 : 0;

    // 若状态未改变则直接返回
    if (rectWidth == lastRectWidth && currentDirection == lastDirection) return;

    // 清除旧指示条
    if (lastDirection != 0) {
        if (lastDirection == 1) {
            // 清除右侧旧条
            Draw_Rectangle(centerX - rectHeight, 
                          centerY, 
                          centerX + rectHeight, 
                          centerY + lastRectWidth, 
                          0x0000);
        } else {
            // 清除左侧旧条
            Draw_Rectangle(centerX - rectHeight,
                          centerY + lastRectWidth,
                          centerX + rectHeight,
                          centerY,
                          0x0000);
        }
    }

    // 绘制新指示条
    if (currentDirection != 0) {
        if (currentDirection == 1) {
            // 绘制右侧新条
            Draw_Rectangle(centerX - rectHeight,
                          centerY,
                          centerX + rectHeight,
                          centerY + rectWidth,
                          0xFFFF);
        } else {
            // 绘制左侧新条
            Draw_Rectangle(centerX - rectHeight,
                          centerY + rectWidth,
                          centerX + rectHeight,
                          centerY,
                          0xFFFF);
        }
    }

    // 重绘中心线（可能被覆盖）
    Draw_Rectangle(centerX - 2, centerY, centerX + 2, centerY, 0xFFFF);

    // 更新记录值
    lastRectWidth = rectWidth;
    lastDirection = currentDirection;
}

void setup() {
  Serial.begin(115200);
  
  // 初始化引脚
  pinMode(TE_PIN, OUTPUT);
  pinMode(RES_PIN, OUTPUT);
  pinMode(DC_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(SCLK_PIN, OUTPUT);
  pinMode(SDI_PIN, OUTPUT);

  // 初始化屏幕
  Initial_ST7305();

  // 初始清屏并绘制静态元素
  Clear_Screen(0x0000);
  Draw_Rectangle(centerX - 2, centerY, centerX + 2, centerY, 0xFFFF); // 绘制固定中心线

  // 主循环
  while(true) {
    for (int i = -50; i <= 50; i += 10) {
        Draw_Balance_Indicator(i); // 仅更新指示条
        delay(500);
    }
    delay(500);
  }
}

void loop() {
  // 保留空循环
}

