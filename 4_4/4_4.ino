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

const uint16_t COL_START = 0x17;  // 列起始地址23
const uint16_t COL_END = 0x24;    // 列结束地址36 (14列宽)
const uint16_t ROW_START = 0x00;  // 行起始地址0
const uint16_t ROW_END = 0xBF;    // 行结束地址191 (192行高)

const uint16_t topLeftX = COL_START; // 23
const uint16_t topLeftY = ROW_START; // 0

// 8x8字体绘制参数
const uint8_t FONT_WIDTH = 8;
const uint8_t FONT_HEIGHT = 8;



const uint8_t font[10][8] = {
  {0x3E, 0x7F, 0x63, 0x63, 0x63, 0x63, 0x7F, 0x3E}, // 0
  {0x0C, 0x1C, 0x3C, 0x0C, 0x0C, 0x0C, 0x3F, 0x3F}, // 1
  {0x3E, 0x7F, 0x63, 0x06, 0x0C, 0x18, 0x7F, 0x7F}, // 2
  {0x3E, 0x7F, 0x03, 0x1E, 0x03, 0x63, 0x7F, 0x3E}, // 3
  {0x06, 0x0E, 0x1E, 0x36, 0x66, 0x7F, 0x06, 0x06}, // 4
  {0x7F, 0x7F, 0x60, 0x7E, 0x03, 0x63, 0x7F, 0x3E}, // 5
  {0x3E, 0x7F, 0x60, 0x7E, 0x63, 0x63, 0x7F, 0x3E}, // 6
  {0x7F, 0x7F, 0x03, 0x06, 0x0C, 0x18, 0x18, 0x18}, // 7
  {0x3E, 0x7F, 0x63, 0x3E, 0x63, 0x63, 0x7F, 0x3E}, // 8
  {0x3E, 0x7F, 0x63, 0x63, 0x3F, 0x03, 0x7F, 0x3E}  // 9
};


// 旋转矩阵计算函数（90度顺时针）
void Rotate90(const uint8_t src[8], uint8_t dst[8]) {
  memset(dst, 0, 8); // 清空目标数组
  for(int y=0; y<8; y++) {       // 原始行号
    for(int x=0; x<8; x++) {     // 原始列号
      // 计算旋转后的坐标
      int newY = x;             // 新行号 = 原始列号
      int newX = 7 - y;         // 新列号 = 7 - 原始行号
      // 获取原始位状态
      bool bitState = (src[y] >> (7 - x)) & 0x01;
      // 设置新位状态
      dst[newY] |= (bitState << (7 - newX));
    }
  }
}

void RotateAndScale(const uint8_t src[8], uint8_t dst[8]) {
  memset(dst, 0, 8); // 初始化目标数组
  for(int y=0; y<8; y++) {
    for(int x=0; x<8; x++) {
      // 获取原始像素状态
      bool bit = (src[y] >> (7 - x)) & 0x01;
      if(!bit) continue;

      // 计算旋转后的坐标
      int newX = 7 - y; // 旋转后的列号
      int newY = x;     // 旋转后的行号

      // 缩放后的坐标（整除2）
      int scaledX = newX / 2;
      int scaledY = newY / 2;

      // 确保在4x4范围内
      if(scaledX >=4 || scaledY >=4) continue;

      // 设置目标位（高四位）
      dst[scaledY] |= (1 << (7 - scaledX));
    }
  }
}

// 设置列地址范围
void Set_Column_Address(uint16_t start, uint16_t end) {
  Write_Command(0x2A);
  Write_Data(start >> 8);
  Write_Data(start & 0xFF);
  Write_Data(end >> 8);
  Write_Data(end & 0xFF);
}

// 设置行地址范围
void Set_Row_Address(uint16_t start, uint16_t end) {
  Write_Command(0x2B);
  Write_Data(start >> 8);
  Write_Data(start & 0xFF);
  Write_Data(end >> 8);
  Write_Data(end & 0xFF);
}
// 实时旋转绘制函数
void Draw_Rotated_Number(uint8_t num) {
  if(num > 9) return;
  
  uint8_t rotatedData[8] = {0};
  // Rotate90(font[num], rotatedData); // 生成旋转数据

  RotateAndScale(font[num], rotatedData);

  // 计算显示区域（保持中心点）
  // const uint16_t startX = centerX - 4; // 29-4=25
  // const uint16_t startY = centerY - 4; // 95-4=91
  const uint16_t startX = COL_START; // 23
  const uint16_t startY = ROW_START + 5; // 0+5=5

  // 设置绘制区域（8x8）
  Write_Command(0x2A); 
  Write_Data(startX); Write_Data(startX + 7);
  Write_Command(0x2B);
  Write_Data(startY); Write_Data(startY + 7);
  Write_Command(0x2C);

  // 绘制旋转后的数据
  for(int y=0; y<8; y++) {
    uint8_t line = rotatedData[y];
    for(int x=7; x>=0; x--) { // 从高位到低位
      if(line & (1 << x)) {
        // 白色像素（4字节数据）
        Write_Data(0xFF); Write_Data(0xFF); 
        Write_Data(0xFF); Write_Data(0xFF);
      } else {
        // 黑色背景
        Write_Data(0x00); Write_Data(0x00);
        Write_Data(0x00); Write_Data(0x00);
      }
    }
  }
}

// void Draw_Rotated_Number(uint8_t num) {
//   if(num > 9) return;
  
//   uint8_t rotatedData[8] = {0};
//   RotateAndScale(font[num], rotatedData); // 生成旋转并缩放后的数据

//   // 计算显示区域（4x4，居中）
//   const uint16_t startX = centerX + 2; 
//   const uint16_t startY = centerY + 2;

//   // 设置列地址范围（4列）
//   Set_Column_Address(startX, startX + 3);
//   // 设置行地址范围（4行）
//   Set_Row_Address(startY, startY + 3);
//   Write_Command(0x2C); // 开始写入数据

//   // 绘制每个像素
//   for(int y=0; y<8; y++) {
//     uint8_t line = rotatedData[y];
//     for(int x=0; x<8; x++) { // 处理高四位
//       bool bitState = (line >> (7 - x)) & 0x01;
//       if(bitState) {
//         // 白色像素（4字节）
//         Write_Data(0xFF); Write_Data(0xFF);
//         Write_Data(0xFF); Write_Data(0xFF);
//       } else {
//         // 黑色背景
//         Write_Data(0x00); Write_Data(0x00);
//         Write_Data(0x00); Write_Data(0x00);
//       }
//     }
//   }
// }

// 清除数字区域（在显示新数字前调用）
// void Clear_Number_Area() {
//   // 使用相同坐标区域
//   uint16_t startX = centerX - FONT_WIDTH/2;
//   uint16_t startY = centerY - FONT_HEIGHT/2;

//   Write_Command(0x2A);
//   Write_Data(startX);
//   Write_Data(startX + FONT_WIDTH - 1);

//   Write_Command(0x2B);
//   Write_Data(startY);
//   Write_Data(startY + FONT_HEIGHT - 1);

//   Write_Command(0x2C);
  
//   // 填充黑色（168*384屏每个像素需要4字节）
//   for(uint32_t i=0; i<FONT_WIDTH*FONT_HEIGHT; i++){
//     Write_Data(0x00);
//     Write_Data(0x00);
//     Write_Data(0x00);
//     Write_Data(0x00);
//   }
// }

void Clear_Number_Area() {
  const uint16_t startX = centerX - 2;
  const uint16_t startY = centerY - 2;

  Set_Column_Address(startX, startX + 3);
  Set_Row_Address(startY, startY + 3);
  Write_Command(0x2C);
  
  // 填充黑色（4x4区域）
  for(uint32_t i=0; i<4*4; i++) {
    Write_Data(0x00); Write_Data(0x00);
    Write_Data(0x00); Write_Data(0x00);
  }
}

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
  // Draw_Rectangle(centerX - 2, centerY, centerX + 2, centerY, 0xFFFF); // 绘制固定
}


void loop() {
  for(int i=0; i<10; i++){
    Clear_Number_Area();
    
    Draw_Rotated_Number(i);
    
    delay(1000);
  }
}

