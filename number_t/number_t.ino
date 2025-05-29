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

const uint8_t font_rotated[10][8] PROGMEM = {
  {0x00,0x7C,0xC6,0xC6,0xC6,0xC6,0x7C,0x00}, // 0
  {0x00,0x18,0x38,0x18,0x18,0x18,0x7E,0x00}, // 1
  {0x00,0x7C,0xC6,0x0C,0x30,0xC6,0xFE,0x00}, // 2
  {0x00,0x7C,0xC6,0x1C,0x06,0xC6,0x7C,0x00}, // 3
  {0x00,0x0C,0x1C,0x3C,0x6C,0xFE,0x0C,0x00}, // 4
  {0x00,0xFE,0xC0,0xFC,0x06,0xC6,0x7C,0x00}, // 5
  {0x00,0x7C,0xC0,0xFC,0xC6,0xC6,0x7C,0x00}, // 6
  {0x00,0xFE,0x06,0x0C,0x18,0x30,0x30,0x00}, // 7
  {0x00,0x7C,0xC6,0x7C,0xC6,0xC6,0x7C,0x00}, // 8
  {0x00,0x7C,0xC6,0xC6,0x7E,0x06,0x7C,0x00}  // 9
};

const uint8_t font_mirror[10][8] PROGMEM = {
  {0x3E,0x63,0x63,0x63,0x63,0x63,0x3E,0x00}, // 0 → 0x7C镜像
  {0x0C,0x1C,0x3C,0x0C,0x0C,0x0C,0x3F,0x00}, // 1 → 0x30镜像
  {0x3E,0x63,0x03,0x0E,0x38,0x60,0x7F,0x00}, // 2 → 0x5E镜像
  {0x3E,0x63,0x03,0x1E,0x03,0x63,0x3E,0x00}, // 3 → 0x3E镜像
  {0x06,0x0E,0x1E,0x36,0x7F,0x06,0x06,0x00}, // 4 → 0x60镜像
  {0x7F,0x60,0x7E,0x03,0x03,0x63,0x3E,0x00}, // 5 → 0x7F镜像
  {0x1E,0x30,0x60,0x7E,0x63,0x63,0x3E,0x00}, // 6 → 0x3C镜像
  {0x7F,0x03,0x06,0x0C,0x18,0x18,0x18,0x00}, // 7 → 0xFE镜像
  {0x3E,0x63,0x63,0x3E,0x63,0x63,0x3E,0x00}, // 8 → 0x7C镜像
  {0x3E,0x63,0x63,0x3F,0x03,0x06,0x3C,0x00}  // 9 → 0x3E镜像
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

// 绘制单个字符
void Draw_Char(uint16_t x, uint16_t y, char c, uint16_t color) {
  if (c < '0' || c > '9') return;
  uint8_t index = c - '0';
  
  Set_Column_Address(x, x + 7);
  Set_Row_Address(y, y + 7);
  Write_Command(0x2C);

  for (uint8_t row = 0; row < 8; row++) {
    uint8_t line = font[index][row];
    for (int8_t col = 7; col >= 0; col--) {
      if (line & (1 << col)) {
        // 绘制像素
        Write_Data(color >> 8); Write_Data(color);
        Write_Data(color >> 8); Write_Data(color);
      } else {
        // 黑色背景
        Write_Data(0x00); Write_Data(0x00);
        Write_Data(0x00); Write_Data(0x00);
      }
    }
  }
}

// 旋转90度后的字体数据生成工具
uint8_t* CreateRotatedFont() {
  static uint8_t rotated[10][8] = {0};
  
  for(int n=0; n<10; n++) {
    for(uint8_t y=0; y<8; y++) {    // 原始行号
      uint8_t newByte = 0;
      for(uint8_t x=0; x<8; x++) {  // 原始列号
        // 获取原始位（从最高位开始）
        uint8_t originalBit = (font[n][7-x] >> y) & 0x01;
        // 设置新字节的对应位（从最高位开始）
        newByte |= (originalBit << (7 - x));
      }
      rotated[n][y] = newByte;
    }
  }
  return (uint8_t*)rotated;
}

// 实际使用的旋转后字体数据
const uint8_t* rotated_font = CreateRotatedFont();

// 绘制数字（支持3位数）
void Draw_Number(uint16_t x, uint16_t y, int num, uint16_t color) {
  char str[4];
  itoa(num, str, 10);
  uint8_t len = strlen(str);
  
  for (uint8_t i = 0; i < len; i++) {
    Draw_Char(x + (i * 8), y, str[i], color);
  }
}
void Draw_Center_Number(uint8_t num) {
  if(num > 9) return;

  // 计算字体起始坐标（保持中心对齐）
  uint16_t startX = centerX - FONT_WIDTH/2;  // 29 - 4 = 25
  uint16_t startY = centerY - FONT_HEIGHT/2; // 95 - 4 = 91

  // 设置精确绘制区域（8x8像素）
  Write_Command(0x2A); // 列地址设置
  Write_Data(startX); 
  Write_Data(startX + FONT_WIDTH - 1);

  Write_Command(0x2B); // 行地址设置
  Write_Data(startY);
  Write_Data(startY + FONT_HEIGHT - 1);

  Write_Command(0x2C); // 开始写入数据

  // 绘制数字
  for(uint8_t row=0; row<8; row++){
    uint8_t line = font[num][row];
    for(int8_t col=7; col>=0; col--){ // 从高位到低位
      if(line & (1<<col)){
        // 白色像素（根据你的初始化配置调整）
        Write_Data(0xFF); // R
        Write_Data(0xFF); // G
        Write_Data(0xFF); // B
        Write_Data(0xFF); // Alpha（如有需要）
      } else {
        // 黑色背景
        Write_Data(0x00);
        Write_Data(0x00); 
        Write_Data(0x00);
        Write_Data(0x00);
      }
    }
  }
}

// 实时旋转绘制函数
void Draw_Rotated_Number(uint8_t num) {
  if(num > 9) return;
  
  uint8_t rotatedData[8] = {0};
  Rotate90(font[num], rotatedData); // 生成旋转数据

  // 计算显示区域（保持中心点）
  const uint16_t startX = centerX - 4; // 29-4=25
  const uint16_t startY = centerY - 4; // 95-4=91

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

// 清除数字区域（在显示新数字前调用）
void Clear_Number_Area() {
  // 使用相同坐标区域
  uint16_t startX = centerX - FONT_WIDTH/2;
  uint16_t startY = centerY - FONT_HEIGHT/2;

  Write_Command(0x2A);
  Write_Data(startX);
  Write_Data(startX + FONT_WIDTH - 1);

  Write_Command(0x2B);
  Write_Data(startY);
  Write_Data(startY + FONT_HEIGHT - 1);

  Write_Command(0x2C);
  
  // 填充黑色（168*384屏每个像素需要4字节）
  for(uint32_t i=0; i<FONT_WIDTH*FONT_HEIGHT; i++){
    Write_Data(0x00);
    Write_Data(0x00);
    Write_Data(0x00);
    Write_Data(0x00);
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

// void Draw_Rotated_Number(uint8_t num) {
//   if(num > 9) return;
  
//   const uint8_t width = 8;
//   const uint8_t height = 8;
  
//   // 计算起始坐标（保持中心点不变）
//   uint16_t startX = centerX - height/2; // 原垂直方向变水平
//   uint16_t startY = centerY - width/2;  // 原水平方向变垂直

//   // 设置绘制区域（旋转后尺寸）
//   Write_Command(0x2A);
//   Write_Data(startX);
//   Write_Data(startX + height - 1);

//   Write_Command(0x2B);
//   Write_Data(startY);
//   Write_Data(startY + width - 1);

//   Write_Command(0x2C);

//   // 旋转后绘制逻辑
//   for(uint8_t col=0; col<width; col++){ // 原列变行
//     uint8_t mask = 0x80 >> col;        // 位掩码
//     for(uint8_t row=0; row<height; row++){
//       uint8_t line = pgm_read_byte(&font_rotated[num][row]);
//       if(line & mask){
//         // 白色像素（根据硬件格式调整）
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

void Draw_Mirror_Number(uint8_t num) {
  if(num > 9) return;

  // 保持原始中心坐标
  const uint16_t startX = centerX - 4; // 29-4=25
  const uint16_t startY = centerY - 4; // 95-4=91

  // 设置绘制区域
  Write_Command(0x2A);
  Write_Data(startX);
  Write_Data(startX + 7);

  Write_Command(0x2B);
  Write_Data(startY);
  Write_Data(startY + 7);

  Write_Command(0x2C);

  // 镜像绘制逻辑
  for(uint8_t row=0; row<8; row++) {
    uint8_t line = pgm_read_byte(&font_mirror[num][row]);
    for(int8_t col=7; col>=0; col--){ // 反向扫描实现镜像
      if(line & (1<<col)) {
        // 白色像素（根据硬件调整）
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

// 自动生成镜像字体的工具函数
uint8_t MirrorByte(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4; // 交换半字节
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2; // 交换每2位
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1; // 交换相邻位
  return b;
}

void Draw_Mirror_Rotate_Number(uint8_t num) {
  if(num > 9) return;

  const uint8_t size = 8; // 8x8像素
  const uint16_t startX = centerX - size/2; // 水平居中
  const uint16_t startY = centerY - size/2; // 垂直居中

  // 设置绘制区域
  Write_Command(0x2A);
  Write_Data(startX);
  Write_Data(startX + size - 1);

  Write_Command(0x2B);
  Write_Data(startY);
  Write_Data(startY + size - 1);

  Write_Command(0x2C);

  // 复合变换绘制逻辑
  for(uint8_t col=0; col<size; col++){    // 旋转后的列（原行）
    uint8_t line = pgm_read_byte(&font[num][col]);
    for(uint8_t row=0; row<size; row++){  // 旋转后的行（镜像处理）
      uint8_t bitPos = 7 - row;          // 水平镜像处理
      
      if(line & (1 << bitPos)){           // 检查对应位
        // 白色像素（RGB8888格式）
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

//   const uint8_t width = 8;
//   const uint8_t height = 8;
  
//   // 计算起始坐标（保持中心点不变）
//   uint16_t startX = centerX - height/2; // 29 - 4 = 25
//   uint16_t startY = centerY - width/2;  // 95 - 4 = 91

//   // 设置绘制区域（交换宽高）
//   Write_Command(0x2A); // 列地址设置
//   Write_Data(startX);
//   Write_Data(startX + height - 1);

//   Write_Command(0x2B); // 行地址设置
//   Write_Data(startY);
//   Write_Data(startY + width - 1);

//   Write_Command(0x2C); // 开始写入数据

//   // 绘制旋转后的数字
//   for(uint8_t row=0; row<height; row++) {
//     uint8_t line = rotated_font[num*8 + row];
//     for(int8_t bit=7; bit>=0; bit--) { // 从高位到低位
//       if(line & (1 << bit)) {
//         // 白色像素（根据硬件格式调整）
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
  Draw_Rectangle(centerX - 2, centerY, centerX + 2, centerY, 0xFFFF); // 绘制固定
}


void loop() {
  for(int i=0; i<10; i++){
    Clear_Number_Area();
    // Draw_Center_Number(i);
    Draw_Rotated_Number(i);
    // Draw_Mirror_Number(i); // 显示镜像数字
    // Draw_Mirror_Rotate_Number(i);
    delay(1000);
  }
}

