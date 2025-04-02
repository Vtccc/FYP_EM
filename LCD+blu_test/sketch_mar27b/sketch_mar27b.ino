

// 定义引脚（根据实际连接修改）
#define TE_PIN   1  // 帧同步（未使用）
#define RES_PIN  2  // 复位
#define DC_PIN   3  // 数据/命令选择
#define CS_PIN   4  // 片选
#define SCLK_PIN 5  // 软件SPI时钟
#define SDI_PIN  6  // 软件SPI数据输出

// 屏幕参数
const uint16_t SCREEN_WIDTH = 168;   // 列数
const uint16_t SCREEN_HEIGHT = 384;  // 行数
const uint8_t COL_ADDR_START = 0x17; // 列起始地址
const uint8_t COL_ADDR_END = 0x24;   // 列结束地址
const uint8_t ROW_ADDR_START = 0x00; // 行起始地址
const uint8_t ROW_ADDR_END = 0xBF;   // 行结束地址

// 颜色定义（4字节格式）
#define COLOR_WHITE 0xFF, 0xFF, 0xFF, 0xFF
#define COLOR_BLACK 0x00, 0x00, 0x00, 0x00

//-----------------------------------------------------------------------------
// SoftSPI 数据传输函数
//-----------------------------------------------------------------------------
void SPI_Transfer(uint8_t data) {
  for (uint8_t bit = 0; bit < 8; bit++) {
    digitalWrite(SCLK_PIN, LOW);              // 时钟下降沿
    digitalWrite(SDI_PIN, (data & 0x80) ? HIGH : LOW); // 发送最高位
    digitalWrite(SCLK_PIN, HIGH);             // 时钟上升沿（数据采样）
    data <<= 1;
  }
}

//-----------------------------------------------------------------------------
// 写命令和数据函数（SoftSPI）
//-----------------------------------------------------------------------------
void Write_Command(uint8_t cmd) {
  digitalWrite(DC_PIN, LOW);  // DC=0 表示命令
  digitalWrite(CS_PIN, LOW);
  SPI_Transfer(cmd);
  digitalWrite(CS_PIN, HIGH);
}

void Write_Data(uint8_t data) {
  digitalWrite(DC_PIN, HIGH); // DC=1 表示数据
  digitalWrite(CS_PIN, LOW);
  SPI_Transfer(data);
  digitalWrite(CS_PIN, HIGH);
}

//-----------------------------------------------------------------------------
// 初始化函数（与原代码一致）
//-----------------------------------------------------------------------------
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

//-----------------------------------------------------------------------------
// 地址和绘图函数（适配SoftSPI）
//-----------------------------------------------------------------------------
void Set_Address_Window(uint8_t col_start, uint8_t col_end, uint8_t row_start, uint8_t row_end) {
  Write_Command(0x2A); // 列地址
  Write_Data(col_start);
  Write_Data(col_end);

  Write_Command(0x2B); // 行地址
  Write_Data(row_start);
  Write_Data(row_end);

  Write_Command(0x2C); // 开始写入数据
}

void Pixel_To_Address(uint16_t x, uint16_t y, uint8_t *col, uint8_t *row) {
  *col = (x / 12) + COL_ADDR_START; // 列地址 = x/12 + 0x17
  *row = y / 2;                     // 行地址 = y/2
}

void Clear_Screen(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4) {
  Draw_Rectangle(COL_ADDR_START, COL_ADDR_END, ROW_ADDR_START, ROW_ADDR_END, d1, d2, d3, d4);
}

void Draw_Rectangle(uint8_t col_start, uint8_t col_end, uint8_t row_start, uint8_t row_end, 
                   uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4) {
  Set_Address_Window(col_start, col_end, row_start, row_end);

  // 计算总像素数（每个地址块对应12列×2行）
  uint16_t col_blocks = col_end - col_start + 1;
  uint16_t row_blocks = row_end - row_start + 1;
  uint32_t total_pixels = col_blocks * row_blocks * 12 * 2;

  // 批量发送数据（SoftSPI）
  digitalWrite(DC_PIN, HIGH);
  digitalWrite(CS_PIN, LOW);
  for (uint32_t i = 0; i < total_pixels; i++) {
    SPI_Transfer(d1);
    SPI_Transfer(d2);
    SPI_Transfer(d3);
    SPI_Transfer(d4);
  }
  digitalWrite(CS_PIN, HIGH);
}

//-----------------------------------------------------------------------------
// 主程序
//-----------------------------------------------------------------------------
void setup() {
  // 初始化引脚
  pinMode(RES_PIN, OUTPUT);
  pinMode(DC_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(SCLK_PIN, OUTPUT);
  pinMode(SDI_PIN, OUTPUT);

  Serial.begin(115200);
  Initial_ST7305();
  
  Clear_Screen(COLOR_BLACK); 
  Clear_Screen(COLOR_WHITE);
  Serial.println("1");

  // 示例：绘制居中黑色矩形（需计算坐标）
  // uint8_t col_start, col_end, row_start, row_end;
  // Pixel_To_Address(34, 167, &col_start, &row_start);
  // Pixel_To_Address(134, 217, &col_end, &row_end);
  // Draw_Rectangle(col_start, col_end, row_start, row_end, COLOR_BLACK);
  Serial.println("1");
}

void loop() {}