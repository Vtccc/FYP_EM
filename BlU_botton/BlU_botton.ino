#include <Arduino.h>
#include "BLEDevice.h"
#include <OneButton.h>  // 包含OneButton库

#define ServiceCharacteristic(S, C) ThisClient->getService(S)->getCharacteristic(C)

static BLEDevice *ThisDevice;
static BLEClient *ThisClient = ThisDevice->createClient();
BLEScan *ThisScan = ThisDevice->getScan();
static BLEUUID ServiceUUID((uint16_t)0xFEA6);
static BLEUUID CommandWriteCharacteristicUUID("b5f90072-aa8d-11e3-9046-0002a5d5c51b");
static bool ItsOn = false;
static bool IsConnected = false;
unsigned long startTime;

// 定义按钮引脚和OneButton对象
const int BUTTON_PIN = 12;  // 假设按钮连接GPIO0
OneButton button(BUTTON_PIN, true); // true表示使用内部上拉电阻

bool ScanAndConnect(void) {
  ThisScan->clearResults();
  ThisScan->start(3, false); // 添加false参数避免重复扫描
  for (int i = 0; i < ThisScan->getResults().getCount(); i++) {
    if (ThisScan->getResults().getDevice(i).haveServiceUUID() && 
        ThisScan->getResults().getDevice(i).isAdvertisingService(BLEUUID(ServiceUUID))) {
      ThisScan->stop();
      if(ThisClient->connect(new BLEAdvertisedDevice(ThisScan->getResults().getDevice(i)))) {
        IsConnected = true;
        return true;
      }
    }
  }
  return false;
}

// 按钮单击事件处理函数 - 只发送开始指令
void handleButtonClick() {
  if (IsConnected && !ItsOn) {
    Serial.println("Button pressed - Sending start command");
    
    // 发送开始录制指令
    ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue({0x03, 0x01, 0x01, 0x01});
    ItsOn = true;
    digitalWrite(LED_BUILTIN, 1);
    startTime = millis();
    Serial.println("Recording started");
  } else if (!IsConnected) {
    Serial.println("Not connected to device!");
  } else if (ItsOn) {
    Serial.println("Recording already in progress");
  }
}

void setup(void) {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);
  
  // 初始化OneButton
  button.attachClick(handleButtonClick);
  
  ThisDevice->init("");
  ThisDevice->setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  
  Serial.println("Initializing... Attempting to connect to BLE device");
  
  // 开机自动连接设备
  if (ScanAndConnect()) {
    Serial.println("Connected to BLE device successfully");
    digitalWrite(LED_BUILTIN, 1); // 连接成功点亮LED
    delay(500);
    digitalWrite(LED_BUILTIN, 0); // 然后熄灭
  } else {
    Serial.println("Initial connection failed! Restarting...");
    delay(1000);
    ESP.restart();
  }
  
  Serial.println("System ready. Press button to start recording.");
}

void loop(void) {
  static unsigned long lastKeepAlive = 0;
  
  // 必须持续调用tick()以检测按钮事件
  button.tick();
  
  // 检查连接状态
  if (!ThisClient->isConnected() && IsConnected) {
    Serial.println("Device disconnected!");
    IsConnected = false;
    ItsOn = false;
    digitalWrite(LED_BUILTIN, 0);
    
    // 尝试重新连接
    Serial.println("Attempting to reconnect...");
    if (ScanAndConnect()) {
      Serial.println("Reconnected successfully");
    }
  }
  
  if (ItsOn) {
    // 30秒后自动停止
    if (millis() - startTime >= 30000) {
      ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue({0x03, 0x01, 0x01, 0x00});
      digitalWrite(LED_BUILTIN, 0);
      ItsOn = false;
      Serial.println("Recording stopped after 30 seconds");
    } 
    // 每5秒发送保持连接指令
    else if (millis() - lastKeepAlive >= 5000) {
      ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue({0x01, 0x05});
      lastKeepAlive = millis();
      Serial.println("Sending keep-alive");
    }
  }
  
  delay(10); // 防止CPU过载
}