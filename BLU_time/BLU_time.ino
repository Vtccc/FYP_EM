#include <Arduino.h>
#include "BLEDevice.h"

#define ServiceCharacteristic(S, C) ThisClient->getService(S)->getCharacteristic(C)

static BLEDevice *ThisDevice;
static BLEClient *ThisClient = ThisDevice->createClient();
BLEScan *ThisScan = ThisDevice->getScan();
static BLEUUID ServiceUUID((uint16_t)0xFEA6);
static BLEUUID CommandWriteCharacteristicUUID("b5f90072-aa8d-11e3-9046-0002a5d5c51b");
static bool ItsOn = false;
unsigned long startTime;  // 添加开始时间记录

bool ScanAndConnect(void) {
  ThisScan->clearResults();
  ThisScan->start(3);
  for (int i = 0; i < ThisScan->getResults().getCount(); i++)
    if (ThisScan->getResults().getDevice(i).haveServiceUUID() && 
        ThisScan->getResults().getDevice(i).isAdvertisingService(BLEUUID(ServiceUUID))) {
      ThisScan->stop();
      ThisClient->connect(new BLEAdvertisedDevice(ThisScan->getResults().getDevice(i)));
      return true;
    }
  return false;
}

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);
  ThisDevice->init("");
  ThisDevice->setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);

  if (ScanAndConnect()) {
    // 发送打开指令
    ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue({0x03, 0x01, 0x01, 0x01});
    ItsOn = true;
    digitalWrite(LED_BUILTIN, 1);
    startTime = millis();  // 记录开始时间
  } else {
    ESP.restart();
  }
}

void loop(void) {
  static unsigned long lastKeepAlive = 0;
  
  if (ItsOn) {
    // 检查是否超过30秒
    if (millis() - startTime >= 30000) {
      // 发送停止录制指令（请根据实际设备协议替换正确指令）
      ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue({0x03, 0x01, 0x01, 0x00});
      digitalWrite(LED_BUILTIN, 0);  // 关闭LED
      ItsOn = false;
      ThisClient->disconnect();      // 断开连接
      
      while(true) { delay(1000); }   // 停止后续操作
    }
    
    // 保持连接指令（非阻塞方式）
    if (millis() - lastKeepAlive >= 5000) {
      ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue({0x01, 0x05});
      lastKeepAlive = millis();
    }
  }
}