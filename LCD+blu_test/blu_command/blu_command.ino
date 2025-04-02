#include <Arduino.h>
#include "BLEDevice.h"

#define ServiceCharacteristic(S, C) ThisClient->getService(S)->getCharacteristic(C)

static BLEDevice *ThisDevice;
static BLEClient *ThisClient = ThisDevice->createClient();
BLEScan *ThisScan = ThisDevice->getScan();
static BLEUUID ServiceUUID((uint16_t)0xFEA6);
static BLEUUID CommandWriteCharacteristicUUID("b5f90072-aa8d-11e3-9046-0002a5d5c51b");
static bool ItsOn = false;

bool ScanAndConnect(void)
{
  ThisScan->clearResults();
  ThisScan->start(3);
  for (int i = 0; i < ThisScan->getResults().getCount(); i++)
    if (ThisScan->getResults().getDevice(i).haveServiceUUID() && ThisScan->getResults().getDevice(i).isAdvertisingService(BLEUUID(ServiceUUID)))
    {
      ThisScan->stop();
      ThisClient->connect(new BLEAdvertisedDevice(ThisScan->getResults().getDevice(i)));
      return true;
    }
  return false;
}

void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);
  ThisDevice->init("");
  ThisDevice->setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);

  // 直接执行连接和发送指令的逻辑
  if (ScanAndConnect())
  {
    // 发送打开指令
    // ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue({0x03, 0x01, 0x01, 0x01});
    ItsOn = true;
    digitalWrite(LED_BUILTIN, 1); // 打开LED表示设备已连接并发送指令
  }
  else
  {
    ESP.restart(); // 如果连接失败，重启设备
  }
}

void loop(void)
{
  // 保持设备运行
  if (ItsOn)
  {
    // 每隔5秒发送一次保持连接的指令
    delay(5000);
    ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue({0x01, 0x05});
  }
}