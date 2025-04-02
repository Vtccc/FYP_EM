#include <Arduino.h>
#include "BLEDevice.h"

#define ServiceCharacteristic(S, C) ThisClient->getService(S)->getCharacteristic(C)

static BLEDevice *ThisDevice;
static BLEClient *ThisClient = ThisDevice->createClient();
BLEScan *ThisScan = ThisDevice->getScan();
static BLEUUID ServiceUUID("fea6"); // GoPro BLE 服务 UUID
static BLEUUID QuerySendCharacteristicUUID("b5f90076-aa8d-11e3-9046-0002a5d5c51b"); // 查询发送特征 UUID
static BLEUUID QueryReceiveCharacteristicUUID("b5f90077-aa8d-11e3-9046-0002a5d5c51b"); // 查询接收特征 UUID
static bool ItsOn = false;
static bool NotificationEnabled = false;

// 通知回调函数
static void NotificationCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
  Serial.print("Notification received: ");
  for (int i = 0; i < length; i++) {
    Serial.printf("%02X ", pData[i]); // 以十六进制格式输出查询结果
  }
  Serial.println();
}

bool ScanAndConnect(void)
{
  ThisScan->clearResults();
  ThisScan->start(3); // 扫描 3 秒
  for (int i = 0; i < ThisScan->getResults().getCount(); i++)
  {
    if (ThisScan->getResults().getDevice(i).haveServiceUUID() && ThisScan->getResults().getDevice(i).isAdvertisingService(ServiceUUID))
    {
      Serial.println("Found GoPro device!");
      ThisScan->stop();
      if (ThisClient->connect(new BLEAdvertisedDevice(ThisScan->getResults().getDevice(i))))
      {
        Serial.println("Connected to GoPro!");
        return true;
      }
      else
      {
        Serial.println("Failed to connect to GoPro!");
        return false;
      }
    }
  }
  Serial.println("No GoPro device found!");
  return false;
}

void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);
  Serial.begin(115200); // 初始化串口用于输出查询结果
  ThisDevice->init("");
  ThisDevice->setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);

  // 直接执行连接和发送查询指令的逻辑
  if (ScanAndConnect())
  {
    // 获取查询接收特征并启用通知
    BLERemoteCharacteristic* pRemoteCharacteristic = ServiceCharacteristic(ServiceUUID, QueryReceiveCharacteristicUUID);
    if (pRemoteCharacteristic && pRemoteCharacteristic->canNotify())
    {
      pRemoteCharacteristic->registerForNotify(NotificationCallback); // 注册通知回调
      NotificationEnabled = true;
      Serial.println("Notification enabled!");
    }
    else
    {
      Serial.println("Notification not supported!");
    }

    // 获取查询发送特征
    BLERemoteCharacteristic* pSendCharacteristic = ServiceCharacteristic(ServiceUUID, QuerySendCharacteristicUUID);
    if (pSendCharacteristic)
    {
      // 发送查询指令：查询 GoPro 状态
      uint8_t queryCommand[] = {0x02, 0x13,0x10}; // 查询状态命令
      pSendCharacteristic->writeValue(queryCommand, sizeof(queryCommand));
      Serial.println("Query command sent successfully!");
      ItsOn = true;
      digitalWrite(LED_BUILTIN, 1); // 打开LED表示设备已连接并发送指令
    }
    else
    {
      Serial.println("Failed to find query send characteristic!");
      ESP.restart(); // 如果找不到特征，重启设备
    }
  }
  else
  {
    ESP.restart(); // 如果连接失败，重启设备
  }
}

void loop(void)
{
  // 保持设备运行
  if (ItsOn && NotificationEnabled)
  {
    // 主循环不需要做任何事情，数据通过通知回调处理
    delay(1000);
  }
}