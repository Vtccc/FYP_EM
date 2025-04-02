#include <Arduino.h>
#include "BLEDevice.h"

#define ServiceCharacteristic(S, C) ThisClient->getService(S)->getCharacteristic(C)

static BLEDevice *ThisDevice;
static BLEClient *ThisClient = ThisDevice->createClient();
BLEScan *ThisScan = ThisDevice->getScan();
static BLEUUID ServiceUUID("fea6");
static BLEUUID QuerySendCharacteristicUUID("b5f90076-aa8d-11e3-9046-0002a5d5c51b");
static BLEUUID QueryReceiveCharacteristicUUID("b5f90077-aa8d-11e3-9046-0002a5d5c51b");
static bool ItsOn = false;
static bool NotificationEnabled = false;
static std::string receivedData;

// 解析 TLV 数据，返回已解析的字节数
size_t ParseTLVData(const std::string& data)
{
  size_t offset = 0;
  while (offset + 2 <= data.length())
  {
    uint8_t tag = data[offset];
    uint8_t length = data[offset + 1];
    offset += 2;

    // 跳过 Length=0 的无效 Tag
    if (length == 0)
    {
      Serial.printf("Skipping invalid Tag: 0x%02X (Length=0)\n", tag);
      continue;
    }

    if (offset + length > data.length()) break;

    std::string value = data.substr(offset, length);
    offset += length;

    Serial.printf("Tag: 0x%02X, Length: %d, Value: ", tag, length);
    for (char c : value) Serial.printf("%02X ", (uint8_t)c);
    Serial.println();

    // 检查录制状态
    if (tag == 0x02 && length == 1)
    {
      Serial.println(value[0] == 0x01 ? "Recording!" : "Not recording.");
    }
  }
  return offset;
}

static void NotificationCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
  receivedData.append((char*)pData, length);
  size_t parsedLength = ParseTLVData(receivedData);
  receivedData = receivedData.substr(parsedLength);
}

bool ScanAndConnect(void)
{
  ThisScan->clearResults();
  ThisScan->start(3);
  for (int i = 0; i < ThisScan->getResults().getCount(); i++)
  {
    if (ThisScan->getResults().getDevice(i).isAdvertisingService(ServiceUUID))
    {
      ThisScan->stop();
      if (ThisClient->connect(new BLEAdvertisedDevice(ThisScan->getResults().getDevice(i))))
      {
        Serial.println("Connected!");
        return true;
      }
    }
  }
  return false;
}

void setup()
{
  Serial.begin(115200);
  ThisDevice->init("");
  ThisDevice->setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  if (ScanAndConnect())
  {
    BLERemoteCharacteristic* pRemoteChar = ServiceCharacteristic(ServiceUUID, QueryReceiveCharacteristicUUID);
    if (pRemoteChar && pRemoteChar->canNotify())
    {
      pRemoteChar->registerForNotify(NotificationCallback);
      NotificationEnabled = true;
      Serial.println("Notifications enabled!");
    }

    BLERemoteCharacteristic* pSendChar = ServiceCharacteristic(ServiceUUID, QuerySendCharacteristicUUID);
    if (pSendChar)
    {
      uint8_t cmd[] = {0x01, 0x13}; // 使用正确的查询命令
      pSendChar->writeValue(cmd, sizeof(cmd));
      ItsOn = true;
    }
  }
}

void loop()
{
  delay(1000);
}