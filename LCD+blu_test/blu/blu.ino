#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>

// ================= BLE UUID配置 =================
#define GOPRO_SERVICE_UUID       "FEA6"
#define GOPRO_QUERY_REQ_UUID     "b5f90076-aa8d-11e3-9046-0002a5d5c51b"
#define GOPRO_QUERY_RSP_UUID     "b5f90077-aa8d-11e3-9046-0002a5d5c51b"

// ================= 全局变量 =================
BLEClient* pClient;
BLERemoteCharacteristic* pQueryReqChar;
BLERemoteCharacteristic* pQueryRspChar;
bool isConnected = false;

// ================= TLV响应解析 =================
void parseTLVResponse(uint8_t* data, size_t length) {
  Serial.println("\n=== Recieve GoPro ===");
  Serial.print("Original (HEX): ");
  for (int i=0; i<length; i++) {
    Serial.printf("%02X ", data[i]);
  }
  Serial.println("\n----------------------");

  // TLV格式解析
  size_t index = 0;
  while (index < length) {
    if (index + 2 > length) {
      Serial.println("[ERROR] data incomplete");
      break;
    }

    uint8_t type = data[index++];
    uint8_t len = data[index++];

    if (index + len > length) {
      Serial.println("[ERROR] too long");
      break;
    }

    Serial.printf("Status ID: 0x%02X\n", type);
    Serial.printf("Len: %d\n", len);
    Serial.print("Content: ");
    
    // 打印数据内容
    for (int i=0; i<len; i++) {
      Serial.printf("%02X ", data[index+i]);
    }
    
    // 特殊状态解析（示例）
    if (type == 0x01) { // 录制状态
      Serial.printf("\nResult: %sRecording\n", 
                   data[index] ? "1" : "0");
    }
    else if (type == 0x02) { // 电量状态
      Serial.printf("\nBattery: %d%%\n", data[index]);
    }
    
    Serial.println("----------------------");
    index += len;
  }
}

// ================= 通知回调 =================
void notifyCallback(BLERemoteCharacteristic* pChar, 
                   uint8_t* data, 
                   size_t length, 
                   bool isNotify) {
  if (pChar->getUUID().toString() == GOPRO_QUERY_RSP_UUID) {
    parseTLVResponse(data, length);
  }
}

// ================= 发送指令 =================
void sendControlCommand() {
  if (!isConnected || !pQueryReqChar) return;

  uint8_t command[] = {0x01, 0x13};
  
  if (pQueryReqChar->canWrite()) {
    pQueryReqChar->writeValue({0x01, 0x13});
    Serial.println("[发送] 指令: 02 13 10");
  }
}

// ================= BLE连接 =================
bool connectToGoPro() {
  BLEDevice::init("ESP32-Controller");
  BLEScan* pScan = BLEDevice::getScan();
  pScan->setActiveScan(true);
  pScan->setInterval(100);
  pScan->setWindow(99);

  Serial.println("[Scan] Searching GoPro");
  BLEScanResults results = pScan->start(5);

  for (int i = 0; i < results.getCount(); i++) {
    BLEAdvertisedDevice device = results.getDevice(i);

    if (device.haveServiceUUID() && 
        device.getServiceUUID().equals(BLEUUID(GOPRO_SERVICE_UUID))) {
          
      pClient = BLEDevice::createClient();
      if (pClient->connect(&device)) {
        Serial.println("[Connect] Connect to GoPro");

        BLERemoteService* pService = pClient->getService(GOPRO_SERVICE_UUID);
        if (!pService) {
          pClient->disconnect();
          return false;
        }

        // 获取特征
        pQueryReqChar = pService->getCharacteristic(GOPRO_QUERY_REQ_UUID);
        pQueryRspChar = pService->getCharacteristic(GOPRO_QUERY_RSP_UUID);
        
        if (!pQueryReqChar || !pQueryRspChar) {
          pClient->disconnect();
          return false;
        }

        // 启用通知
        if (pQueryRspChar->canNotify()) {
          pQueryRspChar->registerForNotify(notifyCallback);
          Serial.println("[Notify] Notify enable");
        }

        isConnected = true;
        return true;
      }
    }
  }
  return false;
}

// ================= 主程序 =================
void setup() {
  Serial.begin(115200);
  delay(1000); // 等待串口初始化
  Serial.println("\n===== GoPro Status =====");

  if (connectToGoPro()) {
    sendControlCommand();
  } else {
    Serial.println("[ERROR] Connect fail");
  }
}

void loop() {
  // 每10秒查询一次
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 10000) {
    if (isConnected) {
      sendControlCommand();
    } else {
      Serial.println("[Retry] Connect again...");
      isConnected = connectToGoPro();
    }
    lastCheck = millis();
  }
  delay(100);
}