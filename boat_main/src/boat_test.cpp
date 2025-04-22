#include <Arduino.h>
#include "esp_log.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
// #include <Adafruit_ICM20948.h>
#include "SD.h"
#include <QMC5883LCompass.h>
#include "SPI.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "IMU.h"
#include "registers.h"

#include "LCDD.h"

#include "BLEDevice.h"

// // E-paper library
// #include "DEV_Config.h"
// #include "EPD.h"
// #include "GUI_Paint.h"
// #include "ImageData.h"
// #include <stdlib.h>

static const char *TAG_GPS = "GPS_Task";
static const char *TAG_IMU = "IMU_Task";
static const char *TAG_SD = "SD_Card_Task";
static const char *TAG_COMPASS = "Compass_Task";
static const char *TAG_EPAPER = "EPaper_Task";

// GPS
static const int RXPin = 4, TXPin = 5;
static const uint32_t GPSBaud = 38400;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
EspSoftwareSerial::UART ss;

// IMU
#define IMU_SDA 6
#define IMU_SCL 7
#define IMU_DRDY 12
ICM42688 IMU(Wire, 0x68, IMU_SDA, IMU_SCL);
String rollsString = "";
double ax, ay, az;
double gx, gy, gz;
double roll, pitch, yaw;

// Compass
int compass_azimuth;

// SD Card
#define PIN_SPI_CS 10 // The ESP32 pin GPIO5
File myFile;
int count = 0;
double rolls[10] = {400, 400, 400, 400, 400, 400, 400, 400, 400, 400};

// RTC
int64_t get_current_time_us() {
    return esp_timer_get_time();
}

//BLU
#define BUTTON_PIN 14
#define DEBOUNCE_DELAY 50

static BLEDevice *ThisDevice;
static BLEUUID ServiceUUID((uint16_t)0xFEA6);
static BLEUUID CommandWriteCharacteristicUUID("b5f90072-aa8d-11e3-9046-0002a5d5c51b");
static bool ItsOn = false;
static bool bleInitialized = false;

SemaphoreHandle_t sdMutex;

// Function to format time as a string
String get_formatted_time() {
    int64_t current_time_us = get_current_time_us();
    int64_t seconds = current_time_us / 1000000;
    int64_t microseconds = current_time_us % 1000000;
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%lld.%06lld", seconds, microseconds);
    return String(buffer);
}

String arrayToString(double *array, size_t size)
{
  String result = "[";
  for (size_t i = 0; i < size; ++i)
  {
    result += String(array[i], 2);
    if (i < size - 1)
    {
      result += ",";
    }
  }
  result += "]";
  return result;
}

void displayInfo()
{
  static char logBuffer[256];
  static char satellites[16], location[32], date[16], time[16];

  if (gps.satellites.isValid())
  {
    sprintf(satellites, "%d", gps.satellites.value());
  }
  else
  {
    strcpy(satellites, "INVALID");
  }

  if (gps.location.isValid())
  {
    sprintf(location, "%.6f,%.6f", gps.location.lat(), gps.location.lng());
  }
  else
  {
    strcpy(location, "INVALID");
  }

  if (gps.date.isValid())
  {
    sprintf(date, "%d/%d/%d", gps.date.month(), gps.date.day(), gps.date.year());
  }
  else
  {
    strcpy(date, "INVALID");
  }

  if (gps.time.isValid())
  {
    sprintf(time, "%02d:%02d:%02d.%02d", gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond());
  }
  else
  {
    strcpy(time, "INVALID");
  }

  sprintf(logBuffer, "displayInfo, Satellites: %s, Location: %s, Date/Time: %s, Time: %s", satellites, location, date, time);

  ESP_LOGI(TAG_GPS, "%s", logBuffer);
}

void GPS_Task(void *pvParameters)
{
  ESP_LOGI(TAG_GPS, "GPS_Task");
  Serial1.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  while (true)
  {
    while (Serial1.available())
    {
      if (gps.encode(Serial1.read()))
      {
        // displayInfo();
      }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void IMU_Task(void *pvParameters) {
  ESP_LOGI(TAG_IMU, "IMU_Task");


  Wire.begin(IMU_SDA, IMU_SCL);
  int status = IMU.begin();
  if (status < 0) {
    ESP_LOGE(TAG_IMU, "IMU 初始化失败: %d", status);
    vTaskDelete(NULL);
  }

  IMU.setAccelFS(ICM42688::gpm8);    // ±8G
  IMU.setGyroFS(ICM42688::dps500);   // ±500dps
  IMU.setAccelODR(ICM42688::odr12_5);
  IMU.setGyroODR(ICM42688::odr12_5);

  while (true) {

    IMU.getAGT();

 
    ax = IMU.accX() * 9.81;        // m/s²
    ay = IMU.accY() * 9.81;
    az = IMU.accZ() * 9.81;
    
    gx = IMU.gyrX() * (M_PI / 180.0); // rad/s
    gy = IMU.gyrY() * (M_PI / 180.0);
    gz = IMU.gyrZ() * (M_PI / 180.0);


    roll = -(atan2(ay, az) * 180.0 / M_PI);
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
    yaw += (gz * 0.01);           // 积分得到偏航角

   
    Serial.printf("Roll: %.2f, Pitch: %.2f\n", roll, pitch);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void Button_Task(void *pvParameters) {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    bool lastState = HIGH;
    bool pressed = false;
    unsigned long lastDebounceTime = 0;

    while(true) {
        bool currentState = digitalRead(BUTTON_PIN);
        
        if (currentState != lastState) {
            lastDebounceTime = millis();
        }

        if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
            if (currentState == LOW && !pressed) {
                pressed = true;
                
                // 执行BLE操作
                if (bleInitialized && !ItsOn) {
                    BLEScan* ThisScan = ThisDevice->getScan();
                    ThisScan->clearResults();
                    ThisScan->start(3, false);
                    
                    for(int i = 0; i < ThisScan->getResults().getCount(); i++) {
                        BLEAdvertisedDevice device = ThisScan->getResults().getDevice(i);
                        if(device.haveServiceUUID() && device.isAdvertisingService(ServiceUUID)) {
                            BLEClient* ThisClient = ThisDevice->createClient();
                            if(ThisClient->connect(&device)) {
                                BLERemoteCharacteristic* pChar = ThisClient->getService(ServiceUUID)
                                    ->getCharacteristic(CommandWriteCharacteristicUUID);
                                    
                                if(pChar) {
                                    uint8_t cmdOn[] = {0x03, 0x01, 0x01, 0x01};
                                    pChar->writeValue(cmdOn, sizeof(cmdOn), true);
                                    ItsOn = true;
                                    
                                    // 记录到SD卡
                                    if(xSemaphoreTake(sdMutex, portMAX_DELAY)) {
                                        myFile.printf("%s,ButtonPressed,1\n", get_formatted_time().c_str());
                                        myFile.flush();
                                        xSemaphoreGive(sdMutex);
                                    }
                                }
                            }
                            break;
                        }
                    }
                    ThisScan->clearResults();
                }
            } else if (currentState == HIGH && pressed) {
                pressed = false;
            }
        }
        
        lastState = currentState;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void SD_Card_Task(void *pvParameters)
{
  SPIClass myspi = SPIClass(HSPI);
  myspi.begin(12, 13, 11, 10);
  while (!SD.begin(10)) 
  {
    Serial.println(F("SD Card failed, or not present"));
    vTaskDelay(500 / portTICK_PERIOD_MS); // Wait for 1 second before retrying
  }
  ESP_LOGI(TAG_SD, "SD CARD OK!");

  // Find the smallest available file number
  int fileNumber = 0;
  char filename[30];
  while (true)
  {
    sprintf(filename, "/sailing_data%03d.txt", fileNumber);
    if (!SD.exists(filename))
    {
      break;
    }
    fileNumber++;
  }

  // Open the file and write the header
  myFile = SD.open(filename, FILE_WRITE);
  if (!myFile)
  {
    Serial.println(F("Failed to open file!"));
    vTaskDelete(NULL);
  }

  // Write file header
  if(xSemaphoreTake(sdMutex, portMAX_DELAY)) {
    myFile.println("{");
    myFile.println("refreshRate=10;");
    myFile.println("version=1.0;");
    myFile.println("format=[timestamp,gps.satellites,gps.hdop,gps.location.age,gps.lat,gps.lng,gps.speed,gps.course,gps.month,gps.day,gps.year,gps.hour,gps.minute,gps.second,gps.centisecond,compass,roll,pitch,yaw,button_event]");
    myFile.println("}");
    myFile.flush(); // Ensure header is written

    xSemaphoreGive(sdMutex);
  }
  
  while (true)
  {
    if(xSemaphoreTake(sdMutex, portMAX_DELAY)) {
    // 原有数据记录代码
    // Get current time
    String current_time = get_formatted_time();

    // Prepare data to write
    size_t rollsSize = sizeof(rolls) / sizeof(rolls[0]);
    rollsString = arrayToString(rolls, rollsSize);

    // Write data to file
    myFile.printf("%s,%d,%f,%d,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%d\n",
                  current_time.c_str(), gps.satellites.value(), gps.hdop.hdop(), gps.location.age(),
                  gps.location.lat(), gps.location.lng(), gps.speed.mps(), gps.course.deg(),
                  gps.date.month(), gps.date.day(), gps.date.year(), gps.time.hour(),
                  gps.time.minute(), gps.time.second(), gps.time.centisecond(),
                  compass_azimuth, roll, pitch, yaw, ItsOn ? 1 : 0);

    // Flush data to SD card
    myFile.flush();
    xSemaphoreGive(sdMutex);
    // Print time to Serial
    Serial.print("Current Time: ");
    Serial.println(current_time);

    // Reset the rolls array and count
    // for (int i = 0; i < 10; i++)
    // {
    //   rolls[i] = 400;
    // }
    // count = 0;
    // rollsString.clear();

            
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust delay to ensure 10 records per second
  }

  // Close the file (this will never be reached in this example)
  myFile.close();
}

void Compass_Task(void *pvParameters)
{
  QMC5883LCompass compass;
  compass.init();
  while (1)
  {
    // Read compass values
    compass.read();

    // Return Azimuth reading
    compass_azimuth = compass.getAzimuth();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void ePaper_Task(void *pvParameters)
{
      const int rectHeight = 1;        
    const int maxValue = 50;         
    const double k = 0.05;           
    const uint16_t bgColor = 0x0000; 
    const uint16_t fgColor = 0xFFFF; 

    
    const uint16_t centerX = (23 + 36) / 2; 
    const uint16_t centerY = 191 / 2;       


    LCDD display(1, 2, 3, 4, 5, 6);
    display.Initial_ST7305();
    display.Clear_Screen(bgColor);


    int prevWidth = 0;
    int16_t prevStartX = 0, prevEndX = 0;

    while(true) {

        double clampedValue = constrain(roll, -maxValue, maxValue);
        
  
        double scaledValue = tanh(k * -clampedValue) / tanh(k * maxValue);
        int dynamicWidth = scaledValue * 60; 
        uint16_t rectWidth = abs(dynamicWidth);

        
        uint16_t newStartX = centerX - rectWidth/2;
        uint16_t newEndX = newStartX + rectWidth - 1;

        
        if(prevWidth > 0) {
            display.Draw_Rectangle(
                prevStartX, 
                centerY - rectHeight/2,
                prevEndX,
                centerY + rectHeight/2 - 1,
                bgColor
            );
        }

        
        display.Draw_Rectangle(
            newStartX,
            centerY - rectHeight/2,
            newEndX,
            centerY + rectHeight/2 - 1,
            fgColor
        );

       
        display.Draw_Rectangle(
            centerX - 2,
            centerY - 1,
            centerX + 2,
            centerY + 1,
            fgColor
        );

        
        prevWidth = rectWidth;
        prevStartX = newStartX;
        prevEndX = newEndX;

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup()
{

  sdMutex = xSemaphoreCreateMutex();
  ThisDevice = new BLEDevice();
  ThisDevice->init("SailTracker");
  ThisDevice->setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  bleInitialized = true;

  Serial.begin(115200);
  xTaskCreate(GPS_Task, TAG_GPS, 4096, NULL, 10, NULL);
  xTaskCreate(IMU_Task, TAG_IMU, 4096, NULL, 11, NULL);
  xTaskCreate(SD_Card_Task, TAG_SD, 8192, NULL, 1, NULL);
  xTaskCreate(Compass_Task, TAG_COMPASS, 4096, NULL, 1, NULL);
  xTaskCreate(ePaper_Task, TAG_EPAPER, 4096, NULL, 1, NULL);
  xTaskCreate(Button_Task, "Button_Task", 4096, NULL, 12, NULL);
}

void loop()
{
  if(ItsOn) {
        static unsigned long lastKeepAlive = 0;
        if(millis() - lastKeepAlive > 5000) {
            // 发送保持活跃命令
            uint8_t keepAlive[] = {0x01, 0x05};
            // 需要实现获取characteristic的逻辑
            // pChar->writeValue(keepAlive, sizeof(keepAlive));
            lastKeepAlive = millis();
        }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
}