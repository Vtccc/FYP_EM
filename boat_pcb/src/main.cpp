#include <Arduino.h>
#include "esp_log.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "SD.h"
#include <QMC5883LCompass.h>
#include "SPI.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "IMU.h"
#include "registers.h"
#include "LCDD.h"
#include "BLEDevice.h"
#include "BLEClient.h"
#include "OneButton.h"

// Forward declarations
class MyClientCallback;
void BLE_Task(void* pvParameters);
bool connectToGoPro();
void sendCommand(uint8_t* command, size_t length);
void buttonClickHandler();
void handleDisconnection();
void handleKeepalive();
void handleRecordingTimeout();


// BLE Definitions
static const char* BLE_TAG = "BLE_Task";
static BLEUUID ServiceUUID((uint16_t)0xFEA6);
static BLEUUID CommandWriteCharacteristicUUID("b5f90072-aa8d-11e3-9046-0002a5d5c51b");

// Global BLE objects
static BLEDevice *ThisDevice;
static BLEClient *ThisClient;
static BLEScan *ThisScan;
static BLERemoteCharacteristic* pCommandCharacteristic = nullptr;
#define ServiceCharacteristic(S, C) ThisClient->getService(S)->getCharacteristic(C)

// Button
#define BUTTON_PIN 12
OneButton button(BUTTON_PIN, true);

// State management
volatile bool isRecording = false;
volatile bool isConnected = false;
uint32_t recordingStartTime = 0;
const uint32_t RECORDING_DURATION = 30000; // 30 seconds
const uint32_t KEEPALIVE_INTERVAL = 5000;  // 5 seconds
uint32_t lastKeepaliveTime = 0;

// Mutex for BLE operations
SemaphoreHandle_t bleMutex;

// BLE Client Callbacks implementation
class MyClientCallback : public BLEClientCallbacks {
public:
    void onConnect(BLEClient* pclient) override {
        ESP_LOGI(BLE_TAG, "Connected to GoPro");
        isConnected = true;

    }

    void onDisconnect(BLEClient* pclient) override {
        ESP_LOGW(BLE_TAG, "Disconnected from GoPro");
        isConnected = false;
        isRecording = false;
        digitalWrite(LED_BUILTIN, LOW);

    }
};

static const char *TAG_GPS = "GPS_Task";
static const char *TAG_IMU = "IMU_Task";
static const char *TAG_SD = "SD_Card_Task";
static const char *TAG_COMPASS = "Compass_Task";
static const char *TAG_EPAPER = "EPaper_Task";

// GPS
static const int RXPin = 43, TXPin = 44;
static const uint32_t GPSBaud = 38400;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
EspSoftwareSerial::UART ss;

// IMU - Using different pins
#define IMU_SDA 7  // Changed to match working example
#define IMU_SCL 8  // Changed to match working example
#define IMU_DRDY 9
TwoWire IMU_Wire = TwoWire(1);
ICM42688 IMU(IMU_Wire, 0x68, IMU_SDA, IMU_SCL);
String rollsString = "";
float ax, ay, az;  // Changed to float to match working example
float gx, gy, gz;  // Changed to float to match working example
double roll, pitch, yaw;

// Compass
int compass_azimuth;

// SD Card
File myFile;
int count = 0;
double rolls[10] = {400, 400, 400, 400, 400, 400, 400, 400, 400, 400};
#define chipSelect 38 // CS pin for the SD card module
#define MOSI_PIN 39  // Custom MOSI pin
#define MISO_PIN 41  // Custom MISO pin
#define SCK_PIN 40   // Custom SCK pin

// LCD pins
#define LCD_TE_PIN 1
#define LCD_RES_PIN 2
#define LCD_DC_PIN 3
#define LCD_CS_PIN 4
#define LCD_SCLK_PIN 5
#define LCD_SDI_PIN 6

LCDD lcd(LCD_TE_PIN, LCD_RES_PIN, LCD_DC_PIN, LCD_CS_PIN, LCD_SCLK_PIN, LCD_SDI_PIN);
// Mutex for I2C access
SemaphoreHandle_t i2cMutex = NULL;
const int centerX = (23+36)/2;        // 屏幕水平中心
const int centerY = 191/2;            // 屏幕垂直中心
const int rectHeight = 1;             // 矩形固定高度
const int maxValue = 50;              // 最大输入值
const double k = 0.05;                // 非线性缩放因子

const uint16_t COL_START = 0x17;      // 列起始地址23
const uint16_t COL_END = 0x24;        // 列结束地址36 (14列宽)
const uint16_t ROW_START = 0x00;      // 行起始地址0
const uint16_t ROW_END = 0xBF;        // 行结束地址191 (192行高)

// Function declarations
void GPS_Task(void *pvParameters);
void IMU_Task(void *pvParameters);
int64_t get_current_time_us();
String get_formatted_time();
String arrayToString(double *array, size_t size);
void scanI2C();
bool ScanAndConnect(void);
void BLE_Task(void* pvParameters);
bool connectToGoPro();
void sendCommand(uint8_t* command, size_t length);
void buttonClickHandler();
void handleDisconnection();
void handleKeepalive();
void handleRecordingTimeout();

// Task handles
TaskHandle_t gpsTaskHandle = NULL;
TaskHandle_t imuTaskHandle = NULL;
TaskHandle_t sdCardTaskHandle = NULL;
TaskHandle_t compassTaskHandle = NULL;
TaskHandle_t lcdTaskHandle = NULL;

// Task priorities
#define GPS_TASK_PRIORITY 2
#define IMU_TASK_PRIORITY 4  // Increased priority
#define SD_TASK_PRIORITY 1
#define COMPASS_TASK_PRIORITY 1
#define LCD_TASK_PRIORITY 3

// Task stack sizes
#define GPS_TASK_STACK 8192
#define IMU_TASK_STACK 8192
#define SD_TASK_STACK 8192
#define COMPASS_TASK_STACK 4096
#define LCD_TASK_STACK 8192



void GPS_Task(void *pvParameters) {
    ESP_LOGI(TAG_GPS, "GPS_Task");
    Serial1.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
    
    while (true) {
        while (Serial1.available() > 0) {
            gps.encode(Serial1.read());
        }
        // Serial.printf("gps");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void scanI2C() {
    Serial.println("Scanning I2C bus...");
    byte error, address;
    int deviceCount = 0;
    
    for(address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
            deviceCount++;
        }
    }
    
    if (deviceCount == 0) {
        Serial.println("No I2C devices found!");
    } else {
        Serial.print("Found ");
        Serial.print(deviceCount);
        Serial.println(" device(s)");
    }
}

void IMU_Task(void *pvParameters) {
    Serial.println("Starting IMU Task");

    // Create mutex for I2C access
    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL) {
        Serial.println("Failed to create I2C mutex");
        vTaskDelete(NULL);
        return;
    }

    // Initialize I2C
    Wire.begin(IMU_SDA, IMU_SCL);
    vTaskDelay(100);
    pinMode(IMU_SDA, INPUT_PULLUP);
    pinMode(IMU_SCL, INPUT_PULLUP);
    
    // Initialize IMU
    Serial.println("Beginning IMU");
    int status = IMU.begin();
    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        // vTaskDelete(NULL);
        return;
    }

    Serial.println("Configuring IMU");
    // Configure IMU
    IMU.setAccelFS(ICM42688::gpm8);    // ±8G
    IMU.setGyroFS(ICM42688::dps500);   // ±500dps
    IMU.setAccelODR(ICM42688::odr12_5);
    IMU.setGyroODR(ICM42688::odr12_5);

    Serial.println("IMU initialized successfully!");

    while (true) {
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            // Read IMU data
            IMU.getAGT();
            
            // Get accelerometer data
            ax = IMU.accX();
            ay = IMU.accY();
            az = IMU.accZ();
            
            // Get gyroscope data
            gx = IMU.gyrX();
            gy = IMU.gyrY();
            gz = IMU.gyrZ();

            xSemaphoreGive(i2cMutex);

            // Calculate roll, pitch, yaw
            roll = -(atan2(ay, az) * 180.0 / M_PI);
            pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
            yaw += (gz * 0.01);           

            // Print data
            Serial.print("AccX: ");
            Serial.println(ax);
            Serial.print("AccY: ");
            Serial.println(ay);
            Serial.print("AccZ: ");
            Serial.println(az);

            Serial.print("GyroX: ");
            Serial.println(gx);
            Serial.print("GyroY: ");
            Serial.println(gy);
            Serial.print("GyroZ: ");
            Serial.println(gz);

            Serial.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll, pitch, yaw);
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);  // Changed to 500ms to match working example
    }
}

void SD_Card_Task(void *pvParameters)
{
    Serial.println("Starting SD Card Task");
    
    // Initialize SPI for SD card
    SPIClass myspi = SPIClass(HSPI);
    myspi.begin(SCK_PIN, MISO_PIN, MOSI_PIN, chipSelect);
    
    // Try to initialize SD card with retries
    int retryCount = 0;
    while (!SD.begin(chipSelect, myspi)) 
    {
        Serial.print("SD Card initialization failed, attempt ");
        Serial.print(retryCount + 1);
        Serial.println("/5");
        
        if (retryCount >= 4) {
            Serial.println("SD Card initialization failed after 5 attempts!");
            vTaskDelete(NULL);
            return;
        }
        retryCount++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    Serial.println("SD Card initialized successfully!");

    // Find the smallest available file number
    int fileNumber = 0;
    char filename[30];
    while (true)
    {
        sprintf(filename, "/sailing_data%03d.txt", fileNumber);
        if (!SD.exists(filename))
        {
            Serial.print("Creating new file: ");
            Serial.println(filename);
            break;
        }
        fileNumber++;
    }

    // Open the file and write the header
    myFile = SD.open(filename, FILE_WRITE);
    if (!myFile)
    {
        Serial.println("Failed to open file for writing!");
        vTaskDelete(NULL);
        return;
    }

    Serial.println("Writing file header...");
    
    // Write file header
    myFile.println("{");
    myFile.println("refreshRate=10;");
    myFile.println("version=1.0;");
    myFile.println("format=[timestamp,gps.satellites,gps.hdop,gps.location.age,gps.lat,gps.lng,gps.speed,gps.course,gps.month,gps.day,gps.year,gps.hour,gps.minute,gps.second,gps.centisecond,compass,roll,pitch,yaw]");
    myFile.println("}");
    myFile.flush(); // Ensure header is written
    
    Serial.println("File header written successfully");

    while (true)
    {
        // Get current time
        String current_time = get_formatted_time();

        // Prepare data to write
        size_t rollsSize = sizeof(rolls) / sizeof(rolls[0]);
        rollsString = arrayToString(rolls, rollsSize);

        // Write data to file
        myFile.printf("%s,%d,%f,%d,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f\n",
                    current_time.c_str(), gps.satellites.value(), gps.hdop.hdop(), gps.location.age(),
                    gps.location.lat(), gps.location.lng(), gps.speed.mps(), gps.course.deg(),
                    gps.date.month(), gps.date.day(), gps.date.year(), gps.time.hour(),
                    gps.time.minute(), gps.time.second(), gps.time.centisecond(),
                    compass_azimuth, roll, pitch, yaw);

        // Flush data to SD card
        myFile.flush();

        // Print time to Serial
        // Serial.print("Data written at: ");
        // Serial.println(current_time);

        vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust delay to ensure 10 records per second
    }
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

void LCD_Task(void *pvParameters) {
    lcd.Initial_ST7305();
    lcd.Clear_Screen(0x0000); 
    lcd.Draw_Rectangle(centerX, centerY, centerX+4, centerY, true); // Draw center line
    lcd.Draw_Rectangle(25,185,36,191,0xFFFF);
    
    while (true) {
        // Display GPS time
        int hours, minutes;
        
        if (gps.time.isValid()) {
            hours = gps.time.hour() + 7;  // Add 7 hours for timezone
            minutes = gps.time.minute();
        } else {
            hours = 0;
            minutes = 0;
        }
        
        // Draw hours (tens place)
        lcd.Draw_Rotated_Number(0, 23, 5);
        
        // Draw hours (ones place)
        lcd.Draw_Rotated_Number(hours % 10, 23, 13);
        
        // Draw colon
        lcd.Draw_Rotated_Number(10, 23, 21); // Assuming 10 is the colon in your font array
        
        // Draw minutes (tens place)
        lcd.Draw_Rotated_Number(minutes / 10, 23, 29);
        
        // Draw minutes (ones place)
        lcd.Draw_Rotated_Number(minutes % 10, 23, 37);
        
        // Draw balance indicator
        int rollValue = (int)(pitch); // Scale roll for better display
        lcd.Draw_Balance_Indicator(rollValue);
        
        vTaskDelay(25 / portTICK_PERIOD_MS); // Update every 100ms
    }
}

void BLE_Task(void* pvParameters) {
    ESP_LOGI(BLE_TAG, "Starting BLE Task");
    
    // Initialize BLE with retries
    int initRetries = 0;
    while (initRetries < 3) {
        try {
            ThisDevice = new BLEDevice();
            ThisDevice->init("");
            ThisDevice->setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
            break;
        } catch (...) {
            ESP_LOGE(BLE_TAG, "BLE initialization failed, retry %d/3", initRetries + 1);
            initRetries++;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    
    if (initRetries >= 3) {
        ESP_LOGE(BLE_TAG, "BLE initialization failed after 3 attempts");
        vTaskDelete(NULL);
        return;
    }
    
    ThisClient = ThisDevice->createClient();
    ThisClient->setClientCallbacks(new MyClientCallback());
    
    ThisScan = ThisDevice->getScan();
    ThisScan->setInterval(100);  // Set scan interval
    ThisScan->setWindow(99);     // Set scan window
    
    // Initialize button
    button.attachClick(buttonClickHandler);
    
    // Try initial connection with retries
    int connectionRetries = 0;
    while (connectionRetries < 3) {
        if (ScanAndConnect()) {
            ESP_LOGI(BLE_TAG, "Connected to BLE device successfully");
            break;
        }
        ESP_LOGE(BLE_TAG, "Connection attempt %d/3 failed", connectionRetries + 1);
        connectionRetries++;
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    
    if (connectionRetries >= 3) {
        ESP_LOGE(BLE_TAG, "Initial connection failed after 3 attempts!");
    }
    
    while (true) {
        // Handle button events
        button.tick();
        
        // Check connection status
        if (!ThisClient->isConnected() && isConnected) {
            ESP_LOGW(BLE_TAG, "Device disconnected!");
            isConnected = false;
            isRecording = false;
            
            // Try to reconnect with exponential backoff
            static uint32_t reconnectDelay = 1000;
            ESP_LOGI(BLE_TAG, "Attempting to reconnect in %d ms...", reconnectDelay);
            vTaskDelay(reconnectDelay / portTICK_PERIOD_MS);
            
            if (ScanAndConnect()) {
                ESP_LOGI(BLE_TAG, "Reconnected successfully");
                reconnectDelay = 1000;  // Reset delay on successful connection
            } else {
                reconnectDelay = (reconnectDelay * 2 > 10000) ? 10000 : reconnectDelay * 2;  // Max 10 second delay
            }
        }
        
        if (isRecording) {
            // Check for recording timeout
            if (millis() - recordingStartTime >= RECORDING_DURATION) {
                uint8_t stopCommand[] = {0x03, 0x01, 0x01, 0x00};
                ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue({0x03, 0x01, 0x01, 0x00});
                isRecording = false;
                ESP_LOGI(BLE_TAG, "Recording stopped after 30 seconds");
            }
            // Send keepalive
            else if (millis() - lastKeepaliveTime >= KEEPALIVE_INTERVAL) {
                uint8_t keepaliveCommand[] = {0x01, 0x05};
                ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue({0x01, 0x05});
                lastKeepaliveTime = millis();
                ESP_LOGI(BLE_TAG, "Sending keep-alive");
            }
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

bool ScanAndConnect(void) {
    ThisScan->clearResults();
    ThisScan->start(5, false);  // Increased scan duration to 5 seconds
    
    for (int i = 0; i < ThisScan->getResults().getCount(); i++) {
        BLEAdvertisedDevice device = ThisScan->getResults().getDevice(i);
        if (device.haveServiceUUID() && device.isAdvertisingService(BLEUUID(ServiceUUID))) {
            ESP_LOGI(BLE_TAG, "Found target device: %s", device.toString().c_str());
            ThisScan->stop();
            
            // Try to connect with timeout
            if (ThisClient->connect(&device)) {
                // Wait for connection to establish
                int connectTimeout = 0;
                while (!ThisClient->isConnected() && connectTimeout < 50) {
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    connectTimeout++;
                }
                
                if (ThisClient->isConnected()) {
                    isConnected = true;
                    return true;
                }
            }
        }
    }
    
    ThisScan->stop();
    return false;
}

void sendCommand(uint8_t* command, size_t length) {
    if (xSemaphoreTake(bleMutex, portMAX_DELAY) == pdTRUE) {
        if (isConnected && pCommandCharacteristic != nullptr) {
            pCommandCharacteristic->writeValue(command, length);
        }
        xSemaphoreGive(bleMutex);
    }
}

void buttonClickHandler() {
    ESP_LOGI(BLE_TAG, "button clicked");
    if (isConnected) {
        if (!isRecording) {
            ESP_LOGI(BLE_TAG, "Button pressed - Sending start command");
            
            // Send start recording command
            uint8_t startCommand[] = {0x03, 0x01, 0x01, 0x01};
            ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue({0x03, 0x01, 0x01, 0x01});
            
            isRecording = true;
            digitalWrite(LED_BUILTIN, HIGH);
            recordingStartTime = millis();
   
            ESP_LOGI(BLE_TAG, "Recording started");
        } else {
            ESP_LOGI(BLE_TAG, "Button pressed - Sending stop command");
            
            // Send stop recording command
            uint8_t stopCommand[] = {0x03, 0x01, 0x01, 0x00};
            ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue({0x03, 0x01, 0x01, 0x00});
            
            isRecording = false;
            digitalWrite(LED_BUILTIN, LOW);
   
            ESP_LOGI(BLE_TAG, "Recording stopped");
        }
    } else {
        ESP_LOGW(BLE_TAG, "Not connected to device!");

    }
}

void handleDisconnection() {
    static uint32_t lastReconnectAttempt = 0;
    const uint32_t RECONNECT_INTERVAL = 10000; // 10 seconds
    
    if (millis() - lastReconnectAttempt > RECONNECT_INTERVAL) {
        ESP_LOGI(BLE_TAG, "Attempting to reconnect...");
        if (connectToGoPro()) {
            ESP_LOGI(BLE_TAG, "Reconnection successful");
            digitalWrite(LED_BUILTIN, HIGH);
            
            // Restart recording if we were in the middle of a session
            if (isRecording) {
                      ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue({0x03, 0x01, 0x01, 0x01});
                recordingStartTime = millis(); // Reset timer
                ESP_LOGI(BLE_TAG, "starting recording at: %d", recordingStartTime);
            }
        }
        lastReconnectAttempt = millis();
    }
}

void handleKeepalive() {
    if (millis() - lastKeepaliveTime > KEEPALIVE_INTERVAL) {
        uint8_t keepaliveCommand[] = {0x01, 0x05};
        sendCommand(keepaliveCommand, sizeof(keepaliveCommand));
        lastKeepaliveTime = millis();
        ESP_LOGD(BLE_TAG, "Keepalive sent");
    }
}



// RTC
int64_t get_current_time_us() {
    return esp_timer_get_time();
}

// Function to format time as a string
String get_formatted_time() {
    int64_t current_time_us = get_current_time_us();
    int64_t seconds = current_time_us / 1000000;
    int64_t microseconds = current_time_us % 1000000;
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%lld.%06lld", seconds, microseconds);
    return String(buffer);
}

String arrayToString(double *array, size_t size) {
    String result = "[";
    for (size_t i = 0; i < size; ++i) {
        result += String(array[i], 2);
        if (i < size - 1) {
            result += ",";
        }
    }
    result += "]";
    return result;
}

// Add this function after the arrayToString function

void setup() {

    pinMode(6, OUTPUT);
    Serial.begin(115200);
    vTaskDelay(10);  // Give time for serial to initialize

    // pinMode(6, OUTPUT);
    Serial.begin(115200);
    delay(1000);  // Give time for serial to initialize

    
    Serial.println("Starting setup...");
    
    
    // Initialize LCD

    // lcd.Initial_ST7305();
    // lcd.Clear_Screen(0x0000); // Clear screen with black

    // LCDD lcd(LCD_TE_PIN, LCD_RES_PIN, LCD_DC_PIN, LCD_CS_PIN, LCD_SCLK_PIN, LCD_SDI_PIN);
    // // lcd.Initial_ST7305();
    // // lcd.Clear_Screen(0x0000); // Clear screen with black
    // lcd.Clear_Screen(0xffff);

    // Initialize BLE mutex
    bleMutex = xSemaphoreCreateMutex();
    if (bleMutex == NULL) {
        Serial.println("Failed to create BLE mutex!");
        return;
    }
    
    // Setup button
    button.attachClick(buttonClickHandler);
    
    // Create BLE task on core 0 with high priority

    xTaskCreatePinnedToCore(BLE_Task, "BLE_Task", 20000, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(IMU_Task, "IMU_Task", 20000, NULL, IMU_TASK_PRIORITY, &imuTaskHandle, 1);
    xTaskCreatePinnedToCore(GPS_Task, "GPS_Task", GPS_TASK_STACK, NULL, GPS_TASK_PRIORITY, &gpsTaskHandle, 1);
    xTaskCreatePinnedToCore(SD_Card_Task, "SD_Card_Task", SD_TASK_STACK, NULL, SD_TASK_PRIORITY, &sdCardTaskHandle, 1);
    xTaskCreatePinnedToCore(Compass_Task, "Compass_Task", COMPASS_TASK_STACK, NULL, COMPASS_TASK_PRIORITY, &compassTaskHandle, 1);
    xTaskCreatePinnedToCore(LCD_Task, "LCD_Task", LCD_TASK_STACK, NULL, LCD_TASK_PRIORITY, &lcdTaskHandle, 1);
    Serial.println("Setup complete!");
}

void loop() {
    // button.tick();  // Handle button presses
    // vTaskDelay(10 / portTICK_PERIOD_MS);
}