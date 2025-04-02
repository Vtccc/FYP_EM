#include <Arduino.h>
#include "esp_log.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include "SD.h"
#include <QMC5883LCompass.h>
#include "SPI.h"
#include "esp_sntp.h"
#include "esp_timer.h"

// E-paper library
#include "DEV_Config.h"
#include "EPD.h"
#include "GUI_Paint.h"
#include "ImageData.h"
#include <stdlib.h>

static const char *TAG_GPS = "GPS_Task";
static const char *TAG_IMU = "IMU_Task";
static const char *TAG_SD = "SD_Card_Task";
static const char *TAG_COMPASS = "Compass_Task";
static const char *TAG_EPAPER = "EPaper_Task";

// GPS
static const int RXPin = 41, TXPin = 42;
static const uint32_t GPSBaud = 38400;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
EspSoftwareSerial::UART ss;

// IMU
Adafruit_ICM20948 icm;
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

void IMU_Task(void *pvParameters)
{
  ESP_LOGI(TAG_IMU, "IMU_Task");

  Wire.begin(21, 20);
  icm.begin_I2C(0x68, &Wire);

  sensors_event_t accel, gyro, temp;

  while (true)
  {
    // Get sensor events
    icm.getEvent(&accel, &gyro, &temp);

    // Store accelerometer and gyroscope values
    ax = accel.acceleration.x;
    ay = accel.acceleration.y;
    az = accel.acceleration.z;

    gx = gyro.gyro.x;
    gy = gyro.gyro.y;
    gz = gyro.gyro.z;

    // Compute Euler angles (roll, pitch, yaw)
    roll = -(atan2(ay, az) * 180.0 / PI);                        
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI; 
    yaw += (gz * 0.01);

    // Store roll values during 1 second
    // rolls[count++ % 10] = roll;
    // for (int i = 0; i < 10; i++)
    // {
    //   Serial.printf("%f," ,rolls[i]);
    // }
    // Serial.printf("\n");
    Serial.printf("%f,",roll);

    vTaskDelay(100 / portTICK_PERIOD_MS);
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
  myFile.println("{");
  myFile.println("refreshRate=10;");
  myFile.println("version=1.0;");
  myFile.println("format=[timestamp,gps.satellites,gps.hdop,gps.location.age,gps.lat,gps.lng,gps.speed,gps.course,gps.month,gps.day,gps.year,gps.hour,gps.minute,gps.second,gps.centisecond,compass,roll,pitch,yaw]");
  myFile.println("}");
  myFile.flush(); // Ensure header is written

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
    Serial.print("Current Time: ");
    Serial.println(current_time);

    // Reset the rolls array and count
    // for (int i = 0; i < 10; i++)
    // {
    //   rolls[i] = 400;
    // }
    // count = 0;
    // rollsString.clear();

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
  const int rectHeight = 40;
  const int lineLength = 60;
  const double maxAngle = 50.0;
  const double k = 0.05; // Non-linear scaling factor

  DEV_Module_Init();

  EPD_2in13_V4_Init();
  EPD_2in13_V4_Clear();

  // Create a new image cache
  UBYTE *BlackImage;
  UWORD Imagesize = ((EPD_2in13_V4_WIDTH % 8 == 0) ? (EPD_2in13_V4_WIDTH / 8) : (EPD_2in13_V4_WIDTH / 8 + 1)) * EPD_2in13_V4_HEIGHT;
  if ((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL)
  {
    Serial.println(F("Failed to apply for black memory..."));
    while (1)
      ;
  }
  Paint_NewImage(BlackImage, EPD_2in13_V4_WIDTH, EPD_2in13_V4_HEIGHT, 90, WHITE);
  Paint_Clear(WHITE);

  Paint_NewImage(BlackImage, EPD_2in13_V4_WIDTH, EPD_2in13_V4_HEIGHT, 90, WHITE);
  Paint_SelectImage(BlackImage);

  while (true)
  {
    // Implenment non-linear scaling of roll to rectWidth
    if (roll < -maxAngle)
      roll = -maxAngle;
    if (roll > maxAngle)
      roll = maxAngle;

    double scaledRoll = tanh(k * -roll) / tanh(k * maxAngle);
    int rectWidth = (int)(scaledRoll * (EPD_2in13_V4_HEIGHT / 2));

    // Clear the previous rectangle
    Paint_ClearWindows(0, EPD_2in13_V4_WIDTH / 2 - rectHeight / 2 - 2, EPD_2in13_V4_HEIGHT, EPD_2in13_V4_WIDTH / 2 + rectHeight / 2 + 2, WHITE);

    // Draw the new rectangle and center line
    Paint_DrawRectangle(EPD_2in13_V4_HEIGHT / 2, EPD_2in13_V4_WIDTH / 2 - rectHeight / 2, EPD_2in13_V4_HEIGHT / 2 + rectWidth, EPD_2in13_V4_WIDTH / 2 + rectHeight / 2, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawLine(EPD_2in13_V4_HEIGHT / 2, EPD_2in13_V4_WIDTH / 2 - lineLength / 2, EPD_2in13_V4_HEIGHT / 2, EPD_2in13_V4_WIDTH / 2 + lineLength / 2, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

    // Partial refresh of the display
    EPD_2in13_V4_Display_Partial(BlackImage);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  xTaskCreate(GPS_Task, TAG_GPS, 4096, NULL, 10, NULL);
  xTaskCreate(IMU_Task, TAG_IMU, 4096, NULL, 11, NULL);
  xTaskCreate(SD_Card_Task, TAG_SD, 8192, NULL, 1, NULL);
  xTaskCreate(Compass_Task, TAG_COMPASS, 4096, NULL, 1, NULL);
  xTaskCreate(ePaper_Task, TAG_EPAPER, 4096, NULL, 1, NULL);
}

void loop()
{
  vTaskDelete(NULL);
}