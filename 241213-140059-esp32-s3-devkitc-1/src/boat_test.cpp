#include <Arduino.h>
#include "esp_log.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include "SD.h"
#include <QMC5883LCompass.h>
#include "SPI.h"

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

// GPS
static const int RXPin = 41, TXPin = 42;
static const uint32_t GPSBaud = 38400;
// The TinyGPSPlus object
TinyGPSPlus gps;
// The serial connection to the GPS device
EspSoftwareSerial::UART ss;

// IMU
Adafruit_ICM20948 icm;
int16_t ax, ay, az;
int16_t gx, gy, gz;

int compass_azimuth;


// SD Card
#define PIN_SPI_CS 10 // The ESP32 pin GPIO5
File myFile;

void displayInfo()
{
  static char logBuffer[256];
  static char satellites[16], location[32], date[16], time[16];

  if (gps.satellites.isValid()) {
    sprintf(satellites, "%d", gps.satellites.value());
  } else {
    strcpy(satellites, "INVALID");
  }

  if (gps.location.isValid()) {
    sprintf(location, "%.6f,%.6f", gps.location.lat(), gps.location.lng());
  } else {
    strcpy(location, "INVALID");
  }

  if (gps.date.isValid()) {
    sprintf(date, "%d/%d/%d", gps.date.month(), gps.date.day(), gps.date.year());
  } else {
    strcpy(date, "INVALID");
  }

  if (gps.time.isValid()) {
    sprintf(time, "%02d:%02d:%02d.%02d", gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond());
  } else {
    strcpy(time, "INVALID");
  }

  sprintf(logBuffer, "displayInfo, Satellites: %s, Location: %s, Date/Time: %s, Time: %s", satellites, location, date, time);

  ESP_LOGI(TAG_GPS, "%s", logBuffer);
}


void GPS_Task(void *pvParameters) {
  ESP_LOGI(TAG_GPS, "GPS_Task");
  Serial1.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  while (true) {
    while (Serial1.available()) {
      if (gps.encode(Serial1.read())) {
        displayInfo();
      }
    }
    ESP_LOGI(TAG_GPS, "High water mark of GPS_Task: %d", uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void IMU_Task(void *pvParameters) {
  ESP_LOGI(TAG_IMU, "IMU_Task");

  Wire.begin(21, 20);
  icm.begin_I2C(0x68, &Wire);

  sensors_event_t accel, gyro, temp;

  static char acceleration[20], gyroscopic[20];

  while (true) {
    // Get sensor events
    icm.getEvent(&accel, &gyro, &temp);

    // Store accelerometer and gyroscope values
    ax = accel.acceleration.x;
    ay = accel.acceleration.y;
    az = accel.acceleration.z;

    gx = gyro.gyro.x;
    gy = gyro.gyro.y;
    gz = gyro.gyro.z;

    // Log the data
    Serial.print("Accel (m/s^2): ");
    Serial.print(ax);
    Serial.print(", ");
    Serial.print(ay);
    Serial.print(", ");
    Serial.print(az);
    Serial.print("    ");
    Serial.print("Gyro (rps): ");
    Serial.print(gx);
    Serial.print(", ");
    Serial.print(gy);
    Serial.print(", ");
    Serial.println(gz);

    ESP_LOGI(TAG_IMU, "Accel (m/s^2): %.2f, %.2f, %.2f", ax, ay, az);
    ESP_LOGI(TAG_IMU, "Gyro (rps): %.2f, %.2f, %.2f", gx, gy, gz);

    ESP_LOGI(TAG_IMU, "High water mark of IMU_Task: %d", uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void SD_Card_Task(void *pvParameters) {
  SPIClass myspi = SPIClass(HSPI);
  myspi.begin(12, 13, 11, 10);
  while (!SD.begin(10)){
    ESP_LOGE(TAG_SD, "SD CARD FAILED, OR NOT PRESENT! Retrying...");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second before retrying
  }
  ESP_LOGI(TAG_SD, "SD CARD OK!");
  // Find the smallest available file number using C style file functions
  int fileNumber = 0;
  char filename[30];
  while (true) {
    sprintf(filename, "/sailing_data%03d.txt", fileNumber);
    if (!SD.exists(filename)) {
      myFile = SD.open(filename, FILE_WRITE);
      myFile.close();
      break;
    }
    fileNumber++;
  }
  while (true) {
    // Serial.println("SD_Card_Task");
    if (SD.exists(filename)) {
      Serial.println(F("File exists on SD Card. Now appending data to it."));
      // create a new file by opening a new file and immediately close it
      myFile = SD.open(filename, FILE_APPEND);
      // printf to the file as a csv: ax, ay, az, gx, gy, gz, gps.location.lat(), gps.location.lng(), gps.speed.mps(), gps.date.month(), gps.date.day(), gps.date.year(), gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond(), WIND_DEGREE, WIND_REGION
      myFile.printf("%d,%d,%d,%d,%d,%d,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d\n", ax, ay, az, gx, gy, gz, gps.location.lat(), gps.location.lng(), gps.speed.mps(), gps.date.month(), gps.date.day(), gps.date.year(), gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond(), compass_azimuth);
      ESP_LOGI(TAG_SD, "Data appended to file.");
      myFile.close();
    } else {
      ESP_LOGE(TAG_SD, "File %s does not exist on SD Card.", filename);
    }
    ESP_LOGI(TAG_SD, "High water mark of SD_Card_Task: %d", uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void Compass_Task(void *pvParameters) {
  QMC5883LCompass compass;
  compass.init();
  while (1) {
    // Read compass values
    compass.read();

    // Return Azimuth reading
    compass_azimuth = compass.getAzimuth();
    ESP_LOGI(TAG_COMPASS, "Compass_Task, Azimuth: %d", compass_azimuth);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void ePaper_Task(void *pvParameters) {
  DEV_Module_Init();

  EPD_2in13_V4_Init();
	EPD_2in13_V4_Clear();
  
  // Create a new image cache
	UBYTE *BlackImage;
	UWORD Imagesize = ((EPD_2in13_V4_WIDTH % 8 == 0) ? (EPD_2in13_V4_WIDTH / 8) : (EPD_2in13_V4_WIDTH / 8 + 1)) * EPD_2in13_V4_HEIGHT;
	if ((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL)
	{
		printf("Failed to apply for black memory...\r\n");
		while (1)
			;
	}

  Paint_NewImage(BlackImage, EPD_2in13_V4_WIDTH, EPD_2in13_V4_HEIGHT, 90, WHITE);
	Debug("Partial refresh\r\n");
	Paint_SelectImage(BlackImage);

	PAINT_TIME sPaint_time;
	sPaint_time.Hour = 12;
	sPaint_time.Min = 34;
	sPaint_time.Sec = 56;
	UBYTE num = 10;

  while (1) {
    // Display data on ePaper

    sPaint_time.Sec = sPaint_time.Sec + 1;
		if (sPaint_time.Sec == 60)
		{
			sPaint_time.Min = sPaint_time.Min + 1;
			sPaint_time.Sec = 0;
			if (sPaint_time.Min == 60)
			{
				sPaint_time.Hour = sPaint_time.Hour + 1;
				sPaint_time.Min = 0;
				if (sPaint_time.Hour == 24)
				{
					sPaint_time.Hour = 0;
					sPaint_time.Min = 0;
					sPaint_time.Sec = 0;
				}
			}
		}
		Paint_ClearWindows(150, 80, 150 + Font20.Width * 7, 80 + Font20.Height, WHITE);
		Paint_DrawTime(150, 80, &sPaint_time, &Font20, WHITE, BLACK);

		// num = num - 1;
		// if (num == 0)
		// {
		// 	break;
		// }
		EPD_2in13_V4_Display_Partial(BlackImage);
		// DEV_Delay_ms(500); // Analog clock 1s
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  xTaskCreate(GPS_Task, TAG_GPS, 4096, NULL, 10, NULL);
  delay(200);
  xTaskCreate(IMU_Task, TAG_IMU, 4096, NULL, 11, NULL);
  delay(200);
  xTaskCreate(SD_Card_Task, TAG_SD, 8192, NULL, 1, NULL);
  delay(200);
  xTaskCreate(Compass_Task, TAG_COMPASS, 4096, NULL, 1, NULL);

  ESP_LOGI("setup", "setup");
}

void loop() {
  vTaskDelete(NULL);
}