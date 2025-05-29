#include <SPI.h>
#include <SD.h>

// Define custom pins
const int chipSelect = 38; // CS pin for the SD card module
const int MOSI_PIN = 39;  // Custom MOSI pin
const int MISO_PIN = 41;  // Custom MISO pin
const int SCK_PIN = 40;   // Custom SCK pin

// Create an instance of SPIClass for software SPI
SPIClass mySPI = SPIClass(HSPI); // Use HSPI for software SPI

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to be ready
  }

  // Initialize SPI with custom pins
  mySPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, chipSelect);

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect, mySPI)) {
    Serial.println("Initialization failed!");
    return;
  }
  Serial.println("Initialization successful.");

  listFiles();
  readFile("11.txt"); // Change to the name of your file
}

void loop() {
  // Nothing to do here
}

void listFiles() {
  Serial.println("Listing files:");
  File root = SD.open("/");
  File file = root.openNextFile();

  while (file) {
    Serial.print("FILE: ");
    Serial.println(file.name());
    file = root.openNextFile();
  }
}

void readFile(const char* filename) {
  File file = SD.open(filename);

  if (file) {
    Serial.print("Reading file: ");
    Serial.println(filename);
    
    while (file.available()) {
      Serial.write(file.read());
    }
    file.close();
  } else {
    Serial.print("Failed to open file: ");
    Serial.println(filename);
  }
}