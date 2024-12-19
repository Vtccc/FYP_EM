#include <Arduino.h>

// Define UART pins and baud rate
static const int RXPin = 41; // RX pin for GPS module
static const int TXPin = 42; // TX pin for GPS module
static const uint32_t GPSBaudRate = 38400;   // GPS chip baud rate
static const uint32_t USBPassthroughBaud = 38400; // USB Serial baud rate for u-center

// Create a HardwareSerial instance for the GPS module
HardwareSerial SerialGPS(1);

void setup() {
  // Initialize USB Serial (for u-center)
  Serial.begin(USBPassthroughBaud);
  while (!Serial) {
    delay(10); // Wait for Serial to initialize
  }
  Serial.println("ESP32 GPS Passthrough Initialized");

  // Initialize UART for the GPS module
  SerialGPS.begin(GPSBaudRate, SERIAL_8N1, RXPin, TXPin);
}

void loop() {
  // Pass data from the GPS module to the computer (u-center)
  if (SerialGPS.available()) {
    Serial.write(SerialGPS.read());
  }

  // Pass data from the computer (u-center) to the GPS module
  if (Serial.available()) {
    SerialGPS.write(Serial.read());
  }
}
