#include <Arduino.h>

// Define UART pins and baud rate
static const int RXPin = 41; // RX pin for the module
static const int TXPin = 42; // TX pin for the module
static const uint32_t ModuleBaudRate = 115200;
static const uint32_t USBBaudRate = 115200;

// Create a HardwareSerial instance for the module connection
HardwareSerial SerialModule(1);

void setup()
{
  // Initialize USB Serial for communication with the computer
  Serial.begin(USBBaudRate);
  while (!Serial)
  {
    // Wait for Serial to initialize
    delay(10);
  }
  Serial.println("ESP32 Serial Monitor and Passthrough Initialized");

  // Initialize the UART for the module connection
  SerialModule.begin(ModuleBaudRate, SERIAL_8N1, RXPin, TXPin);
}

void loop()
{
  // Pass data from the module to the computer (USB Serial)
  if (SerialModule.available())
  {
    char incoming = SerialModule.read();
    Serial.write(incoming); // Forward to USB Serial
    Serial.print(incoming); // Echo back to monitor for observation
  }

  // Pass data from the computer (USB Serial) to the module
  if (Serial.available())
  {
    char incoming = Serial.read();
    SerialModule.write(incoming); // Forward to the module
    Serial.print(incoming);       // Echo back to monitor for observation
  }
}
