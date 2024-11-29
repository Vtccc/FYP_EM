#include <Wire.h>
#include <Adafruit_ICM20948.h>

// Create an ICM20948 object
Adafruit_ICM20948 icm;

void setup() {
Serial.begin(115200);
delay(1000);

// Initialize I²C with custom SDA and SCL pins
Wire.begin(38, 37); // SDA = GPIO25, SCL = GPIO26

// Try to initialize the ICM20948
if (!icm.begin_I2C(0x68, &Wire)) { // Default I²C address is 0x69
Serial.println("Failed to find ICM20948 chip");
while (1) {
delay(10);
}
}
Serial.println("ICM20948 found and initialized!");

// Configure the accelerometer, gyroscope, and magnetometer
icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
}

void loop() {
// Read accelerometer
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mag;
sensors_event_t temp;

// Get sensor events
icm.getEvent(&accel, &gyro, &mag, &temp);

// Print accelerometer data
Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" m/s^2, ");
Serial.print("Y: "); Serial.print(accel.acceleration.y); Serial.print(" m/s^2, ");
Serial.print("Z: "); Serial.print(accel.acceleration.z); Serial.println(" m/s^2");

// Print gyroscope data
Serial.print("Gyro X: "); Serial.print(gyro.gyro.x); Serial.print(" rad/s, ");
Serial.print("Y: "); Serial.print(gyro.gyro.y); Serial.print(" rad/s, ");
Serial.print("Z: "); Serial.print(gyro.gyro.z); Serial.println(" rad/s");

// Print magnetometer data
Serial.print("Mag X: "); Serial.print(mag.magnetic.x); Serial.print(" uT, ");
Serial.print("Y: "); Serial.print(mag.magnetic.y); Serial.print(" uT, ");
Serial.print("Z: "); Serial.print(mag.magnetic.z); Serial.println(" uT");

delay(1000); // Delay for readability
}