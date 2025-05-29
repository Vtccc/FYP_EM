// #include "ICM42688.h"
#include "IMU.h"

#define IMU_SDA 6
#define IMU_SCL 7
#define IMU_DRDY 12

ICM42688 IMU(Wire, 0x68, IMU_SDA, IMU_SCL);





void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  int status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  // setting the accelerometer full scale range to +/-8G
  IMU.setAccelFS(ICM42688::gpm8);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroFS(ICM42688::dps500);

  // set output data rate to 12.5 Hz
  IMU.setAccelODR(ICM42688::odr12_5);
  IMU.setGyroODR(ICM42688::odr12_5);
}

void loop() {
  // put your main code here, to run repeatedly:
  IMU.getAGT();
  /*Accelerometer Data*/
  float accX = IMU.accX();
  float accY = IMU.accY();
  float accZ = IMU.accZ();
  /*Gyroscope Data*/
  float gyroX = IMU.gyrX();
  float gyroY = IMU.gyrY();
  float gyroZ = IMU.gyrZ();


  Serial.print("AccX: ");
  Serial.println(accX);
  Serial.print("AccY: ");
  Serial.println(accY);
  Serial.print("AccZ: ");
  Serial.println(accZ);

  Serial.print("GyroX: ");
  Serial.println(gyroX);
  Serial.print("GyroY: ");
  Serial.println(gyroY);
  Serial.print("GyroZ: ");
  Serial.println(gyroZ);



  delay(500);
}
