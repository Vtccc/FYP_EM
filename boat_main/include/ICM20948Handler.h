#ifndef ICM20948HANDLER_H
#define ICM20948HANDLER_H
#include <Arduino.h>
#include "Wire.h"
#include "ICM_20948.h"

extern TaskHandle_t IMU_task_handle;
extern double roll, pitch, yaw;

#define SERIAL_PORT Serial
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 0

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

void IMU_Task(void *pvParameters);

void initICM20948()
{
	SERIAL_PORT.begin(115200);
	WIRE_PORT.begin(7, 6, 400000);

	bool initialized = false;
	while (!initialized)
	{
		myICM.begin(WIRE_PORT, AD0_VAL);

#ifndef QUAT_ANIMATION
		SERIAL_PORT.print(F("Initialization of the sensor returned: "));
		SERIAL_PORT.println(myICM.statusString());
#endif
		if (myICM.status != ICM_20948_Stat_Ok)
		{
#ifndef QUAT_ANIMATION
			SERIAL_PORT.println(F("Trying again..."));
#endif
			delay(500);
		}
		else
		{
			initialized = true;
		}
	}
	SERIAL_PORT.println(F("Device connected!"));

	bool success = true; // Use success to show if the DMP configuration was successful

	// Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
	success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

	// DMP sensor options are defined in ICM_20948_DMP.h
	//    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
	//    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
	//    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
	//    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
	//    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
	//    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
	//    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
	//    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
	//    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
	//    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
	//    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
	//    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
	//    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
	//    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
	//    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

	// Enable the DMP Game Rotation Vector sensor
	success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

	// Enable any additional sensors / features
	// success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
	// success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
	// success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

	// Configuring DMP to output data at multiple ODRs:
	// DMP is capable of outputting multiple sensor data at different rates to FIFO.
	// Setting value can be calculated as follows:
	// Value = (DMP running rate / ODR ) - 1
	// E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
	success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
	// success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
	// success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
	// success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
	// success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
	// success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

	// Enable the FIFO
	success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

	// Enable the DMP
	success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

	// Reset DMP
	success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

	// Reset FIFO
	success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

	// Check success
	if (success)
	{
#ifndef QUAT_ANIMATION
		SERIAL_PORT.println(F("DMP enabled!"));
#endif
	}
	else
	{
		SERIAL_PORT.println(F("Enable DMP failed!"));
		SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
		while (1)
			; // Do nothing more
	}
    xTaskCreatePinnedToCore(IMU_Task, "IMU_Task", 4096, NULL, 1, &IMU_task_handle, 1);
}

void IMU_Task(void *pvParameters)
{
	for (;;)
	{
		// Read any DMP data waiting in the FIFO
		// Note:
		//    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
		//    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
		//    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
		//    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
		//    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
		icm_20948_DMP_data_t data;
		myICM.readDMPdataFromFIFO(&data);

		if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
		{
			// SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
			// if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
			// if ( data.header < 0x100) SERIAL_PORT.print( "0" );
			// if ( data.header < 0x10) SERIAL_PORT.print( "0" );
			// SERIAL_PORT.println( data.header, HEX );

			if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
			{
				// Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
				// In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
				// The quaternion data is scaled by 2^30.

				// SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

				// Scale to +/- 1
				double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
				double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
				double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

				/*
				SERIAL_PORT.print(F("Q1:"));
				SERIAL_PORT.print(q1, 3);
				SERIAL_PORT.print(F(" Q2:"));
				SERIAL_PORT.print(q2, 3);
				SERIAL_PORT.print(F(" Q3:"));
				SERIAL_PORT.println(q3, 3);
		  */

				// Convert the quaternions to Euler angles (roll, pitch, yaw)
				// https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

				double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

				double q2sqr = q2 * q2;

				// roll (x-axis rotation)
				double t0 = +2.0 * (q0 * q1 + q2 * q3);
				double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
				roll = atan2(t0, t1) * 180.0 / PI;

				// pitch (y-axis rotation)
				double t2 = +2.0 * (q0 * q2 - q3 * q1);
				t2 = t2 > 1.0 ? 1.0 : t2;
				t2 = t2 < -1.0 ? -1.0 : t2;
				pitch = asin(t2) * 180.0 / PI;

				// yaw (z-axis rotation)
				double t3 = +2.0 * (q0 * q3 + q1 * q2);
				double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
				yaw = atan2(t3, t4) * 180.0 / PI;

                SERIAL_PORT.printf("Roll: %f Pitch: %f Yaw: %f\n", roll, pitch, yaw);

				// if (abs(pitch) > 20) {
				// 	// The boat is capsized
				// 	ESP_LOGI("IMU_Task", "Boat is capsized. Roll: %f Pitch: %f Yaw: %f", roll, pitch, yaw);
				// }
				// ESP_LOGI("IMU_Task", "Roll: %f Pitch: %f Yaw: %f", roll, pitch, yaw);

#ifndef QUAT_ANIMATION
				// SERIAL_PORT.print(F("Roll:"));
				// SERIAL_PORT.print(roll, 1);
				// SERIAL_PORT.print(F(" Pitch:"));
				// SERIAL_PORT.print(pitch, 1);
				// SERIAL_PORT.print(F(" Yaw:"));
				// SERIAL_PORT.println(yaw, 1);
#else
				// Output the Quaternion data in the format expected by ZaneL's Node.js Quaternion animation tool
				SERIAL_PORT.print(F("{\"quat_w\":"));
				SERIAL_PORT.print(q0, 3);
				SERIAL_PORT.print(F(", \"quat_x\":"));
				SERIAL_PORT.print(q1, 3);
				SERIAL_PORT.print(F(", \"quat_y\":"));
				SERIAL_PORT.print(q2, 3);
				SERIAL_PORT.print(F(", \"quat_z\":"));
				SERIAL_PORT.print(q3, 3);
				SERIAL_PORT.println(F("}"));
#endif
			}
		}
		//   if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
		//   {
		//     delay(10);
		//   }
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

#endif // ICM20948HANDLER_H