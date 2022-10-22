/*!
 * @file        artus_9DOF_Razor_M0_compass_calibration.ino
 * @brief       firmware to upload to a SAMD21 (similar or better) MicroController for calibrating the magnetometer/compass
 * @mainpage    ARTUS IMU TOOLKIT
 * @author      Samanta Scharmacher <samanta.scharmacher@tu-dortmund.de>
 *
 * @note        if floatingPoint arithmetic is too slow consider using fixePoint arithmetic!
 *              Development environment specifics:
 *               - Arduino IDE 1.8.19
 *              Hardware:
 *               - SparkFun 9DoF Razor IMU M0 (SEN-14001) [ https://www.sparkfun.com/products/retired/14001 ]
 *               - MPU9250
 *              Supported Platforms:
 *               - ATSAMD21[G18A] (Seeeduino XIAO, Arduino 33 IoT, Arduino Zero, SparkFun SAMD21 Breakouts)
 *
 *              => USING MotionCal as Calibration Software ( https://www.pjrc.com/store/prop_shield.html )
 */

#include <artus_9DOF_Razor_M0.h>
#include <imu_dataTypes.h>

#define SerialPort SerialUSB

#define __DEBUG__MSG__

artus_9DOF_Razor_M0 artus_imu;
imuDataContainer imuValues;

void setup() {
/// use one of the faster Baud-Rates for communication between IMU to NVIDIA Jetson
    SerialPort.begin(115200);
    while (!SerialPort);
    Wire.begin();
    /// Call artus_imu.imu_init() to verify communication as well as initializing the IMU (MPU9250) to it's default values
    /// On Success this function will return (0), that indicates, that the IMU of given type was set up successfully.
    while (artus_imu.imu_init() != 0){
        SerialPort.println("Unable to communicate with MPU-9250");
        SerialPort.println("Check connections, and retry in 500 ms again.");
        SerialPort.println();
        delay(500);          // Delay in MS
    }
    artus_imu.imu_start();

    Wire.setClock(400000); // 400KHz
}

void loop() {
    /// Update IMU Values and read that information into Data-Variable
    if (artus_imu.updatedIMUvalues(imuValues) ) {
        /// Using special Serial-Output-Format for callibrating the compass Module
        /// raw data
        Serial.print("Raw:");
        Serial.print(int(imuValues.accel_raw.x*8192)); Serial.print(",");
        Serial.print(int(imuValues.accel_raw.y*8192)); Serial.print(",");
        Serial.print(int(imuValues.accel_raw.z*8192)); Serial.print(",");
        Serial.print(int(imuValues.gyro_raw.x*16)); Serial.print(",");
        Serial.print(int(imuValues.gyro_raw.y*16)); Serial.print(",");
        Serial.print(int(imuValues.gyro_raw.z*16)); Serial.print(",");
        Serial.print(int(imuValues.magno_raw.x*10)); Serial.print(",");
        Serial.print(int(imuValues.magno_raw.y*10)); Serial.print(",");
        Serial.print(int(imuValues.magno_raw.z*10)); Serial.println("");

        /// unified data
        Serial.print("Uni:");
        Serial.print(imuValues.accel_raw.x); Serial.print(",");
        Serial.print(imuValues.accel_raw.y); Serial.print(",");
        Serial.print(imuValues.accel_raw.z); Serial.print(",");
        Serial.print(imuValues.gyro_raw.x, 4); Serial.print(",");
        Serial.print(imuValues.gyro_raw.y, 4); Serial.print(",");
        Serial.print(imuValues.gyro_raw.z, 4); Serial.print(",");
        Serial.print(imuValues.magno_raw.x); Serial.print(",");
        Serial.print(imuValues.magno_raw.y); Serial.print(",");
        Serial.print(imuValues.magno_raw.z); Serial.println("");
    }
}