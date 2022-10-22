/*!
 * @file        artus_9DOF_Razor_M0_ROS.ino
 * @brief       firmware to upload to a SAMD21 (similar or better) MicroController for ahrs usage
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
 */

#include <artus_9DOF_Razor_M0.h>
#include <imu_dataTypes.h>

#define SerialPort SerialUSB

#define __ROSPY__MSG__

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
    artus_imu.updatedIMUvalues(imuValues);
#ifndef __ROSPY__MSG__
    ///~~~~~~~~ Header ~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
    SerialPort.println("\t\t\t§-DEBUG-§");
///~~~~~~~~ TimeStamp ~~~~~~~~~~~~~~~~~~~~~~~~~//
    SerialPort.println("TimeStamp:\t "+ String(imuValues.arduTimeStamp) + " [ms]" );
///~~~~~~~~ RAW Accelerometer ~~~~~~~~~~~~~~~~~//
    SerialPort.println("Accel_RAW:\t "+  String(imuValues.accel_raw.x,5) + " , " + String(imuValues.accel_raw.y,5) + " , " + String(imuValues.accel_raw.z,5) + " [g]");
///~~~~~~~~ RAW Gyroscope ~~~~~~~~~~~~~~~~~~~~~//
    SerialPort.println("Gyro_RAW:\t "+  String(imuValues.gyro_raw.x,5) + " , " + String(imuValues.gyro_raw.y,5) + " , " + String(imuValues.gyro_raw.z,5) + " [°/s]");
///~~~~~~~~ RAW Magnetometer ~~~~~~~~~~~~~~~~~~//
    SerialPort.println("Magno_RAW:\t "+  String(imuValues.magno_raw.x,5) + " , " + String(imuValues.magno_raw.y,5) + " , " + String(imuValues.magno_raw.z,5) + " [µT]");
///~~~~~~~~ Temperature ~~~~~~~~~~~~~~~~~~~~~~~//
    SerialPort.println("Temperature:\t "+  String(imuValues.temp) + " [°C]");
///~~~~~~~~ Corrected Accelerometer ~~~~~~~~~~~//
    SerialPort.println("Accel_COR:\t "+  String(imuValues.accel_cor.x,5) + " , " + String(imuValues.accel_cor.y,5) + " , " + String(imuValues.accel_cor.z,5) + " [g]");
///~~~~~~~~ Corrected Gyroscope ~~~~~~~~~~~~~~~//
    SerialPort.println("Gyro_COR:\t "+  String(imuValues.gyro_cor.x,5) + " , " + String(imuValues.gyro_cor.y,5) + " , " + String(imuValues.gyro_cor.z,5) + " [°/s]");
///~~~~~~~~ Corrected Magnetometer ~~~~~~~~~~~~//
    SerialPort.println("Magno_RAW:\t "+  String(imuValues.magno_cor.x,5) + " , " + String(imuValues.magno_cor.y,5) + " , " + String(imuValues.magno_cor.z,5) + " [µT]");
///~~~~~~~~ Euler Angles ~~~~~~~~~~~~~~~~~~~~~~//
    SerialPort.println("RPY:\t\t " + String(imuValues.eulerAngle.roll,5) + " , " + String(imuValues.eulerAngle.pitch,5) + " , " + String(imuValues.eulerAngle.yaw,5) );
///~~~~~~~~ Quaternions ~~~~~~~~~~~~~~~~~~~~~~~//
    SerialPort.println("qx|qy|qz|qw:\t " + String(imuValues.quaternion.qx,5) + " , " + String(imuValues.quaternion.qy,5) + " , " + String(imuValues.quaternion.qz,5) + " , " + String(imuValues.quaternion.qw,5));
///~~~~~~~~ Linear Acceleration ~~~~~~~~~~~~~~~//
    SerialPort.println("LinAccel:\t "+  String(imuValues.linAcc.x,5) + " , " + String(imuValues.linAcc.y,5) + " , " + String(imuValues.linAcc.z,5) + " [m/s²]");
///~~~~~~~~ Angular Velocity ~~~~~~~~~~~~~~~~~~//
    SerialPort.println("AngVel:\t "+  String(imuValues.angVel.x,5) + " , " + String(imuValues.angVel.y,5) + " , " + String(imuValues.angVel.z,5) + " [rad/s]");
///~~~~~~~~ relativ Velocity ~~~~~~~~~~~~~~~~~~//
    SerialPort.println("relVel:\t\t "+  String(imuValues.relVelocity.x,5) + " , " + String(imuValues.relVelocity.y,5) + " , " + String(imuValues.relVelocity.z,5) + " [m/s]");
///~~~~~~~~ relativ Distance ~~~~~~~~~~~~~~~~~~//
    SerialPort.println("relDist:\t "+  String(imuValues.relDistance.x,5) + " , " + String(imuValues.relDistance.y,5) + " , " + String(imuValues.relDistance.z,5) + " [m]");
///~~~~~~~~ Header ~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
#endif
#ifdef __ROSPY__MSG__
///~~~~~~~~ Header ~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
    //SerialPort.print("#E");
    SerialPort.print("#Q");
///~~~~~~~~ Euler Angles ~~~~~~~~~~~~~~~~~~~~~~//
    //SerialPort.print(String(imuValues.eulerAngle.yaw,5) + "," +  String(imuValues.eulerAngle.roll,5) + "," + String(imuValues.eulerAngle.pitch,5)   );
///~~~~~~~~ Quaternions ~~~~~~~~~~~~~~~~~~~~~~~//
    SerialPort.print(String(imuValues.quaternion.qx , 5) + "," + String(imuValues.quaternion.qy , 5) + "," + String(imuValues.quaternion.qz , 5) + "," + String(imuValues.quaternion.qw , 5));
///~~~~~~~~ Linear Acceleration ~~~~~~~~~~~~~~~//
    SerialPort.print("," +  String(imuValues.linAcc.x , 5) + "," + String(imuValues.linAcc.y , 5) + "," + String(imuValues.linAcc.z , 5));
///~~~~~~~~ Angular Velocity ~~~~~~~~~~~~~~~~~~//
    SerialPort.print("," +  String(imuValues.angVel.x , 5) + "," + String(imuValues.angVel.y , 5) + "," + String(imuValues.angVel.z , 5));
///~~~~~~~~ relativ Velocity ~~~~~~~~~~~~~~~~~~//
    SerialPort.print("," + String(imuValues.relVelocity.x , 5) + "," + String(imuValues.relVelocity.y,5) + "," + String(imuValues.relVelocity.z,5) );
///~~~~~~~~ relativ Distance ~~~~~~~~~~~~~~~~~//
    SerialPort.println("," + String(imuValues.relDistance.x , 5) + "," + String(imuValues.relDistance.y , 5) + "," + String(imuValues.relDistance.z , 5) );
///~~~~~~~~ Header ~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
#endif
}