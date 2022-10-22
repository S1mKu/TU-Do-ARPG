/*!
 * @file        artus_LSM6DSOX_LIS3MDL_gyro_calibration.ino
 * @brief       firmware to upload to a SAMD21 (similar or better) MicroController for calibrating the gyro
 * @mainpage    ARTUS IMU TOOLKIT
 * @author      Samanta Scharmacher <samanta.scharmacher@tu-dortmund.de>
 *
 * @note        if floatingPoint arithmetic is too slow consider using fixePoint arithmetic!
 *              Development environment specifics:
 *               - Arduino IDE 1.8.19
 *              Hardware:
 *               - Seeeduino XIAO | Arduino 33 IoT (ATSAMD21)
 *               - Adafruit: LSM6DSOX - LIS3MDL
 *              Supported Platforms:
 *               - ATSAMD21[G18A] (Seeeduino XIAO, Arduino 33 IoT, Arduino Zero, SparkFun SAMD21 Breakouts)
 */

#include <artus_LSM6DSOX_LIS3MDL.h>
#include <imu_dataTypes.h>

#define SerialPort SerialUSB

#define __DEBUG__MSG__

#define SAMPLESIZE 5555

artus_LSM6DSOX_LIS3MDL artus_imu;
imuDataContainer imuValues;

int  counter = 0;
vec3imu cumulatedGyro={0,0,0};
int incomingByte;


void setup() {
    /// use one of the faster Baud-Rates for communication between IMU to NVIDIA Jetson
    SerialPort.begin(115200);
    while (!SerialPort);
    SerialPort.println("=================== INIT =========================");
    Wire.begin();
    /// Call artus_imu.imu_init() to verify communication as well as initializing the IMU (MPU9250) to it's default values
    /// On Success this function will return (0), that indicates, that the IMU of given type was set up successfully.
    while (artus_imu.imu_init() != 0){
        SerialPort.println("Unable to communicate with LSM6DSOX or LIS3MDL");
        SerialPort.println("Check connections, and retry in 500 ms again.");
        SerialPort.println();
        delay(500);          // Delay in MS
    }

    artus_imu.imu_start();

    Wire.setClock(400000); // 400KHz
    SerialPort.println("=================== START ========================");
}

void loop() {
    /// Update IMU Values and read that information into Data-Variable
    if (artus_imu.updatedIMUvalues(imuValues) ){
        cumulatedGyro += imuValues.gyro_raw;
        ///===PrintSerial===
#ifdef __DEBUG__MSG__
        SerialPort.println("Gyro: "+  String(imuValues.gyro_raw.x,5) + " , " + String(imuValues.gyro_raw.y,5) + " , " + String(imuValues.gyro_raw.z,5) + " [°/s]");
#endif
        counter++;

        if(counter >= SAMPLESIZE){
            cumulatedGyro /= SAMPLESIZE;

#ifdef __DEBUG__MSG__
            SerialPort.println("===========================================");
#endif
            SerialPort.println("Gyro_Offset: "+  String(cummulatedGyro.x,5) + " , " + String(cummulatedGyro.y,5) + " , " + String(cummulatedGyro.z,5) + " [°/s]");

#ifdef __DEBUG__MSG__
            SerialPort.println("===========================================");
#endif
            SerialPort.println("=================== STOPP ========================");
            //delay(1000);
            ///Wait for any interaction via Serial to restart
            while (Serial.available() == 0);
            incomingByte = Serial.read();

            SerialPort.println("=================== START ========================");
            counter = 0;
        }
    }
}
