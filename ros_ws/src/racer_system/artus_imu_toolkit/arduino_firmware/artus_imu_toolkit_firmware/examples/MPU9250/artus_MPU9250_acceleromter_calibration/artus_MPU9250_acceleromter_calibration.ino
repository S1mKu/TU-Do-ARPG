/*!
 * @file        artus_MPU9250_accelerometer_calibration.ino
 * @brief       firmware to upload to a SAMD21 (similar or better) MicroController for calibrating the accelerometer
 * @mainpage    ARTUS IMU TOOLKIT
 * @author      Samanta Scharmacher <samanta.scharmacher@tu-dortmund.de>
 *
 * @note        if floatingPoint arithmetic is too slow consider using fixePoint arithmetic!
 *              Development environment specifics:
 *               - Arduino IDE 1.8.19
 *              Hardware:
 *               - Arduino 33 IoT (ATSAMD21)
 *               - MPU9250
 *              Supported Platforms:
 *               - ATSAMD21[G18A] (Arduino 33 IoT, Arduino Zero, SparkFun SAMD21 Breakouts)
 */

#include <artus_mpu9250.h>
#include <imu_dataTypes.h>

/************************************************************************************************************
TODO:
 - implementing direct calculating procedure for acceleration Offsets ans Gain Values.
 - implementing dynamically calibration Procedures to reCalibrate IMU-Sensors while running.
 - implement storing structure (adding attributes to already existing Container) to store callibration values as one record
 - implementing storing for covariances of an imu-sensor (depending on specific sensormoduls)s to dynamically reCalibrate IMU Sensors while running.
*************************************************************************************************************/

#define SerialPort SerialUSB

#define __DEBUG__MSG__

#define SAMPLESIZE 5555

artus_mpu9250 artus_imu;
imuDataContainer imuValues;

vec3imu rawMin={0,0,0}, rawMax={0,0,0}, rawMid={0,0,0}, accel_scale={1,1,1};
//every measuring Position
vec3imu allRawMid[6] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};

int axiscounter=0;
int samplecounter=0;
int incomingByte;

void setup() {
    /// use one of the faster Baud-Rates for communication between IMU to NVIDIA Jetson
    SerialPort.begin(115200);
    while (!SerialPort);
    Wire.begin();
    SerialPort.println("=================== INIT =========================");
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
    SerialPort.println("=================== START ========================");
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
    /// Update IMU Values and read that information into Data-Variable
    if (artus_imu.updatedIMUvalues(imuValues) ){
        /// Sample input Values for 5555 Samples
        if(samplecounter < SAMPLESIZE){
            switch(axiscounter){
                /// (+) X-axis
                case 0:
                    //min-Scalingpoint referef to gravity
                    if(imuValues.accel_raw.x < rawMin.x){
                        rawMin.x = imuValues.accel_raw.x;
                    }
                    //max-Scalingpoint referef to gravity
                    if(imuValues.accel_raw.x > rawMax.x){
                        rawMax.x = imuValues.accel_raw.x;
                    }
                    rawMid.y +=  imuValues.accel_raw.y; //Zero-Point
                    rawMid.z +=  imuValues.accel_raw.z; //Zero-Point

                    allRawMid[0].x += imuValues.accel_raw.x;
                    allRawMid[0].y +=  imuValues.accel_raw.y; //Zero-Point
                    allRawMid[0].z +=  imuValues.accel_raw.z; //Zero-Point
                    break;
                    /// (-) X-axis
                case 1:
                    //min-Scalingpoint referef to gravity
                    if(imuValues.accel_raw.x < rawMin.x){
                        rawMin.x = imuValues.accel_raw.x;
                    }
                    //max-Scalingpoint referef to gravity
                    if(imuValues.accel_raw.x > rawMax.x){
                        rawMax.x = imuValues.accel_raw.x;
                    }
                    rawMid.y +=  imuValues.accel_raw.y; //Zero-Point
                    rawMid.z +=  imuValues.accel_raw.z; //Zero-Point
                    allRawMid[1].x += imuValues.accel_raw.x;
                    allRawMid[1].y +=  imuValues.accel_raw.y; //Zero-Point
                    allRawMid[1].z +=  imuValues.accel_raw.z; //Zero-Point
                    break;

                    /// (+) Y-axis
                case 2:
                    //min-Scalingpoint referef to gravity
                    if(imuValues.accel_raw.y < rawMin.y){
                        rawMin.y = imuValues.accel_raw.y;
                    }
                    //max-Scalingpoint referef to gravity
                    if(imuValues.accel_raw.y > rawMax.y){
                        rawMax.y = imuValues.accel_raw.y;
                    }
                    rawMid.x +=  imuValues.accel_raw.x; //Zero-Point
                    rawMid.z +=  imuValues.accel_raw.z; //Zero-Point
                    allRawMid[2].x += imuValues.accel_raw.x;
                    allRawMid[2].y +=  imuValues.accel_raw.y; //Zero-Point
                    allRawMid[2].z +=  imuValues.accel_raw.z; //Zero-Point
                    break;

                    /// (-) Y-axis
                case 3:
                    //min-Scalingpoint referef to gravity
                    if(imuValues.accel_raw.y < rawMin.y){
                        rawMin.y = imuValues.accel_raw.y;
                    }
                    //max-Scalingpoint referef to gravity
                    if(imuValues.accel_raw.y > rawMax.y){
                        rawMax.y = imuValues.accel_raw.y;
                    }
                    rawMid.x +=  imuValues.accel_raw.x; //Zero-Point
                    rawMid.z +=  imuValues.accel_raw.z; //Zero-Point
                    allRawMid[3].x += imuValues.accel_raw.x;
                    allRawMid[3].y +=  imuValues.accel_raw.y; //Zero-Point
                    allRawMid[3].z +=  imuValues.accel_raw.z; //Zero-Point
                    break;

                    /// (+) Z-axis
                case 4:
                    //min-Scalingpoint referef to gravity
                    if(imuValues.accel_raw.z < rawMin.z){
                        rawMin.z = imuValues.accel_raw.z;
                    }
                    //max-Scalingpoint referef to gravity
                    if(imuValues.accel_raw.z > rawMax.z){
                        rawMax.z = imuValues.accel_raw.z;
                    }
                    rawMid.x +=  imuValues.accel_raw.x; //Zero-Point
                    rawMid.y +=  imuValues.accel_raw.y; //Zero-Point

                    allRawMid[4].x += imuValues.accel_raw.x;
                    allRawMid[4].y +=  imuValues.accel_raw.y; //Zero-Point
                    allRawMid[4].z +=  imuValues.accel_raw.z; //Zero-Point
                    break;

                    /// (-) z-axis
                case 5:
                    //min-Scalingpoint referef to gravity
                    if(imuValues.accel_raw.z < rawMin.z){
                        rawMin.z = imuValues.accel_raw.z;
                    }
                    //max-Scalingpoint referef to gravity
                    if(imuValues.accel_raw.z > rawMax.z){
                        rawMax.z = imuValues.accel_raw.z;
                    }
                    rawMid.x +=  imuValues.accel_raw.x; //Zero-Point
                    rawMid.y +=  imuValues.accel_raw.y; //Zero-Point
                    allRawMid[5].x += imuValues.accel_raw.x;
                    allRawMid[5].y +=  imuValues.accel_raw.y; //Zero-Point
                    allRawMid[5].z +=  imuValues.accel_raw.z; //Zero-Point
                    break;
            }
            samplecounter++;
        } else {
            switch(axiscounter){
                /// (+) X-axis
                case 0:
#ifdef __DEBUG__MSG__
                    SerialPort.println("=============== (+) X-AXIS =======================");
                    SerialPort.println("(+)X_min: "+  String(rawMin.x,5) +" [g]");
                    SerialPort.println("(+)X_max: "+  String(rawMax.x,5) +" [g]");
                    SerialPort.println("raw_mid_xyz: "+  String(rawMid.x,5) + " , " + String(rawMid.y/SAMPLESIZE,5) + " , " + String(rawMid.z/SAMPLESIZE,5)+" [g]");
#endif
                    axiscounter=1;
#ifdef __DEBUG__MSG__
                    SerialPort.println("=============== (-) X-AXIS =======================");
#endif
                    ///Wait for any interaction via Serial to continue
                    while (Serial.available() == 0);
                    incomingByte = Serial.read();// Serial.In Frei räumen??
                    break;
                    /// (-) X-axis
                case 1:
#ifdef __DEBUG__MSG__
//                  SerialPort.println("=============== (-) X-AXIS =======================");
                    SerialPort.println("(-)X_min: "+  String(rawMin.x,5) +" [g]");
                    SerialPort.println("(-)X_max: "+  String(rawMax.x,5) +" [g]");
                    SerialPort.println("raw_mid_xyz: "+  String(rawMid.x,5) + " , " + String(rawMid.y/(SAMPLESIZE*2),5) + " , " + String(rawMid.z/(SAMPLESIZE*2),5)+" [g]");
#endif
                    axiscounter=2;
#ifdef __DEBUG__MSG__
                    SerialPort.println("=============== (+) Y-AXIS =======================");
#endif
                    ///Wait for any interaction via Serial to continue
                    while (Serial.available() == 0);
                    incomingByte = Serial.read();// Serial.In Frei räumen??
                    break;

                    /// (+) Y-axis
                case 2:
#ifdef __DEBUG__MSG__
//                  SerialPort.println("=============== (+) Y-AXIS =======================");
                    SerialPort.println("(+)Y_min: "+  String(rawMin.y,5) +" [g]");
                    SerialPort.println("(+)Y_max: "+  String(rawMax.y,5) +" [g]");
                    SerialPort.println("raw_mid_xyz: "+  String(rawMid.x/SAMPLESIZE,5) + " , " + String(rawMid.y/(SAMPLESIZE*2),5) + " , " + String(rawMid.z/(SAMPLESIZE*3),5)+" [g]");
#endif
                    axiscounter=3;
#ifdef __DEBUG__MSG__
                    SerialPort.println("=============== (-) Y-AXIS =======================");
#endif
                    ///Wait for any interaction via Serial to continue
                    while (Serial.available() == 0);
                    incomingByte = Serial.read();// Serial.In Frei räumen??
                    break;

                    /// (-) Y-axis
                case 3:
#ifdef __DEBUG__MSG__
//                  SerialPort.println("=============== (-) Y-AXIS =======================");
                    SerialPort.println("(-)Y_min: "+  String(rawMin.y,5) +" [g]");
                    SerialPort.println("(-)Y_max: "+  String(rawMax.y,5) +" [g]");
                    SerialPort.println("raw_mid_xyz: "+  String(rawMid.x/(SAMPLESIZE*2),5) + " , " + String(rawMid.y/(SAMPLESIZE*2),5) + " , " + String(rawMid.z/(SAMPLESIZE*4),5)+" [g]");
#endif
                    axiscounter=4;
#ifdef __DEBUG__MSG__
                    SerialPort.println("=============== (+) Z-AXIS =======================");
#endif
                    ///Wait for any interaction via Serial to continue
                    while (Serial.available() == 0);
                    incomingByte = Serial.read();// Serial.In Frei räumen??
                    break;

                    /// (+) Z-axis
                case 4:
#ifdef __DEBUG__MSG__
//                  SerialPort.println("=============== (+) Z-AXIS =======================");
                    SerialPort.println("(+)Z_min: "+  String(rawMin.y,5) +" [g]");
                    SerialPort.println("(+)Z_max: "+  String(rawMax.y,5) +" [g]");
                    SerialPort.println("raw_mid_xyz: "+  String(rawMid.x/(SAMPLESIZE*3),5) + " , " + String(rawMid.y/(SAMPLESIZE*3),5) + " , " + String(rawMid.z/(SAMPLESIZE*4),5)+" [g]");
#endif
                    axiscounter=5;
#ifdef __DEBUG__MSG__
                    SerialPort.println("=============== (-) Z-AXIS =======================");
#endif
                    ///Wait for any interaction via Serial to continue
                    while (Serial.available() == 0);
                    incomingByte = Serial.read();
                    break;

                    /// (-) z-axis
                case 5:
#ifdef __DEBUG__MSG__
//                  SerialPort.println("=============== (-) Z-AXIS =======================");
                    SerialPort.println("(-)Z_min: "+  String(rawMin.y,5) +" [g]");
                    SerialPort.println("(-)Z_max: "+  String(rawMax.y,5) +" [g]");
                    SerialPort.println("raw_mid_xyz: "+  String(rawMid.x/(SAMPLESIZE*4),5) + " , " + String(rawMid.y/(SAMPLESIZE*4),5) + " , " + String(rawMid.z/(SAMPLESIZE*4),5)+" [g]");
#endif
                    rawMid.x/=(SAMPLESIZE*4);
                    rawMid.y/=(SAMPLESIZE*4);
                    rawMid.z/=(SAMPLESIZE*4);

                    //remap
                    accel_scale.x=map_float(rawMid.x, rawMin.x, rawMax.x,-1.0f,1.0f);
                    accel_scale.y=map_float(rawMid.y, rawMin.y, rawMax.y,-1.0f,1.0f);
                    accel_scale.z=map_float(rawMid.z, rawMin.z, rawMax.z,-1.0f,1.0f);

                    SerialPort.println("=============== TOTAL =======================");
                    SerialPort.println("rawMin: "+  String(rawMin.x,5) + " , " + String(rawMin.y,5) + " , " + String(rawMin.z,5)+" [g]");
                    SerialPort.println("rawMax: "+  String(rawMax.x,5) + " , " + String(rawMax.y,5) + " , " + String(rawMax.z,5)+" [g]");
                    SerialPort.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
                    SerialPort.println("rawMid: "+  String(rawMid.x,5) + " , " + String(rawMid.y,5) + " , " + String(rawMid.z,5)+" [g]");
                    SerialPort.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
                    SerialPort.println("[  1  0  0 ]: "+  String(allRawMid[0].x/SAMPLESIZE,5) + " , " + String(allRawMid[0].y/SAMPLESIZE,5) + " , " + String(allRawMid[0].z/SAMPLESIZE,5)+" [g]");
                    SerialPort.println("[ -1  0  0 ]: "+  String(allRawMid[1].x/SAMPLESIZE,5) + " , " + String(allRawMid[1].y/SAMPLESIZE,5) + " , " + String(allRawMid[1].z/SAMPLESIZE,5)+" [g]");
                    SerialPort.println("[  0  1  0 ]: "+  String(allRawMid[2].x/SAMPLESIZE,5) + " , " + String(allRawMid[2].y/SAMPLESIZE,5) + " , " + String(allRawMid[2].z/SAMPLESIZE,5)+" [g]");
                    SerialPort.println("[  0 -1  0 ]: "+  String(allRawMid[3].x/SAMPLESIZE,5) + " , " + String(allRawMid[3].y/SAMPLESIZE,5) + " , " + String(allRawMid[3].z/SAMPLESIZE,5)+" [g]");
                    SerialPort.println("[  0  0  1 ]: "+  String(allRawMid[4].x/SAMPLESIZE,5) + " , " + String(allRawMid[4].y/SAMPLESIZE,5) + " , " + String(allRawMid[4].z/SAMPLESIZE,5)+" [g]");
                    SerialPort.println("[  0  0 -1 ]: "+  String(allRawMid[5].x/SAMPLESIZE,5) + " , " + String(allRawMid[5].y/SAMPLESIZE,5) + " , " + String(allRawMid[5].z/SAMPLESIZE,5)+" [g]");
                    SerialPort.println("=============== END =========================");
                    /// TODO: Automate Process by implementing those procedures
                    SerialPort.println("Note: Don't forget to solve the equation by the least squared method! ");
                    //delay(1000);
                    axiscounter=0;
                    ///Wait for any interaction via Serial to restart
                    while (Serial.available() == 0);
                    incomingByte = Serial.read();
                    break;
            }
            samplecounter=0;
        }
    }
}
    
