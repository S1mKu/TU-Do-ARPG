/*!
 * @file        artus_9DOF_Razor_M0.h
 * @brief       Class to interact with 9DOF_Razor_M0 imu-Sensors.
 * @mainpage    ARTUS IMU TOOLKIT
 * @author      Samanta Scharmacher <samanta.scharmacher@tu-dortmund.de>
 *
 * @note        if floatingPoint arithmetic is too slow consider using fixePoint arithmetic!
 */

#ifndef ARTUS_IMU_TOOLKIT_ARTUS_9DOF_RAZOR_M0_H
#define ARTUS_IMU_TOOLKIT_ARTUS_9DOF_RAZOR_M0_H

#include <SparkFunMPU9250-DMP.h>
#include "imu_sensor.h"

class artus_9DOF_Razor_M0 : IMU_sensor {
private:
    MPU9250_DMP imu;
public:
    artus_9DOF_Razor_M0();

    int imu_init();
    int imu_start();

    int updatedIMUvalues();
    int updatedIMUvalues(imuDataContainer & imuValues);

protected:
    int getRawAcceleration(vec3imu& accelRawValues);
    int getRawGyro(vec3imu& gyroRawValues);
    int getRawMagnetometer(vec3imu& magRawValues);
    int getTemperature(float& temperatureValue);
    int getCorAcceleration(vec3imu& accelCorValues, const vec3imu& accelRawValues);
    int getCorGyro(vec3imu& gyroCorValues , const vec3imu& gyroRawValues);
    int getCorMagnetometer(vec3imu& magCorValues, const vec3imu& magRawValues);
    int getEulerAngle(float& roll,float& pitch, float& yaw);
    int getEulerAngle(euler3imu& eulerAngle);
    int getQuaternion(float& qw, float&qx, float& qy, float& qz );
    int getQuaternion(quaternion4imu&  quaternion);
    int getLinearAcceleration(vec3imu& linearAccel , vec3imu& gravity , const vec3imu& accelCorValues);
    int getAngularVelocity(vec3imu& angularVel , const vec3imu& gyroCorValues);
    int getRelativSpeed(vec3imu& relSpeed, vec3imu& acceleration_cur, vec3imu& acceleration_prev, const unsigned long& curTimeStamp , const unsigned long& prevTimeStamp );
    int getRelativeDistance(vec3imu& relDistance, vec3imu& relVelocity_cur, vec3imu& relVelocity_prev, const unsigned long& curTimeStamp , const unsigned long& prevTimeStamp);
    int getTimeStamp(unsigned long& arduTimeStamp);
    int getGravity(vec3imu& gravity);
};
///RAZOR M0 DEFINITIONS:
///~~~~~~~~~~~~ LED ~~~~~~~~~~~~~~~~~~~~~~~~~///
#define HW_LED_PIN 13           // LED attached to pin 13
///~~~~~~~~~~~~ SD Card ~~~~~~~~~~~~~~~~~~~~~///
#define ENABLE_SD_LOGGING true  // Default SD logging (can be changed via serial menu)
#define SD_CHIP_SELECT_PIN 38   // Pin for SPI Chip_Select
///~~~~~~~~~~~~ IMU ~~~~~~~~~~~~~~~~~~~~~~~~~///
#define MPU9250_INT_PIN 4
#define MPU9250_INT_ACTIVE LOW

///MEMS SetupValues:
///~~~~~~~~~~~~ Accel ~~~~~~~~~~~~~~~~~~~~~~~///
#define ACCEL_2_G       2
#define ACCEL_4_G       4
#define ACCEL_8_G       8
#define ACCEL_16_G      16
///~~~~~~~~~~~~ Gyro ~~~~~~~~~~~~~~~~~~~~~~~~///
#define GYRO_250_DPS    250
#define GYRO_500_DPS    500
#define GYRO_1000_DPS   1000
#define GYRO_2000_DPS   2000
///~~~~~~~~~ LowPassFilter ~~~~~~~~~~~~~~~~~~///
#define LPF_188_HZ      188     // >= 188
#define LPF_98_HZ       98      // >= 98
#define LPF_42_HZ       42      // >= 42
#define LPF_20_HZ       20      // >= 20
#define LPF_10_HZ       10      // >= 10
#define LPF_5_HZ        5       // else 5
///~~~~~~~ Gyro/Accel-SampleRate ~~~~~~~~~~~~///
#define SMPL_1000_HZ    1000
#define SMPL_400_HZ     400
#define SMPL_200_HZ     200
#define SMPL_100_HZ     100
#define SMPL_40_HZ      40
#define SMPL_10_HZ      10
#define SMPL_4_HZ       4
///~~~~~~~ Magnetometer-SampleRate ~~~~~~~~~~///
#define mSMPL_1000_HZ    1000
#define mSMPL_400_HZ     400
#define mSMPL_200_HZ     200
#define mSMPL_100_HZ     100
#define mSMPL_40_HZ      40
#define mSMPL_10_HZ      10
#define mSMPL_4_HZ       4
///~~~~~~~ Temperature-SetUpValues ~~~~~~~~~///
#define ROOM_TEMPERATURE_OFFSET  0.0F
#define TEMPERATURE_SENSITIVITY  333.87F

#endif //ARTUS_IMU_TOOLKIT_ARTUS_9DOF_RAZOR_M0_H
