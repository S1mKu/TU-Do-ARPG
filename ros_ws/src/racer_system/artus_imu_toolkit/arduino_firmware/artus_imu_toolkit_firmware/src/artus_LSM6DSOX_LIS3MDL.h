/*!
 * @file        artus_LSM6DSOX_LIS3MDL.h
 * @brief       Class to interact with LSM6DSOX & LIS3MDL imu-Sensors.
 * @mainpage    ARTUS IMU TOOLKIT
 * @author      Samanta Scharmacher <samanta.scharmacher@tu-dortmund.de>
 *
 * @note        if floatingPoint arithmetic is too slow consider using fixePoint arithmetic!
 */

#ifndef ARTUS_IMU_TOOLKIT_ARTUS_LSM6DSOX_LIS3MDL_H
#define ARTUS_IMU_TOOLKIT_ARTUS_LSM6DSOX_LIS3MDL_H

#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>
#include "imu_sensor.h"

class artus_LSM6DSOX_LIS3MDL : IMU_sensor {
private:
    Adafruit_LIS3MDL lis3mdl;
    Adafruit_LSM6DSOX lsm6ds;
    Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer, *temperature;

public:
    artus_LSM6DSOX_LIS3MDL();

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

#endif //ARTUS_IMU_TOOLKIT_ARTUS_LSM6DSOX_LIS3MDL_H
