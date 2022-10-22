/*!
 * @file        imu_sensor.h
 * @brief       Abstract Class to interact with imu-Sensors.
 * @mainpage    ARTUS IMU TOOLKIT
 * @author      Samanta Scharmacher <samanta.scharmacher@tu-dortmund.de>
 *
 * @note        if floatingPoint arithmetic is too slow consider using fixePoint arithmetic!
 */

#ifndef ARTUS_IMU_TOOLKIT_IMU_SENSOR_H
#define ARTUS_IMU_TOOLKIT_IMU_SENSOR_H

#include <Adafruit_AHRS_FusionInterface.h>
#include <Adafruit_AHRS_NXPFusion.h>
#include <Adafruit_AHRS_Madgwick.h>
#include <Adafruit_AHRS_Mahony.h>

#include "imu_dataTypes.h"

///Other Constants
/// Filter-Confoguration Definition
#define FILTER_UPDATE_RATE_HZ 100.0f

/// helper ~> restructure later in extra file!
#define SENSORS_DPS_TO_RADS (0.017453293F) /**< Degrees/s to rad/s multiplier  */
#define SENSORS_RADS_TO_DPS (57.29577793F) /**< Rad/s to degrees/s  multiplier */
#define SENSORS_g_TO_MS2 (9.80665F) /**< Rad/s to degrees/s  multiplier */
#define SENSORS_MS2_TO_g (0.101971621F) /**< Rad/s to degrees/s  multiplier */

class IMU_sensor {
public:
    IMU_sensor();

    virtual ~IMU_sensor();

    virtual int imu_init()=0;

    virtual int imu_start()=0;

    virtual int updatedIMUvalues()=0;
    virtual int updatedIMUvalues(imuDataContainer & imuValues)=0;

    int setAccelOffset(vec3offset new_acc_offset);
    int setAccelGain(mat3x3gain new_acc_gain);
    int setGyroOffset(vec3offset new_gyro_offset);
    int setMagOffset(vec3offset new_mag_offset);
    int setMagGain(mat3x3gain new_mag_gain);

/// TODO:    vec3offset getAccelOffset();
/// TODO:    mat3x3gain getAccelGain();
/// TODO:    vec3offset getGyroOffset();
/// TODO:    vec3offset getMagOffset();
/// TODO:    mat3x3gain getMagGain();

protected:
    Adafruit_NXPSensorFusion filter_;   // slowest
    // Adafruit_Madgwick filter_;       // faster than NXP
    // Adafruit_Mahony filter_;         // fastest/smalleset

    vec3offset accOffset_ = {0.0,0.0 ,0.0};
    mat3x3gain accGain_ = {{1.0 , 1.0 , 1.0},
                           {1.0 , 1.0 , 1.0},
                           {1.0,1.0,1.0}};
    vec3offset gyroOffset_ = {0.0, 0.0, 0.0};
    vec3offset magOffset_bH_ = {0.0, 0.0, 0.0};
    mat3x3gain magGain_c_ = {{1.0 , 1.0 , 1.0},
                             {1.0 , 1.0 , 1.0},
                             {1.0, 1.0, 1.0 }};

    virtual int getRawAcceleration(vec3imu& accelRawValues)=0;
    virtual int getRawGyro(vec3imu& gyroRawValues)=0;
    virtual int getRawMagnetometer(vec3imu& magRawValues)=0;
    virtual int getTemperature(float& temperatureValue)=0;
    virtual int getCorAcceleration(vec3imu& accelCorValues,const vec3imu& accelRawValues)=0;
    virtual int getCorGyro(vec3imu& gyroCorValues, const vec3imu& gyroRawValues)=0;
    virtual int getCorMagnetometer(vec3imu& magCorValues, const vec3imu& magRawValues)=0;
    virtual int getEulerAngle(float& roll,float& pitch, float& yaw)=0;
    virtual int getEulerAngle(euler3imu& eulerAngle)=0;
    virtual int getQuaternion(float& qw, float&qx, float& qy, float& qz )=0;
    virtual int getQuaternion(quaternion4imu&  quaternion)=0;
    virtual int getLinearAcceleration(vec3imu& linearAccel , vec3imu& gravity , const vec3imu& accelCorValues)=0;
    virtual int getAngularVelocity(vec3imu& angularVel , const vec3imu& gyroCorValues)=0;
    virtual int getRelativSpeed(vec3imu& relSpeed, vec3imu& acceleration_cur, vec3imu& acceleration_prev, const unsigned long& curTimeStamp , const unsigned long& prevTimeStamp )=0;
    virtual int getRelativeDistance(vec3imu& relDistance, vec3imu& relVelocity_cur , vec3imu& relVelocity_prev, const unsigned long& curTimeStamp , const unsigned long& prevTimeStamp)=0;
    virtual int getTimeStamp(unsigned long& arduTimeStamp)=0;
    virtual int getGravity(vec3imu& gravity)=0;

    int getRawAcceleration(imuDataContainer& imuValues);
    int getRawGyro(imuDataContainer& imuValues);
    int getRawMagnetometer(imuDataContainer& imuValues);
    int getTemperature(imuDataContainer& imuValues);
    int getCorAcceleration(imuDataContainer& imuValues);
    int getCorGyro(imuDataContainer& imuValues);
    int getCorMagnetometer(imuDataContainer& imuValues);
    int getEulerAngle(imuDataContainer& imuValues);
    int getQuaternion(imuDataContainer& imuValues);
    int getLinearAcceleration(imuDataContainer& imuValues);
    int getAngularVelocity(imuDataContainer& imuValues);
    int getRelativSpeed(imuDataContainer& imuValues );
    int getRelativeDistance(imuDataContainer& imuValues);
    int getTimeStamp(imuDataContainer& imuValues);
    int getGravity(imuDataContainer& imuValues);

    int setPrevTimeStamp(imuDataContainer& imuValues);
    int setPrevAcceleration(imuDataContainer& imuValues);
    int setPrevVelocity(imuDataContainer& imuValues);

    int setPrevTimeStamp(const unsigned long& arduTimeStamp ,unsigned long& prevTimeStamp);
    int setPrevAcceleration(const vec3imu& acceleration_cur, vec3imu& acceleration_prev);
    int setPrevVelocity(const vec3imu& relVelocity_cur , vec3imu& relVelocity_prev);
};

#endif //ARTUS_IMU_TOOLKIT_IMU_SENSOR_H
