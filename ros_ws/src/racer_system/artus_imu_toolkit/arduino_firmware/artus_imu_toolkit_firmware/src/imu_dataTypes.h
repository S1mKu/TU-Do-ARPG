/*!
 * @file        imu_dataTypes.h
 * @brief       header | struct-class to store/interact with IMU/AHRS-Data Readings.
 * @mainpage    ARTUS IMU TOOLKIT
 * @author      Samanta Scharmacher <samanta.scharmacher@tu-dortmund.de>
 *
 * @note        if floatingPoint arithmetic is too slow consider using fixePoint arithmetic!
 */

#ifndef ARTUS_IMU_TOOLKIT_IMU_DATATYPES_H
#define ARTUS_IMU_TOOLKIT_IMU_DATATYPES_H

/************************************************************************************************************
TODO:
 - implementing direct calculating procedure for acceleration Offsets ans Gain Values.
 - implementing dynamically calibration Procedures to reCalibrate IMU-Sensors while running.
 - implement storing structure (adding attributes to already existing Container) to store callibration values as one record
 - implementing storing for covariances of an imu-sensor (depending on specific sensormoduls)s to dynamically reCalibrate IMU Sensors while running.
*************************************************************************************************************/

#if (ARDUINO >= 100)
#include "Arduino.h"
#endif

#define MS_2_SEC    0.001F

struct vec3imu{
    float x, y, z;
    
    vec3imu();
    vec3imu(float const x, float const y, float const z);

    bool operator==(vec3imu const & other) const;
    bool operator!=(vec3imu const & other) const;

    vec3imu operator+() const;
    vec3imu operator-() const;
    vec3imu operator+(vec3imu const & summand) const;
    vec3imu operator-(vec3imu const & subtrahend) const;
    vec3imu operator*(float const operand) const;
    vec3imu operator/(float const divisor) const;
    vec3imu & operator+=(vec3imu const & summand);
    vec3imu & operator-=(vec3imu const & subtrahend);
    vec3imu & operator*=(float const operand);
    vec3imu & operator/=(float const divisor);

    vec3imu operator+(float const & summand) const;
    vec3imu operator-(float const & subtrahend) const;
    vec3imu operator+(unsigned long const & summand) const;
    vec3imu operator-(unsigned long const & subtrahend) const;
    vec3imu operator*(unsigned long const operand) const;
    vec3imu operator/(unsigned long const divisor) const;

    /// For numerical Integration over time
    static vec3imu quadraturIntegration(vec3imu accumValues,unsigned long a,unsigned long b);
    static vec3imu sehnenTrapezFormel(vec3imu f_a, vec3imu f_b, unsigned long a, unsigned long b);
};

struct euler3imu {
    float roll,pitch,yaw;
    euler3imu(float roll_ , float pitch_ , float yaw_):roll(roll_),pitch(pitch_),yaw(yaw_){}
    euler3imu(): euler3imu(0.0f,0.0f,0.0f) {}
};

struct quaternion4imu {
    float qw,qx,qy,qz;
    quaternion4imu(float qw_, float qx_, float qy_, float qz_):qw(qw_),qx(qx_),qy(qy_),qz(qz_){}
    quaternion4imu() : quaternion4imu(0.0f,0.0f,0.0f,0.0f){}
};

typedef  float vec3offset[3];
typedef  float mat3x3gain[3][3];

struct imuDataContainer{
///------------ TimeStamp ---------
    ///* arduTimeStamp ~> [ms] since Arduino startUP: TimeStamp to determine the package order */
    unsigned long arduTimeStamp = 0;    /// ~> [ms] since Arduino startUP: TimeStamp to determine the package order */
///------------ Raw Sensor Values ---------
    ///RAW DATA (if at all ONLY LOWPASS FILTER applied!) ///
    vec3imu accel_raw;                  /// ~> [g]      =  in total 12 bytes (4Bytes for float *3 times)
    vec3imu gyro_raw;                   /// ~> [deg/s]  =  in total 12 bytes (4Bytes for float *3 times)
    vec3imu magno_raw;                  /// ~> [µT]     =  in total 12 bytes (4Bytes for float *3 times)
///------------ corrected Sensor Values ---------
    vec3imu accel_cor;                  /// ~> [g]      =  in total 12 bytes (4Bytes for float *3 times)
    vec3imu gyro_cor;                   /// ~> [deg/s]  =  in total 12 bytes (4Bytes for float *3 times)
    vec3imu magno_cor;                  /// ~> [µT]     =  in total 12 bytes (4Bytes for float *3 times)
///------------ Environment Values ---------
    float temp = 0;                     /// ~> [°C]     =  in total 4 bytes
    vec3imu gravity;                    /// ~> [g]      =  in total 12 bytes (4Bytes for float *3 times)
///------------ Orientation Values ---------
    /// Euler-Angle + Quaternions ~> calculated by filter ///
    euler3imu eulerAngle;               /// ~> Euler-Angle = in total 12 bytes (4Bytes for float *3 times)
    quaternion4imu quaternion;          /// ~> Quaternion  = in total 16 bytes (4Bytes for float *4 times)
///------------ Motion Values ---------
    /// gravity already substracted ///
    vec3imu linAcc;                     /// ~> [m/s²]   = in total 12 bytes (4Bytes for float *3 times)
    vec3imu angVel;                     /// ~> [rad/s]  = in total 12 bytes (4Bytes for float *3 times)
///------------ experimental ---------
    unsigned long prevTimeStamp = 0;    /// ~> [ms] since Arduino startUP: TimeStamp of Previous Measurment */
    /// relative Velocity := calculated with linear Acceleration over time
    vec3imu relVelocity;                /// ~> [m/s]    = in total 12 bytes (4Bytes for float *3 times)
    vec3imu acceleration_prev;          /// ~> [m/s²]   = in total 12 bytes (4Bytes for float *3 times)

    /// relative distance - calculated with linear Acceleration over time² || calculated with relativ Velocity over time
    vec3imu relDistance;                /// ~> [m]      = in total 12 bytes (4Bytes for float *3 times)
    vec3imu relVelocity_prev;           /// ~> [m/s]    = in total 12 bytes (4Bytes for float *3 times)
    //TODO: Are there more usefull Values to store?
};

#endif //ARTUS_IMU_TOOLKIT_IMU_DATATYPES_H
