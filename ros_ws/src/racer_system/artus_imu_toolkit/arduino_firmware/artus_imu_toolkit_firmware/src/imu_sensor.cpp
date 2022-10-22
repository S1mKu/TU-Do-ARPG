/*!
 * @file        imu_sensor.cpp
 * @brief       Abstract Class to interact with imu-Sensors.
 * @mainpage    ARTUS IMU TOOLKIT
 * @author      Samanta Scharmacher <samanta.scharmacher@tu-dortmund.de>
 *
 * @note        if floatingPoint arithmetic is too slow consider using fixePoint arithmetic!
 */

#include "imu_sensor.h"

IMU_sensor::IMU_sensor() {

    vec3offset accOffset_default =  {0.0,0.0 ,0.0};
    mat3x3gain accGain_default = {{1.0 , 1.0 , 1.0},
                                  {1.0 , 1.0 , 1.0},
                                  {1.0,1.0,1.0}};
    vec3offset gyroOffset_default = {0.0, 0.0, 0.0};
    vec3offset magOffset_bH_default = {0.0, 0.0, 0.0};
    mat3x3gain magGain_c_default = {{1.0 , 1.0 , 1.0},
                                    {1.0 , 1.0 , 1.0},
                                    {1.0, 1.0, 1.0 }};

    setAccelOffset(accOffset_default);
    setAccelGain(accGain_default);
    setGyroOffset(gyroOffset_default);
    setMagOffset(magOffset_bH_default);
    setMagGain(magGain_c_default);
}

IMU_sensor::~IMU_sensor(){}

int IMU_sensor::setAccelOffset(vec3offset new_acc_offset){
    for (int i = 0 ; i < 3 ; i++){
        this->accOffset_[i] = new_acc_offset[i];
    }
    return 0;
}

int IMU_sensor::setAccelGain(mat3x3gain new_acc_gain){
    for (int i = 0 ; i < 3 ; i++){
        for (int j = 0; j < 3; j++) {
            this->accGain_[i][j]=new_acc_gain[i][j];
        }
    }
    return 0;
}

int IMU_sensor::setGyroOffset(vec3offset new_gyro_offset){
    for (int i = 0 ; i < 3 ; i++){
        this->gyroOffset_[i] = new_gyro_offset[i];
    }
    return 0;
}

int IMU_sensor::setMagOffset(vec3offset new_mag_offset){
    for (int i = 0 ; i < 3 ; i++){
        this->magOffset_bH_[i] = new_mag_offset[i];
    }
    return 0;
}

int IMU_sensor::setMagGain(mat3x3gain  new_mag_gain){
    for (int i = 0 ; i < 3 ; i++){
        for (int j = 0; j < 3; j++) {
            this->magGain_c_[i][j]=new_mag_gain[i][j];
        }
    }
    return 0;
}

///TODO: is it working as desired?
//vec3offset IMU_sensor::getAccelOffset(){
//    return accOffset_;
//}
///TODO: is it working as desired?
//mat3x3gain IMU_sensor::getAccelGain( ){
//    return accGain_;
//}
///TODO: is it working as desired?
//vec3offset IMU_sensor::getGyroOffset( ){
//    return gyroOffset_;
//}
///TODO: is it working as desired?
//vec3offset IMU_sensor::getMagOffset( ){
//    return magOffset_bH_;
//}
///TODO: is it working as desired?
//mat3x3gain IMU_sensor::getMagGain( ){
//    return magGain_c_;
//}

int IMU_sensor::getRawAcceleration(imuDataContainer& imuValues){
    return getRawAcceleration(imuValues.accel_raw);
}

int IMU_sensor::getRawGyro(imuDataContainer& imuValues){
    return getRawGyro(imuValues.gyro_raw);
}

int IMU_sensor::getRawMagnetometer(imuDataContainer& imuValues){
    return getRawMagnetometer(imuValues.magno_raw);
}

int IMU_sensor::getTemperature(imuDataContainer& imuValues){
    return getTemperature(imuValues.temp);
}

int IMU_sensor::getCorAcceleration(imuDataContainer& imuValues){
    return getCorAcceleration(imuValues.accel_cor, imuValues.accel_raw);
}

int IMU_sensor::getCorGyro(imuDataContainer& imuValues){
    return getCorGyro(imuValues.gyro_cor, imuValues.gyro_raw);
}

int IMU_sensor::getCorMagnetometer(imuDataContainer& imuValues){
    return getCorMagnetometer(imuValues.magno_cor, imuValues.magno_raw);
}

int IMU_sensor::getEulerAngle(imuDataContainer& imuValues){
    return getEulerAngle(imuValues.eulerAngle);
}

int IMU_sensor::getQuaternion(imuDataContainer& imuValues){
    return getQuaternion(imuValues.quaternion);
}

int IMU_sensor::getLinearAcceleration(imuDataContainer& imuValues){
    return getLinearAcceleration(imuValues.linAcc, imuValues.gravity, imuValues.accel_cor);
}

int IMU_sensor::getAngularVelocity(imuDataContainer& imuValues){
    return getAngularVelocity(imuValues.angVel, imuValues.gyro_cor);
}

int IMU_sensor::getRelativSpeed(imuDataContainer& imuValues ){
    return getRelativSpeed(imuValues.relVelocity,imuValues.linAcc, imuValues.acceleration_prev,imuValues.arduTimeStamp, imuValues.prevTimeStamp);
}

int IMU_sensor::getRelativeDistance(imuDataContainer& imuValues){
    return getRelativeDistance(imuValues.relDistance, imuValues.relVelocity,imuValues.relVelocity_prev,imuValues.arduTimeStamp,imuValues.prevTimeStamp);
}

int IMU_sensor::getTimeStamp(imuDataContainer& imuValues){
    return getTimeStamp(imuValues.arduTimeStamp);
}

int IMU_sensor::getGravity(imuDataContainer& imuValues){
    return getGravity(imuValues.gravity);
}

int IMU_sensor::setPrevTimeStamp(imuDataContainer& imuValues){
    if(imuValues.prevTimeStamp == imuValues.arduTimeStamp){
        return 1;
    }
    imuValues.prevTimeStamp = imuValues.arduTimeStamp;
    return 0;
}

int IMU_sensor::setPrevAcceleration(imuDataContainer& imuValues){
    if(imuValues.acceleration_prev == imuValues.linAcc){
        return 1;
    }
    imuValues.acceleration_prev = imuValues.linAcc;
    return 0;
}

int IMU_sensor::setPrevVelocity(imuDataContainer& imuValues){
    if(imuValues.relVelocity_prev == imuValues.relVelocity){
        return 1;
    }
    imuValues.relVelocity_prev = imuValues.relVelocity;
    return 0;
}

int IMU_sensor::setPrevTimeStamp(const unsigned long&  arduTimeStamp , unsigned long& prevTimeStamp){
    if(prevTimeStamp == arduTimeStamp){
        return 1;
    }
    prevTimeStamp = arduTimeStamp;
    return 0;
}

int IMU_sensor::setPrevAcceleration(const vec3imu& acceleration_cur, vec3imu& acceleration_prev){
    if(acceleration_prev == acceleration_cur){
        return 1;
    }
    acceleration_prev = acceleration_cur;
    return 0;
}

int IMU_sensor::setPrevVelocity(const vec3imu& relVelocity_cur , vec3imu& relVelocity_prev){
    if(relVelocity_prev == relVelocity_cur){
        return 1;
    }
    relVelocity_prev = relVelocity_cur;
    return 0;
}