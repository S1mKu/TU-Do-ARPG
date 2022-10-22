/*!
 * @file        artus_LSM6DSOX_LIS3MDL.cpp
 * @brief       Class to interact with LSM6DSOX & LIS3MDL imu-Sensors.
 * @mainpage    ARTUS IMU TOOLKIT
 * @author      Samanta Scharmacher <samanta.scharmacher@tu-dortmund.de>
 *
 * @note        if floatingPoint arithmetic is too slow consider using fixePoint arithmetic!
 */

#include "artus_LSM6DSOX_LIS3MDL.h"

artus_LSM6DSOX_LIS3MDL::artus_LSM6DSOX_LIS3MDL(){//} : filter_(filter) {
    vec3offset accOffset_arda = {0.00080732 , 0.012271 , 0.0052966};
    mat3x3gain accGain_arda = {{0.99455 , 0.0024532 , 0.0030739},
                               {-0.013746 , 1.0016 , -0.022257},
                               {0.029301 , 0.046798 , 0.99584}};
    vec3offset gyroOffset_arda = {-0.36530, 0.16497, -0.58260};
    vec3offset magOffset_bH_arda = {-80.78, 26.91, -46.47};
    mat3x3gain magGain_c_arda = {{0.98 , 0.044 , -0.003 },
                                 {0.044,1.045,0.013  },
                                 {-0.003, 0.013, 0.979 }};

    setAccelOffset(accOffset_arda);
    setAccelGain(accGain_arda);
    setGyroOffset(gyroOffset_arda);
    setMagOffset(magOffset_bH_arda);
    setMagGain(magGain_c_arda);
}

int artus_LSM6DSOX_LIS3MDL::imu_init(){
    if (!lsm6ds.begin_I2C()) {
        return 1;
    }
    accelerometer = lsm6ds.getAccelerometerSensor();
    gyroscope = lsm6ds.getGyroSensor();
    temperature = lsm6ds.getTemperatureSensor();
    if (!lis3mdl.begin_I2C(0x1C, &Wire)) {
        return 2;
    }
    magnetometer = &lis3mdl;
    return 0;
}

int artus_LSM6DSOX_LIS3MDL::imu_start(){
    /// set lowest range
    lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

    /// set slightly above refresh rate
    lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

    ///Start filter Instance to filter/"correct" Output
    filter_.begin(FILTER_UPDATE_RATE_HZ);

    return 0;
}

int artus_LSM6DSOX_LIS3MDL::updatedIMUvalues(){
    /// After calling update() the ax, ay, az, gx, gy, gz, mx, my, mz, time, and temerature class variables are all updated.
    /// Access them by placing the object in front:
    sensors_event_t accel, gyro, mag, temp;

    if(!(accelerometer->getEvent(&accel))){
        return 1;
    }

    if (!(gyroscope->getEvent(&gyro))){
        return 2;
    }

    if(!(magnetometer->getEvent(&mag))) {
        return 3;
    }

    if (!(temperature->getEvent(&temp))){
        return 4;
    }
    return 0;
}

int artus_LSM6DSOX_LIS3MDL::updatedIMUvalues(imuDataContainer & imuValues){
///~~~~~~~~~~~~~ TimeStamp ~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
    getTimeStamp(imuValues.arduTimeStamp) ;
///~~~~~~~~~~~~~ RAW Values ~~~~~~~~~~~~~~~~~~~~~~~~~~~//
    getRawAcceleration(imuValues.accel_raw) ;
    getRawGyro(imuValues.gyro_raw) ;
    getRawMagnetometer(imuValues.magno_raw) ;
    getTemperature(imuValues.temp);
///~~~~~~~~~~~~~ Corrected Values ~~~~~~~~~~~~~~~~~~~~~//
    getRawAcceleration(imuValues.accel_cor) ;
    getRawGyro(imuValues.gyro_cor) ;
    getRawMagnetometer(imuValues.magno_cor) ;
///------------- update filter ------------------------///
    filter_.update(imuValues.gyro_cor.x , imuValues.gyro_cor.y, imuValues.gyro_cor.z, imuValues.accel_cor.x, imuValues.accel_cor.y, imuValues.accel_cor.z,  imuValues.magno_cor.x, imuValues.magno_cor.y, imuValues.magno_cor.z);
///~~~~~~~~~~~~~ Euler Angle ~~~~~~~~~~~~~~~~~~~~~~~~~~//
    getEulerAngle(imuValues.eulerAngle);
///~~~~~~~~~~~~~ Quaternions ~~~~~~~~~~~~~~~~~~~~~~~~~~//
    getQuaternion(imuValues.quaternion);
///~~~~~~~~~~~~~ Linear Acceleration ~~~~~~~~~~~~~~~~~~//
    getLinearAcceleration(imuValues.linAcc, imuValues.gravity, imuValues.accel_cor);
///~~~~~~~~~~~~~ Angular Velocity ~~~~~~~~~~~~~~~~~~~~~//
    getAngularVelocity(imuValues.angVel,imuValues.gyro_cor);
///~~~~~~~~~~~~~ relative Velocity ~~~~~~~~~~~~~~~~~~~~//
    getRelativSpeed(imuValues.relVelocity, imuValues.linAcc,imuValues.acceleration_prev, imuValues.arduTimeStamp, imuValues.prevTimeStamp);
///~~~~~~~~~~~~~ relativ Distance ~~~~~~~~~~~~~~~~~~~~//
    getRelativeDistance(imuValues.relDistance,imuValues.relVelocity, imuValues.relVelocity_prev,imuValues.arduTimeStamp,imuValues.prevTimeStamp);
///------------- Set Values for next Iteration ------///
    setPrevTimeStamp(imuValues);
    setPrevAcceleration(imuValues);
    setPrevVelocity(imuValues);

    return 0;
}

int artus_LSM6DSOX_LIS3MDL::getRawAcceleration(vec3imu& accelRawValues){
    sensors_event_t accel;
    if(!(accelerometer->getEvent(&accel))){
        return 1;
    }
    accelRawValues.x = accel.acceleration.x * SENSORS_MS2_TO_g;     //~> in [g]
    accelRawValues.y = accel.acceleration.y * SENSORS_MS2_TO_g;     //~> in [g]
    accelRawValues.z = accel.acceleration.z * SENSORS_MS2_TO_g;     //~> in [g]
    return 0;
}

int artus_LSM6DSOX_LIS3MDL::getRawGyro(vec3imu& gyroRawValues){
    sensors_event_t gyro;
    if (!(gyroscope->getEvent(&gyro))){
        return 1;
    }
    gyroRawValues.x = gyro.gyro.x * SENSORS_RADS_TO_DPS;            //~> to [°/s]
    gyroRawValues.y = gyro.gyro.y * SENSORS_RADS_TO_DPS;            //~> to [°/s]
    gyroRawValues.z = gyro.gyro.z * SENSORS_RADS_TO_DPS;            //~> to [°/s]
    return 0;
}

int artus_LSM6DSOX_LIS3MDL::getRawMagnetometer(vec3imu& magRawValues){
    sensors_event_t mag;
    if(!(magnetometer->getEvent(&mag))) {
        return 1;
    }
    magRawValues.x = mag.magnetic.x ;                               //~> to [µT]
    magRawValues.y = mag.magnetic.y ;                               //~> to [µT]
    magRawValues.z = mag.magnetic.z ;                               //~> to [µT]
    return 0;
}

int artus_LSM6DSOX_LIS3MDL::getTemperature(float& temperatureValue){
    sensors_event_t temp;
    if (!(temperature->getEvent(&temp))){
        return 1;
    }
    temperatureValue = temp.temperature;                            //~> to [°C]
    return 0;
}

int artus_LSM6DSOX_LIS3MDL::getCorAcceleration(vec3imu& accelCorValues, const vec3imu& accelRawValues){
// ===== Value Correction ============================================================================================
//    (a_cor)=(acc_Matrix)*(a_raw-bH) + (acc_offset)
//     ~> a_cor_x = (acc11*a_raw_x + acc12*a_raw_y + acc13*a_raw_z)+acc_Offset_10
// ~~~~ At Car ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// --- Accel (Offset) ---
//     float acc_offset[3] = {0.00080732,0.012271,0.0052966};
// --- Scale… ---
//     float acc[3][3] = {{0.99455,0.0024532,0.0030739},{-0.013746,1.0016,-0.022257},{0.029301,0.046798,0.99584}};
// ===== Applying Correction =========================================================================================
    accelCorValues.x = (accGain_[0][0]*accelRawValues.x + accGain_[0][1]*accelRawValues.y + accGain_[0][2]*accelRawValues.z)+ accOffset_[0];
    accelCorValues.y = (accGain_[1][0]*accelRawValues.x + accGain_[1][1]*accelRawValues.y + accGain_[1][2]*accelRawValues.z)+ accOffset_[1];
    accelCorValues.z = (accGain_[2][0]*accelRawValues.x + accGain_[2][1]*accelRawValues.y + accGain_[2][2]*accelRawValues.z)+ accOffset_[2];
    return 0;
}

int artus_LSM6DSOX_LIS3MDL::getCorGyro(vec3imu& gyroCorValues , const vec3imu& gyroRawValues){
// ===== Value Correction ============================================================================================
// --- Simple Offset…… for now
//      float gyroOffset[3] = {{-0.36530, 0.16497, -0.58260};
// ===== Applying Correction =========================================================================================
    gyroCorValues.x = gyroRawValues.x-(gyroOffset_[0]);
    gyroCorValues.y = gyroRawValues.y-(gyroOffset_[1]);
    gyroCorValues.z = gyroRawValues.z-(gyroOffset_[2]);
    return 0;
}

int artus_LSM6DSOX_LIS3MDL::getCorMagnetometer(vec3imu& magCorValues, const vec3imu& magRawValues){
// ===== Value Correction ============================================================================================
//    (m_cor)=(cMatrixByMotionCal)*(m_raw-bH_offsetByMotionCal)
//     ~> m_cor_x = c00(m_raw_x-bH0)+c01(m_raw_y-bH1) + c02(m_raw_z-bH2)
// ~~~~ At My Home ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// --- HardIron (Offset) ---
//     float bH[3] = {-80.78, 26.91, -46.47};
// --- SoftIron etc. ---
//     float c[3][3] = {{0.98,0.044, -0.003 },{0.044,1.045,0.013  },{-0.003, 0.013, 0.979 }};
// ~~~~ At Car ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// --- HardIron (Offset) ---
//     float bH[3] = {2.63, 12.79, -37.02};
// --- SoftIron etc. ---
//     float c[3][3] = {{1.009,0.0039, -0.071},{0.039,1.073, 0.020  },{-0.071, 0.020, 0.931 }};
// ===== Applying Correction =========================================================================================
    magCorValues.x = magGain_c_[0][0]*(magRawValues.x-magOffset_bH_[0]) + magGain_c_[0][1]*(magRawValues.y-magOffset_bH_[1]) + magGain_c_[0][2]*(magRawValues.z-magOffset_bH_[2]);
    magCorValues.y = magGain_c_[1][0]*(magRawValues.x-magOffset_bH_[0]) + magGain_c_[1][1]*(magRawValues.y-magOffset_bH_[1]) + magGain_c_[1][2]*(magRawValues.z-magOffset_bH_[2]);
    magCorValues.z = magGain_c_[1][0]*(magRawValues.x-magOffset_bH_[0]) + magGain_c_[2][1]*(magRawValues.y-magOffset_bH_[1]) + magGain_c_[2][2]*(magRawValues.z-magOffset_bH_[2]);
    return 0;
}

int artus_LSM6DSOX_LIS3MDL::getEulerAngle(float& roll,float& pitch, float& yaw){
    roll = (filter_.getRoll());
    pitch = (filter_.getPitch());
    yaw = (filter_.getYaw());
    return 0;
}

int artus_LSM6DSOX_LIS3MDL::getEulerAngle(euler3imu& eulerAngle){
    return getEulerAngle(eulerAngle.roll,eulerAngle.pitch,eulerAngle.yaw);
}

int artus_LSM6DSOX_LIS3MDL::getQuaternion(float& qw, float& qx, float& qy, float& qz ){
    filter_.getQuaternion(&(qw), &(qx), &(qy), &(qz));
    return 0;
}

int artus_LSM6DSOX_LIS3MDL::getQuaternion(quaternion4imu&  quaternion){
    return getQuaternion(quaternion.qw, quaternion.qx, quaternion.qy, quaternion.qz);
}

int artus_LSM6DSOX_LIS3MDL::getLinearAcceleration(vec3imu& linearAccel, vec3imu& gravity , const vec3imu& accelCorValues){
    getGravity(gravity);
    linearAccel = (accelCorValues - gravity) * SENSORS_g_TO_MS2;    //~> to [m/s²]
    return 0;
}

int artus_LSM6DSOX_LIS3MDL::getAngularVelocity(vec3imu& angularVel , const vec3imu& gyroCorValues){
    angularVel = gyroCorValues * SENSORS_DPS_TO_RADS;               //~> [rad/s]
    return 0;
}

int artus_LSM6DSOX_LIS3MDL::getRelativSpeed(vec3imu& relSpeed, vec3imu& acceleration_cur, vec3imu& acceleration_prev, const unsigned long& curTimeStamp , const unsigned long& prevTimeStamp ){
    if (curTimeStamp == prevTimeStamp){
        return 1;
    }
    if (acceleration_cur == acceleration_prev){
        return 2;
    }
    if (acceleration_prev == vec3imu()){
        return 3;
    }
    relSpeed = vec3imu::sehnenTrapezFormel(acceleration_prev,acceleration_cur,prevTimeStamp,curTimeStamp);
    return 0;
}

int artus_LSM6DSOX_LIS3MDL::getRelativeDistance(vec3imu& relDistance,vec3imu& relVelocity_cur, vec3imu& relVelocity_prev, const unsigned long& curTimeStamp , const unsigned long& prevTimeStamp ){
    if (curTimeStamp == prevTimeStamp){
        return 1;
    }
    if (relVelocity_cur == relVelocity_prev){
        return 2;
    }
    if (relVelocity_prev == vec3imu()){
        return 3;
    }
    relDistance = vec3imu::sehnenTrapezFormel(relVelocity_prev,relVelocity_cur,prevTimeStamp,curTimeStamp);
    return 0;
}

int artus_LSM6DSOX_LIS3MDL::getTimeStamp(unsigned long& arduTimeStamp){
    arduTimeStamp = millis();
    return 0;
}

int artus_LSM6DSOX_LIS3MDL::getGravity(vec3imu& gravity) {
    filter_.getGravityVector(&gravity.x,&gravity.y,&gravity.z);
    return 0;
}