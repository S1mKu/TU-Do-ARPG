/*!
 * @file        artus_mpu9250.cpp
 * @brief       Class to interact with MPU9250 imu-Sensors.
 * @mainpage    ARTUS IMU TOOLKIT
 * @author      Samanta Scharmacher <samanta.scharmacher@tu-dortmund.de>
 *
 * @note        if floatingPoint arithmetic is too slow consider using fixePoint arithmetic!
 */

#include "artus_mpu9250.h"

artus_mpu9250::artus_mpu9250(){//} : filter_(filter) {
    vec3offset accOffset_mpu9250 = {-0.020649,-0.039069,0.013311};
    mat3x3gain accGain_mpu9250 = {{0.99758 , 0.045014 , -0.025334},
                                  {-0.01319 , 0.99653 , 0.033928},
                                  {0.080139,0.0037845,0.98521}};
    vec3offset gyroOffset_mpu9250 = {-1.44449, 0.78518, -0.27658};
    vec3offset magOffset_bH_mpu9250 = {65.63, -17.57, 63.86};
    mat3x3gain magGain_c_mpu9250 = {{1.030 , 0.005 , 0.048},
                                    {0.005 , 0.993 , 0.009},
                                    {0.048, -0.009, 0.980 }};

    setAccelOffset(accOffset_mpu9250);
    setAccelGain(accGain_mpu9250);
    setGyroOffset(gyroOffset_mpu9250);
    setMagOffset(magOffset_bH_mpu9250);
    setMagGain(magGain_c_mpu9250);
}

int artus_mpu9250::imu_init(){
    /// Call imu.begin() to verify communicaton as well as initializing the IMU (artus_mpu9250) to it's default values
    /// On Succes this function will return `INV_SUCESS`(0), that indicates, that the IMU of given type was set up succesfully.
    return imu.begin();
}

int artus_mpu9250::imu_start(){
    /// Use setSensors to enable ALL MPU-9250 sensors to get most Information a combination of following defines are possible:
    /// INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS, INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

    /// Use setGyroFSR() and setAccelFSR() to configure the gyroscope and accelerometer full scale ranges.
    /// Gyro options are +/- 250, 500, 1000, or 2000 dps ~> The smaller the value lesser noise will appear, but the system will also be less sensitive detecting changes…??
    /// Accel options are +/- 2, 4, 8, or 16 g ~> Assuming, that the maximum acceleration will not exceed -1g to +1g ~> Analogous to the gyro: smaller scale factore leads to (mostly) less noise
    imu.setGyroFSR(GYRO_250_DPS);   // Set gyro to 250 dps
    imu.setAccelFSR(ACCEL_2_G);     // Set accel to +/-2g
    /* Note: the MPU-9250's magnetometer FSR is set at +/- 4912 uT (micro-tesla's)*/

    /// The sample rate of the accel/gyro can be set using setSampleRate.
    /// Acceptable values range from 4Hz to 1kHz
    imu.setSampleRate(SMPL_100_HZ); // Set sample rate to 100Hz

    /// Likewise, the compass (magnetometer) sample rate can be set using the setCompassSampleRate() function.
    /// This value can range between: 1-100Hz
    imu.setCompassSampleRate(mSMPL_100_HZ); // Set mag rate to 100Hz
    /// setLPF() can be used to set the digital low-pass filter of the accelerometer and gyroscope.
    /// Can be any of the following: 188, 98, 42, 20, 10, 5(values are in Hz).
    /// The lower the value, the more aggressive the DLPF (Digital Low Pass Filter) and the less responsive the system will be
    /// Setting LowPass Filter as last Value, because setSampleRate will change LPF frequenz automatically… :/
    imu.setLPF(LPF_5_HZ); // Set LPF corner frequency to 5Hz ~> best filter but

    ///Start filter Instance to filter/"correct" Output
    filter_.begin(FILTER_UPDATE_RATE_HZ);

    return 0;
}

int artus_mpu9250::updatedIMUvalues(){
    /// dataReady() checks to see if new accel/gyro data is available. It will return a boolean true or false
    /// (New magnetometer data cannot be checked, as the library runs that sensor in single-conversion mode.)
    if ( !imu.dataReady() ) {
        return 1;
    }
    /// Call update() to update the imu objects sensor data. You can specify which sensors to update by combining:
    /// UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or UPDATE_TEMP. [Default: update accel, gyro, compass]
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS | UPDATE_TEMP);
    return 0;
}

int artus_mpu9250::updatedIMUvalues(imuDataContainer & imuValues){
    if ( !imu.dataReady() ) {
        return 1;
    }

    /// Call update() to update the imu objects sensor data. You can specify which sensors to update by combining:
    /// UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or UPDATE_TEMP. [Default: update accel, gyro, compass]
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS | UPDATE_TEMP);

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

int artus_mpu9250::getRawAcceleration(vec3imu& accelRawValues){
    accelRawValues.x = imu.calcAccel(imu.ax);       //~> in [g]
    accelRawValues.y = imu.calcAccel(imu.ay);       //~> in [g]
    accelRawValues.z = imu.calcAccel(imu.az);       //~> in [g]
    return 0;
}

int artus_mpu9250::getRawGyro(vec3imu& gyroRawValues){
    gyroRawValues.x = imu.calcGyro(imu.gx);         //~> to [°/s]
    gyroRawValues.y = imu.calcGyro(imu.gy);         //~> to [°/s]
    gyroRawValues.z = imu.calcGyro(imu.gz);         //~> to [°/s]
    return 0;
}

int artus_mpu9250::getRawMagnetometer(vec3imu& magRawValues){
    //Axis rotation to let z point up !!!MUP_9250 ONLY!!!
    magRawValues.x = (-1.0)*imu.calcMag(imu.my);    //~> to [µT]
    magRawValues.y = (-1.0)*imu.calcMag(imu.mx);    //~> to [µT]
    magRawValues.z = (-1.0)*imu.calcMag(imu.mz);    //~> to [µT]
    return 0;
}

int artus_mpu9250::getTemperature(float& temperatureValue){
    long rawTemp = imu.temperature;
    //It seems, that the orignaly returned Value isn't as accurate: redefine Calculation
    temperatureValue = ((((rawTemp*1.0/65536L)-35)*321)-ROOM_TEMPERATURE_OFFSET)/TEMPERATURE_SENSITIVITY+21.0;
    return 0;   //~> to [°C]
}

int artus_mpu9250::getCorAcceleration(vec3imu& accelCorValues, const vec3imu& accelRawValues){
// ===== Value Correction ============================================================================================
//    (a_cor)=(acc_Matrix)*(a_raw-bH) + (acc_offset)
//     ~> a_cor_x = (acc11*a_raw_x + acc12*a_raw_y + acc13*a_raw_z)+acc_Offset_10
// ~~~~ At Car ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// --- Accel (Offset) ---
//     float acc_offset[3] = {-0.020649,-0.039069,0.013311};
// --- Scale… ---
//     float acc[3][3] = {{0.99758,0.045014,-0.025334},{-0.01319,0.99653,0.033928},{0.080139,0.0037845,0.98521}};
// ===== Applying Correction =========================================================================================
    accelCorValues.x = (accGain_[0][0]*accelRawValues.x + accGain_[0][1]*accelRawValues.y + accGain_[0][2]*accelRawValues.z)+ accOffset_[0];
    accelCorValues.y = (accGain_[1][0]*accelRawValues.x + accGain_[1][1]*accelRawValues.y + accGain_[1][2]*accelRawValues.z)+ accOffset_[1];
    accelCorValues.z = (accGain_[2][0]*accelRawValues.x + accGain_[2][1]*accelRawValues.y + accGain_[2][2]*accelRawValues.z)+ accOffset_[2];
    return 0;
}

int artus_mpu9250::getCorGyro(vec3imu& gyroCorValues , const vec3imu& gyroRawValues){
// ===== Value Correction ============================================================================================
// --- Simple Offset…… for now
//      float gyroOffset[3] = {-1.44449, 0.78518, -0.27658};
// ===== Applying Correction =========================================================================================
    gyroCorValues.x = gyroRawValues.x-(gyroOffset_[0]);
    gyroCorValues.y = gyroRawValues.y-(gyroOffset_[1]);
    gyroCorValues.z = gyroRawValues.z-(gyroOffset_[2]);
    return 0;
}

int artus_mpu9250::getCorMagnetometer(vec3imu& magCorValues, const vec3imu& magRawValues){
// ===== Value Correction ============================================================================================
//    (m_cor)=(cMatrixByMotionCal)*(m_raw-bH_offsetByMotionCal)
//     ~> m_cor_x = c00(m_raw_x-bH0)+c01(m_raw_y-bH1) + c02(m_raw_z-bH2)
// ~~~~ At My Home ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// --- HardIron (Offset) ---
//     float bH[3] = {65.63, -17.57, 63.86};
// --- SoftIron etc. ---
//     float c[3][3] = {{1.030,0.005, 0.048 },{0.005,0.993,0.009  },{0.048, -0.009, 0.980 }};
// ~~~~ At Car ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// --- HardIron (Offset) ---
//     float bH[3] = {-30.25, -12.32, 60.18};
// --- SoftIron etc. ---
//     float c[3][3] = {{1.025,0.0001, 0.074},{0.0001,1.006, -0.005  },{0.074, -0.005, 0.974 }};
// ===== Applying Correction =========================================================================================
    magCorValues.x = magGain_c_[0][0]*(magRawValues.x-magOffset_bH_[0]) + magGain_c_[0][1]*(magRawValues.y-magOffset_bH_[1]) + magGain_c_[0][2]*(magRawValues.z-magOffset_bH_[2]);
    magCorValues.y = magGain_c_[1][0]*(magRawValues.x-magOffset_bH_[0]) + magGain_c_[1][1]*(magRawValues.y-magOffset_bH_[1]) + magGain_c_[1][2]*(magRawValues.z-magOffset_bH_[2]);
    magCorValues.z = magGain_c_[1][0]*(magRawValues.x-magOffset_bH_[0]) + magGain_c_[2][1]*(magRawValues.y-magOffset_bH_[1]) + magGain_c_[2][2]*(magRawValues.z-magOffset_bH_[2]);
    return 0;
}

int artus_mpu9250::getEulerAngle(float& roll,float& pitch, float& yaw){
    roll = (filter_.getRoll());
    pitch = (filter_.getPitch());
    yaw = (filter_.getYaw());
    return 0;
}

int artus_mpu9250::getEulerAngle(euler3imu& eulerAngle){
    return getEulerAngle(eulerAngle.roll,eulerAngle.pitch,eulerAngle.yaw);
}

int artus_mpu9250::getQuaternion(float& qw, float& qx, float& qy, float& qz ){
    filter_.getQuaternion(&(qw), &(qx), &(qy), &(qz));
    return 0;
}

int artus_mpu9250::getQuaternion(quaternion4imu&  quaternion){
    return getQuaternion(quaternion.qw, quaternion.qx, quaternion.qy, quaternion.qz);
}

int artus_mpu9250::getLinearAcceleration(vec3imu& linearAccel, vec3imu& gravity , const vec3imu& accelCorValues){
    getGravity(gravity);
    linearAccel = (accelCorValues - gravity) * SENSORS_g_TO_MS2;  //~> to [m/s²]
    return 0;
}

int artus_mpu9250::getAngularVelocity(vec3imu& angularVel , const vec3imu& gyroCorValues){
    angularVel = gyroCorValues * SENSORS_DPS_TO_RADS;  //~> [rad/s]
    return 0;
}

int artus_mpu9250::getRelativSpeed(vec3imu& relSpeed, vec3imu& acceleration_cur, vec3imu& acceleration_prev, const unsigned long& curTimeStamp , const unsigned long& prevTimeStamp ){
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

int artus_mpu9250::getRelativeDistance(vec3imu& relDistance,vec3imu& relVelocity_cur, vec3imu& relVelocity_prev, const unsigned long& curTimeStamp , const unsigned long& prevTimeStamp ){
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

int artus_mpu9250::getTimeStamp(unsigned long& arduTimeStamp){
    arduTimeStamp = millis();
    return 0;
}

int artus_mpu9250::getGravity(vec3imu& gravity) {
    filter_.getGravityVector(&gravity.x,&gravity.y,&gravity.z);
    return 0;
}