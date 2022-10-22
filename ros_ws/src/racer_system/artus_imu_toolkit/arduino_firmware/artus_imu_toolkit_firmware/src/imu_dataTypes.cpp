/*!
 * @file        imu_dataTypes.cpp
 * @brief       header | struct-class to store/interact with IMU/AHRS-Data Readings.
 * @mainpage    ARTUS IMU TOOLKIT
 * @author      Samanta Scharmacher <samanta.scharmacher@tu-dortmund.de>
 *
 * @note        if floatingPoint arithmetic is too slow consider using fixePoint arithmetic!
 */

#include "imu_dataTypes.h"

vec3imu::vec3imu() : vec3imu(0.f, 0.f, 0.f) {/*empty*/}

vec3imu::vec3imu(float const x, float const y, float const z): x(x), y(y), z(z){ /*empty*/}

bool vec3imu::operator==(vec3imu const & other) const{
    return ((x == other.x) && (y == other.y) && (z == other.z));
}

bool vec3imu::operator!=(vec3imu const & other) const{
    return !(operator==(other));
}

vec3imu vec3imu::operator+() const {
    return *this;
}

vec3imu vec3imu::operator-() const {
    return vec3imu{-x,-y,-z};
}

vec3imu vec3imu::operator+(vec3imu const & summand) const {
    return vec3imu{x + summand.x, y + summand.y,z + summand.z};
}

vec3imu vec3imu::operator-(vec3imu const & subtrahend) const {
    return vec3imu{x - subtrahend.x, y - subtrahend.y, z - subtrahend.z};
}

vec3imu vec3imu::operator*(float const operand) const {
    return vec3imu{x * operand, y * operand, z * operand};
}

vec3imu vec3imu::operator/(float const divisor) const {
    return vec3imu{x / divisor, y / divisor, z / divisor};
}

vec3imu & vec3imu::operator+=(vec3imu const & summand) {
    x += summand.x;
    y += summand.y;
    z += summand.z;
    return *this;
}

vec3imu & vec3imu::operator-=(vec3imu const & subtrahend){
    x -= subtrahend.x;
    y -= subtrahend.y;
    z -= subtrahend.z;
    return *this;
}

vec3imu & vec3imu::operator*=(float const operand) {
    x *= operand;
    y *= operand;
    z *= operand;
    return *this;
}

vec3imu & vec3imu::operator/=(float const divisor) {
    x /= divisor;
    y /= divisor;
    z /= divisor;
    return *this;
}

vec3imu vec3imu::operator+(float const & summand) const {
    return vec3imu{x + summand, y + summand,z + summand};
}

vec3imu vec3imu::operator-(float const & subtrahend) const {
    return vec3imu{x - subtrahend, y - subtrahend, z - subtrahend};
}

vec3imu vec3imu::operator+(unsigned long const & summand) const {
    return vec3imu{x + (float)summand, y + (float) summand, z + (float) summand};
}

vec3imu vec3imu::operator-(unsigned long const & subtrahend) const {
    return vec3imu{x - (float) subtrahend, y - (float) subtrahend, z - (float) subtrahend};
}

vec3imu vec3imu::operator*(unsigned long const operand) const {
    return vec3imu{x * (float) operand, y * (float) operand, z * (float) operand};
}

vec3imu vec3imu::operator/(unsigned long const divisor) const {
    return vec3imu{x / (float) divisor, y / (float) divisor, z / (float) divisor};
}

vec3imu vec3imu::quadraturIntegration(vec3imu accumValues , unsigned long a, unsigned long b){
    return (accumValues * ((float)(b-a)));
}

vec3imu vec3imu::sehnenTrapezFormel(vec3imu f_a, vec3imu f_b, unsigned long a, unsigned long b){
    return ((f_a+f_b) * 0.5f) * ((b-a) * MS_2_SEC);
}
