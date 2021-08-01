// I wrote this header file and Cpp file based on jrowberg's i2devlib (https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050_
// these are almost same with the library's example named 'MPU6050_DMP6'
// I just edit a few things from that example
// I really appreciate to this fantastic library and example


#ifndef MPU_H
#define MPU_H

#include <Arduino.h>

void mpu_setup();

float mpu_get_pitch();

#endif
