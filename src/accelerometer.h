#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"

extern int16_t ax, ay, az;
extern int16_t gx, gy, gz;

void setup_mpu();
void test_mpu();

#endif
