#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <Arduino.h>

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup_mpu();
void test_mpu();

#endif
