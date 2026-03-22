#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <Arduino.h>
#include <cstdint>
#include <Wire.h>
#include <MPU6050.h>

#include "base.h"

typedef struct {
	f32 ax, ay, az; // raw accleration
	f32 wx, wy, wz; // raw gyro rates
} imu_m;

extern f32 imu_sample_rate_hz;

b32 imu_setup(void);
void imu_debug(void);
b32 get_imu_data(imu_m *out);

#endif
