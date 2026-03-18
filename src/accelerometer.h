#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <Arduino.h>
#include <cstdint>
#include <Wire.h>
#include <MPU6050.h>

#include "base.h"

typedef struct {
	f32 ax;
	f32 ay;
	f32 az;
	f32 roll;
	f32 pitch;
	f32 yaw;
} acc_meas;

extern f32 mpu_sample_rate_hz;

b32 mpu_setup(void);
void mpu_debug(void);
b32 get_mpu_data(acc_meas *out);

#endif
