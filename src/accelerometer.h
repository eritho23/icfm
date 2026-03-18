#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <Arduino.h>
#include <cstdint>
#include <Wire.h>
#include <MPU6050.h>

#include "base.h"

typedef struct {
	i16 ax; 
	i16 ay; 
	i16 az; 
	i16 gx; 
	i16 gy; 
	i16 gz; 
} acc_meas;

extern acc_meas mpu_data;
extern f32 mpu_sample_rate;

b32 mpu_setup(void);
void mpu_debug(void);
acc_meas* get_mpu_data(void);

#endif
