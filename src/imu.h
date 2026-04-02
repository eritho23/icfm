#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <cstdint>
#include <Wire.h>
#include "Adafruit_ISM330DHCX.h"

#include "base.h"

typedef struct {
	f32 ax, ay, az; // raw accleration
	f32 wx, wy, wz; // raw gyro rates
} imu_m;

extern f32 imu_sample_rate_hz;

b32 imu_setup(f32 accel_avg_out[3]);
void imu_debug(void);
b32 get_imu_data(imu_m *out);

#endif
