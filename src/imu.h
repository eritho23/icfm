#ifndef ICFM_IMU_H
#define ICFM_IMU_H

#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LSM6DS.h>
#include <Arduino.h>
#include <Wire.h>

#include "../include/base.h"

typedef struct {
  f32 ax, ay, az; // raw accleration
  f32 wx, wy, wz; // raw gyro rates
} imu_measurement_t;

extern f32 imu_sample_rate_hz;

b32 imu_init(void);
b32 imu_calibrate(void); // blocks for 10s

void imu_debug(void);
b32 get_imu_data(imu_measurement_t *out);

#endif
