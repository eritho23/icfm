#ifndef ICFM_GPS_H
#define ICFM_GPS_H

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <math.h>

#include "../include/base.h"

extern SFE_UBLOX_GNSS gps;
extern long last_time;

typedef struct {
  f32 x_east_m;
  f32 y_north_m;
  f32 z_up_m;
  u32 i_tow_ms;
  b32 valid;
  b32 fresh;
} gps_measurement_t;

void gps_setup();
b32 gps_read_local_enu(gps_measurement_t *out);

#endif
