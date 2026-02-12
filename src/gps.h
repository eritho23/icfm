#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <SoftwareSerial.h>

extern SFE_UBLOX_GNSS gps;
extern SoftwareSerial my_serial;
extern long last_sime;

void gps_setup();
void test_gps();

#endif
