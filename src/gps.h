#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <SoftwareSerial.h>

extern SFE_UBLOX_GNSS gps;
extern SoftwareSerial mySerial;
extern long lastTime;

void gpsSetup();
void testGps();

#endif
