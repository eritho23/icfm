#ifndef ICFM_UTILS_H
#define ICFM_UTILS_H

#include <Arduino.h>
#include <Wire.h>

#include "../include/base.h"

b32 serial_read_line(char *out, int max_len);
void i2c_scan();

#endif
