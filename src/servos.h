#ifndef SERVOS_H
#define SERVOS_H

#include <Arduino.h>
#include <ESP32Servo.h>

#include "base.h"

static const int servo_pins[4] = {2, 3, 10, 11};

extern Servo servos[4];

void servos_setup();
void test_servos();
void servos_write(f32 fin_angle_deg[4]);

#endif
