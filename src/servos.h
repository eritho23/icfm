#ifndef SERVOS_H
#define SERVOS_H

#include <Arduino.h>
#include <ESP32Servo.h>

int servo_pin = 8;

extern Servo servo;

void servo_setup();
void test_servo();

#endif
