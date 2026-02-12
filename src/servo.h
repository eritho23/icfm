#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>
#include <ESP32Servo.h>

extern int servo_pin;
extern Servo servo;

void servo_setup();
void test_servo();

#endif
