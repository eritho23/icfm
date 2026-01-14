#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>
#include <ESP32Servo.h>

extern int servoPin;
extern Servo servo;

void servoSetup();
void testServo();

#endif
