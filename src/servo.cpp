#include <Arduino.h>

#include <ESP32Servo.h>

int servoPin = 8;
Servo servo;

void servoSetup() {
	servo.attach(servoPin); // 1KHz 10 bits
}

void testServo() {
	servo.write(80);
	delay(800);
	servo.write(100);
	delay(800);
}
