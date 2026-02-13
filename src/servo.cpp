#include <Arduino.h>
#include <ESP32Servo.h>

int servo_pin = 8;
Servo servo;

void servo_setup() {
	servo.attach(servo_pin); // 1KHz 10 bits
}

void test_servo() {
	servo.write(80);
	delay(800);
	servo.write(100);
	delay(800);
}
