#include <Arduino.h>

#include "servo.h"

void setup() {
  Serial.begin(115200);
	servo_setup();
}

void loop() {
	test_servo();
}
