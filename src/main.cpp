#include <Arduino.h>
#include <servo.h>

void setup()
{
  Serial.begin(115200);
	servoSetup();
}

void loop()
{
	testServo();
}
