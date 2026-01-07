#include <Arduino.h>
#include "ESP32Servo.h"

// #include <SparkFun_u-blox_GNSS_Arduino_Library.h>
// SFE_UBLOX_GNSS myGNSS;
//
// #include <SoftwareSerial.h>
// SoftwareSerial mySerial(20, 21); // RX, TX

#include <ESP32Servo.h>

// long lastTime = 0; // Simple local timer. Limits amount of I2C traffic to u-blox module.

int servoPin = 8;
Servo servo;

void setup()
{
  Serial.begin(115200);
	servo.attach(servoPin); // 1KHz 10 bits

  // // Assume that the U-Blox GNSS is running at 9600 baud (the default) or at 38400 baud.
  // // Loop until we're in sync and then ensure it's at 38400 baud.
  // do {
  //   Serial.println("GNSS: trying 38400 baud");
  //   mySerial.begin(38400);
  //   if (myGNSS.begin(mySerial) == true) break;
  //
  //   delay(100);
  //   Serial.println("GNSS: trying 9600 baud");
  //   mySerial.begin(9600);
  //   if (myGNSS.begin(mySerial) == true) {
  //       Serial.println("GNSS: connected at 9600 baud, switching to 38400");
  //       myGNSS.setSerialRate(38400);
  //       delay(100);
  //   } else {
  //       // myGNSS.factoryReset();
  //       delay(2000); // Wait a bit before trying again to limit the Serial output
  //   }
  // } while(1);
  // Serial.println("GNSS serial connected");
  //
  // myGNSS.setUART1Output(COM_TYPE_UBX); // Set the UART port to output UBX only
  // myGNSS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
  // myGNSS.saveConfiguration(); // Save the current settings to flash and BBR
}

void loop()
{
	servo.write(120);
	delay(1000);
	servo.write(150);
	delay(1000);
	servo.write(120);
	delay(1000);
	servo.write(90);
	delay(1000);

  // // Query module only every second. Doing it more often will just cause I2C traffic.
  // // The module only responds when a new position is available
  // if (millis() - lastTime > 1000)
  // {
  //   lastTime = millis(); // Update the timer
  //
  //   long latitude = myGNSS.getLatitude();
  //   Serial.print(F("Lat: "));
  //   Serial.print(latitude);
  //
  //   long longitude = myGNSS.getLongitude();
  //   Serial.print(F(" Long: "));
  //   Serial.print(longitude);
  //   Serial.print(F(" (degrees * 10^-7)"));
  //
  //   long altitude = myGNSS.getAltitude();
  //   Serial.print(F(" Alt: "));
  //   Serial.print(altitude);
  //   Serial.print(F(" (mm)"));
  //
  //   byte SIV = myGNSS.getSIV();
  //   Serial.print(F(" SIV: "));
  //   Serial.print(SIV);
  //
  //   Serial.println();
  // }
}
