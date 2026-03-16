#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <SoftwareSerial.h>

SFE_UBLOX_GNSS gps;

SoftwareSerial soft_serial(20, 21); // RX, TX

long last_time = 0; // Simple local timer. Limits amount of I2C traffic to u-blox module.

void gps_setup() {
	// Assume that the U-Blox GNSS is running at 9600 baud (the default) or at 38400 baud.
	// Loop until we're in sync and then ensure it's at 38400 baud.
	do {
		Serial.println("GNSS: trying 38400 baud");
		soft_serial.begin(38400);
		if (gps.begin(soft_serial) == true) break;

		delay(100);
		Serial.println("GNSS: trying 9600 baud");
		soft_serial.begin(9600);
		if (gps.begin(soft_serial) == true) {
			Serial.println("GNSS: connected at 9600 baud, switching to 38400");
			gps.setSerialRate(38400);
			delay(100);
		} else {
			// gps.factoryReset();
			delay(2000); // Wait a bit before trying again to limit the Serial output
		}
	} while(1);
	Serial.println("GNSS serial connected");

	gps.setUART1Output(COM_TYPE_UBX); // Set the UART port to output UBX only
	gps.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
	gps.saveConfiguration(); // Save the current settings to flash and BBR
}

void test_gps() {
  // Query module only every second. Doing it more often will just cause I2C traffic.
  // The module only responds when a new position is available
  if (millis() - last_time > 1000)
  {
    last_time = millis(); // Update the timer

    long latitude = gps.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = gps.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = gps.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    byte SIV = gps.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    Serial.println();
  }
}
