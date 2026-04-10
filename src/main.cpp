#include <Arduino.h>
#include "gps.h"

void setup() {
	Serial.begin(115200);
	delay(500);

	gps_setup();
}

void loop() {
	gps.checkUblox(); // keep GNSS data updated
	test_gps();       // print GPS data
}
