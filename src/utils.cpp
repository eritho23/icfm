#include "./utils.h"

void i2c_scan() {
  ble_send("I2C scan start");
  u8 found = 0;
  for (u8 addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    u8 err = Wire.endTransmission(true);
    if (err == 0) {
      ble_sendf("I2C device at 0x%02X", addr);
      found++;
    }
    delay(2);
  }
  ble_sendf("I2C scan done, devices found: %u", found);
}
