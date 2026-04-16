#include "./utils.h"

b32 serial_read_line(char *out, int max_len) {
  static char buf[64];
  static int idx = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      buf[idx] = '\0';
      strncpy(out, buf, (size_t)max_len - 1U);
      out[max_len - 1] = '\0';
      idx = 0;
      return true;
    }
    if (idx < (int)sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }

  return false;
}

void i2c_scan() {
  Serial.println("I2C scan start");
  u8 found = 0;
  for (u8 addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    u8 err = Wire.endTransmission(true);
    if (err == 0) {
      Serial.print("I2C device at 0x");
      if (addr < 16) {
        Serial.print('0');
      }
      Serial.println(addr, HEX);
      found++;
    }
    delay(2);
  }
  Serial.print("I2C scan done, devices found: ");
  Serial.println(found);
}
