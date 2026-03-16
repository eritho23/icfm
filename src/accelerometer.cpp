#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"

MPU6050 mpu;

#define OUTPUT_READABLE_ACCELGYRO
//#define OUTPUT_BINARY_ACCELGYRO

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup_mpu() {
  Wire.begin();

  Serial.println("Initializing MPU...");
  mpu.initialize();

  Serial.println("Testing MPU6050 connection...");
  if (mpu.testConnection() == false) {
    Serial.println("MPU6050 connection failed");
    while (true);
  } else {
    Serial.println("MPU6050 connection successful");
  }

  Serial.println("Updating internal sensor offsets...\n");
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  Serial.print("\t"); Serial.print(mpu.getXAccelOffset());
  Serial.print("\t"); Serial.print(mpu.getYAccelOffset());
  Serial.print("\t"); Serial.print(mpu.getZAccelOffset());
  Serial.print("\t"); Serial.print(mpu.getXGyroOffset());
  Serial.print("\t"); Serial.print(mpu.getYGyroOffset());
  Serial.print("\t"); Serial.print(mpu.getZGyroOffset());
  Serial.print("\n");
}

void test_mpu() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

#ifdef OUTPUT_READABLE_ACCELGYRO
  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);
#endif

#ifdef OUTPUT_BINARY_ACCELGYRO
  Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
  Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
  Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
  Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
  Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
  Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
#endif
}
