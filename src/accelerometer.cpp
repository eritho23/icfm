#include "accelerometer.h"

#define OUTPUT_READABLE_ACCELGYRO
//#define OUTPUT_BINARY_ACCELGYRO

MPU6050 mpu;
acc_meas mpu_data; // global store for acc data
f32 mpu_sample_rate;

b32 mpu_setup(void) {
	Wire.begin();

	Serial.println("Initializing MPU...");
	mpu.initialize();

	Serial.println("Testing MPU6050 connection...");
	if (mpu.testConnection() == false) {
		Serial.println("MPU6050 connection failed");
		return false;
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

	// Read + set sample rate
	mpu_sample_rate = mpu.getRate();

	return true;
}

acc_meas* get_acc_data(void) {
	mpu.getMotion6(&(acc_data.ax), &(acc_data.ay), &(acc_data.az), &(acc_data.gx), &(acc_data.gy), &(acc_data.gz));
	return &acc_data;
}

void mpu_debug(void) {
	mpu.getMotion6(&(acc_data.ax), &(acc_data.ay), &(acc_data.az), &(acc_data.gx), &(acc_data.gy), &(acc_data.gz));

#ifdef OUTPUT_READABLE_ACCELGYRO
	Serial.print("a/g:\t");
	Serial.print(acc_data.ax); Serial.print("\t");
	Serial.print(acc_data.ay); Serial.print("\t");
	Serial.print(acc_data.az); Serial.print("\t");
	Serial.print(acc_data.gx); Serial.print("\t");
	Serial.print(acc_data.gy); Serial.print("\t");
	Serial.println(acc_data.gz);
#endif

#ifdef OUTPUT_BINARY_ACCELGYRO
	Serial.write((uint8_t)(acc_data.ax >> 8)); Serial.write((uint8_t)(acc_data.ax & 0xFF));
	Serial.write((uint8_t)(acc_data.ay >> 8)); Serial.write((uint8_t)(acc_data.ay & 0xFF));
	Serial.write((uint8_t)(acc_data.az >> 8)); Serial.write((uint8_t)(acc_data.az & 0xFF));
	Serial.write((uint8_t)(acc_data.gx >> 8)); Serial.write((uint8_t)(acc_data.gx & 0xFF));
	Serial.write((uint8_t)(acc_data.gy >> 8)); Serial.write((uint8_t)(acc_data.gy & 0xFF));
	Serial.write((uint8_t)(acc_data.gz >> 8)); Serial.write((uint8_t)(acc_data.gz & 0xFF));
#endif
}
