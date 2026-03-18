#include "accelerometer.h"

MPU6050 mpu;
f32 mpu_sample_rate_hz;

static const f32 g = 9.819f;  // m/s^2
static const f32 filter_alpha = 0.98f;

static f32 accel_scale = 0.0f;
static f32 gyro_scale = 0.0f;

static f32 gyro_bias_x = 0.0f;
static f32 gyro_bias_y = 0.0f;
static f32 gyro_bias_z = 0.0f;

static f32 roll_est = 0.0f;
static f32 pitch_est = 0.0f;
static f32 yaw_est = 0.0f;
static u32 last_sample_us = 0;

b32 mpu_setup(void) {
	Serial.println("Initializing MPU...");
	mpu.initialize();
	mpu.setDLPFMode(3);
	mpu.setRate(9);

	if (!mpu.testConnection()) {
		Serial.println("MPU6050 connection failed");
		return false;
	}
	Serial.println("MPU6050 connection successful");

	mpu.setXAccelOffset(0); mpu.setYAccelOffset(0); mpu.setZAccelOffset(0);
	mpu.setXGyroOffset(0); mpu.setYGyroOffset(0); mpu.setZGyroOffset(0);

	// Precompute scale factors
	accel_scale = mpu.get_acce_resolution() * g;
	gyro_scale = mpu.get_gyro_resolution() * (PI / 180.0f);
	mpu_sample_rate_hz = 1000.0f / (1.0f + mpu.getRate());

	Serial.println("Calibrating gyro bias (keep still)...");
	const int samples = 500;
	f32 gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
	f32 ax_sum = 0.0f, ay_sum = 0.0f, az_sum = 0.0f;
	for (int i = 0; i < samples; i++) {
		i16 ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
		mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
		ax_sum += (f32)ax_raw * accel_scale;
		ay_sum += (f32)ay_raw * accel_scale;
		az_sum += (f32)az_raw * accel_scale;
		gx_sum += (f32)gx_raw * gyro_scale;
		gy_sum += (f32)gy_raw * gyro_scale;
		gz_sum += (f32)gz_raw * gyro_scale;
		delay(2);
	}

	gyro_bias_x = gx_sum / samples;
	gyro_bias_y = gy_sum / samples;
	gyro_bias_z = gz_sum / samples;

	// Init attitude from averaged accel (used once at startup only)
	const f32 ax_avg = ax_sum / samples;
	const f32 ay_avg = ay_sum / samples;
	const f32 az_avg = az_sum / samples;
	roll_est = atan2f(ay_avg, az_avg);
	pitch_est = atan2f(-ax_avg, sqrtf(ay_avg*ay_avg + az_avg*az_avg));
	yaw_est = 0.0f;

	last_sample_us = micros();
	Serial.println("MPU ready.");
	return true;
}

b32 get_mpu_data(acc_meas *out) {
	i16 ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
	mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

	const u32 now_us = micros();
	f32 dt = (f32)(now_us - last_sample_us) * 1.0e-6f;
	if (dt <= 0.0f || dt > 0.1f) dt = 1.0f / mpu_sample_rate_hz;
	last_sample_us = now_us;

	// Accel in m/s²
	const f32 ax = (f32)ax_raw * accel_scale;
	const f32 ay = (f32)ay_raw * accel_scale;
	const f32 az = (f32)az_raw * accel_scale;

	// Gyro in rad/s, bias corrected
	const f32 gx = (f32)gx_raw * gyro_scale - gyro_bias_x;
	const f32 gy = (f32)gy_raw * gyro_scale - gyro_bias_y;
	const f32 gz = (f32)gz_raw * gyro_scale - gyro_bias_z;

	// Accel-derived roll/pitch — noisy but drift-free, corrects gyro long-term
	const f32 roll_acc  = atan2f(ay, az);
	const f32 pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az));

	// Complementary filter — gyro dominates short-term, accel corrects drift on roll/pitch
	// Yaw: pure gyro integration, will drift slowly without magnetometer or GPS
	roll_est = filter_alpha * (roll_est  + gx * dt) + (1.0f - filter_alpha) * roll_acc;
	pitch_est = filter_alpha * (pitch_est + gy * dt) + (1.0f - filter_alpha) * pitch_acc;
	yaw_est += gz * dt;

	out->ax = ax;
	out->ay = ay;
	out->az = az;
	out->roll = roll_est;
	out->pitch = pitch_est;
	out->yaw = yaw_est;

	return true;
}

void mpu_debug(void) {
	acc_meas data;
	if (!get_mpu_data(&data)) return;
	Serial.print("roll: "); Serial.print(data.roll, 4);
	Serial.print(" pitch: "); Serial.print(data.pitch, 4);
	Serial.print(" yaw: "); Serial.print(data.yaw, 4);
	Serial.print(" ax: "); Serial.print(data.ax, 4);
	Serial.print(" ay: "); Serial.print(data.ay, 4);
	Serial.print(" az: "); Serial.println(data.az, 4);
}
