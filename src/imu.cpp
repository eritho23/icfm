#include "imu.h"

MPU6050 imu;
f32 imu_sample_rate_hz;

static const f32 g = 9.819f;

static f32 accel_scale = 0.0f;
static f32 gyro_scale = 0.0f;
static f32 gyro_bias[3] = {0.0f, 0.0f, 0.0f};  // x, y, z  (rad/s)

static u32 last_sample_us = 0;

b32 imu_setup(f32 accel_avg_out[3]) {
    Serial.println("Initializing IMU...");
    imu.initialize();
    imu.setDLPFMode(3);
    imu.setRate(9);

    if (!imu.testConnection()) {
        Serial.println("IMU connection failed");
        return false;
    }
    Serial.println("IMU connection successful");

    imu.setXAccelOffset(0); imu.setYAccelOffset(0); imu.setZAccelOffset(0);
    imu.setXGyroOffset(0);  imu.setYGyroOffset(0);  imu.setZGyroOffset(0);

    accel_scale = imu.get_acce_resolution() * g;
    gyro_scale = imu.get_gyro_resolution() * (PI / 180.0f);
    imu_sample_rate_hz = 1000.0f / (1.0f + (f32)imu.getRate());

    Serial.println("Calibrating gyro bias, keep stationary...");

    const int samples = 2000;
    f32 gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
    f32 ax_sum = 0.0f, ay_sum = 0.0f, az_sum = 0.0f;

    for (int i = 0; i < samples; i++) {
        i16 ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
        imu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
        ax_sum += (f32)ax_raw * accel_scale;
        ay_sum += (f32)ay_raw * accel_scale;
        az_sum += (f32)az_raw * accel_scale;
        gx_sum += (f32)gx_raw * gyro_scale;
        gy_sum += (f32)gy_raw * gyro_scale;
        gz_sum += (f32)gz_raw * gyro_scale;
        delay(2);
    }

    gyro_bias[0] = gx_sum / samples;
    gyro_bias[1] = gy_sum / samples;
    gyro_bias[2] = gz_sum / samples;

    Serial.print("Gyro bias (rad/s)  x: "); Serial.print(gyro_bias[0], 6);
    Serial.print("  y: "); Serial.print(gyro_bias[1], 6);
    Serial.print("  z: "); Serial.println(gyro_bias[2], 6);

    if (accel_avg_out) {
        accel_avg_out[0] = ax_sum / samples;
        accel_avg_out[1] = ay_sum / samples;
        accel_avg_out[2] = az_sum / samples;
    }

    last_sample_us = micros();
    Serial.println("IMU ready.");
    return true;
}

/*
	Read one sample from the IMU.
	Returns gyro rates adjusted with bias from calibration.
*/
b32 get_imu_data(imu_m *out) {
    i16 ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    imu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    // Raw IMU-frame measurements.
    const f32 ax_i = (f32)ax_raw * accel_scale;
    const f32 ay_i = (f32)ay_raw * accel_scale;
    const f32 az_i = (f32)az_raw * accel_scale;
    const f32 wx_i = (f32)gx_raw * gyro_scale - gyro_bias[0];
    const f32 wy_i = (f32)gy_raw * gyro_scale - gyro_bias[1];
    const f32 wz_i = (f32)gz_raw * gyro_scale - gyro_bias[2];

    // IMU is mounted sideways relative to rocket body axes.
    // body = R_x(-90 deg) * imu, so:
    //   x_b =  x_i
    //   y_b =  z_i
    //   z_b = -y_i
    out->ax = ax_i;
    out->ay = az_i;
    out->az = -ay_i;
    out->wx = wx_i;
    out->wy = wz_i;
    out->wz = -wy_i;

    return true;
}

void imu_debug(void) {
    imu_m d;
    if (!get_imu_data(&d)) return;
    Serial.print("ax: "); Serial.print(d.ax, 4);
    Serial.print(" ay: "); Serial.print(d.ay, 4);
    Serial.print(" az: "); Serial.print(d.az, 4);
    Serial.print(" wx: "); Serial.print(d.wx, 4);
    Serial.print(" wy: "); Serial.print(d.wy, 4);
    Serial.print(" wz: "); Serial.println(d.wz, 4);
}
