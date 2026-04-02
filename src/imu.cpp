#include "imu.h"

Adafruit_ISM330DHCX imu;

f32 imu_sample_rate_hz;
static const f32 g = 9.819f;
static f32 gyro_bias[3] = {0.0f, 0.0f, 0.0f};  // x, y, z  (rad/s)
static u32 last_sample_us = 0;

b32 imu_setup(f32 accel_avg_out[3]) {
    Serial.println("Initializing IMU...");

    if (!imu.begin_I2C()) {
        Serial.println("IMU connection failed");
        return false;
    }
    Serial.println("IMU connection successful");

    imu.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    imu.setGyroRange(ISM330DHCX_GYRO_RANGE_4000_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);

    imu_sample_rate_hz = 104.0f;

    Serial.println("Calibrating gyro bias, keep stationary...");
    const int samples = 2000;
    f32 gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
    f32 ax_sum = 0.0f, ay_sum = 0.0f, az_sum = 0.0f;

    for (int i = 0; i < samples; i++) {
        sensors_event_t accel_ev, gyro_ev, temp_ev;
        imu.getEvent(&accel_ev, &gyro_ev, &temp_ev);
        ax_sum += accel_ev.acceleration.x;
        ay_sum += accel_ev.acceleration.y;
        az_sum += accel_ev.acceleration.z;
        gx_sum += gyro_ev.gyro.x;
        gy_sum += gyro_ev.gyro.y;
        gz_sum += gyro_ev.gyro.z;
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

b32 get_imu_data(imu_m *out) {
    sensors_event_t accel_ev, gyro_ev, temp_ev;
    imu.getEvent(&accel_ev, &gyro_ev, &temp_ev);

    // Adafruit library already outputs SI units:
    // acceleration in m/s^2, gyro in rad/s
    const f32 ax_i = accel_ev.acceleration.x;
    const f32 ay_i = accel_ev.acceleration.y;
    const f32 az_i = accel_ev.acceleration.z;
    const f32 wx_i = gyro_ev.gyro.x - gyro_bias[0];
    const f32 wy_i = gyro_ev.gyro.y - gyro_bias[1];
    const f32 wz_i = gyro_ev.gyro.z - gyro_bias[2];

    // IMU is mounted sideways relative to rocket body axes.
    // body = R_x(-90 deg) * imu, so:
    out->ax = ax_i;
    out->ay = az_i;
    out->az = ay_i;
    out->wx = wy_i;   // pitch rate
    out->wy = -wz_i;  // yaw rate
    out->wz = wx_i;   // roll rate
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
