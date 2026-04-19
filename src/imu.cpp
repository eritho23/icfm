#include "imu.h"

static Adafruit_ISM330DHCX imu;

f32 imu_sample_rate_hz;
static f32 gyro_bias[3] = {0.0f, 0.0f, 0.0f};

static inline void map_sensor_to_body(f32 sx, f32 sy, f32 sz, f32 *bx, f32 *by,
                                      f32 *bz) {
    *bx = sx;
    *by = sy;
    *bz = sz;
}

b32 imu_init(void) {
    ble_send("Initializing IMU...");
    if (!imu.begin_I2C()) {
        ble_send("IMU connection failed");
        return false;
    }
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS); // NOTE: With this we allow
                                                 // ~1,4 rotations/second at
                                                 // max before clipping
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
    imu_sample_rate_hz = 104.0f;

    delay(500); // settle

    // Discard first N samples
    for (int i = 0; i < 100; i++) {
        sensors_event_t a, g, t;
        imu.getEvent(&a, &g, &t);
        delay(10);
    }

    ble_send("IMU init success");

    return true;
}

b32 imu_calibrate(void) {
    ble_send("Calibrating gyro bias...");
    const int samples = 1000;
    f32 gx_sum = 0, gy_sum = 0, gz_sum = 0;

    for (int i = 0; i < samples; i++) {
        sensors_event_t a, g, t;
        imu.getEvent(&a, &g, &t);
        gx_sum += g.gyro.x;
        gy_sum += g.gyro.y;
        gz_sum += g.gyro.z;
        delay(10);
    }

    gyro_bias[0] = gx_sum / samples;
    gyro_bias[1] = gy_sum / samples;
    gyro_bias[2] = gz_sum / samples;

    ble_sendf("Gyro bias (rad/s)  x: %.6f  y: %.6f  z: %.6f", gyro_bias[0],
              gyro_bias[1], gyro_bias[2]);
    ble_send("IMU calibrated.");
    return true;
}

b32 get_imu_data(imu_measurement_t *out) {
    sensors_event_t accel_ev, gyro_ev, temp_ev;
    imu.getEvent(&accel_ev, &gyro_ev, &temp_ev);

    const f32 ax_i = accel_ev.acceleration.x;
    const f32 ay_i = accel_ev.acceleration.y;
    const f32 az_i = accel_ev.acceleration.z;
    const f32 wx_i = gyro_ev.gyro.x - gyro_bias[0];
    const f32 wy_i = gyro_ev.gyro.y - gyro_bias[1];
    const f32 wz_i = gyro_ev.gyro.z - gyro_bias[2];

    map_sensor_to_body(ax_i, ay_i, az_i, &out->ax, &out->ay, &out->az);
    map_sensor_to_body(wx_i, wy_i, wz_i, &out->wx, &out->wy, &out->wz);
    return true;
}

void imu_debug(void) {
    imu_measurement_t d;
    if (!get_imu_data(&d))
        return;
    ble_sendf("ax: %.4f ay: %.4f az: %.4f wx: %.4f wy: %.4f wz: %.4f", d.ax,
              d.ay, d.az, d.wx, d.wy, d.wz);
}
