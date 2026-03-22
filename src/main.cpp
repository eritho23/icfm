#include <Arduino.h>

#include "imu.h"
#include "kalman_filter.h"
#include "base.h"

#define I2C_SDA 0
#define I2C_SCL 1

f32 dt;
constexpr uint32_t log_period_ms = 50;

static imu_m imu;

void setup() {
	Serial.begin(115200);
	delay(500);
	Wire.begin(I2C_SDA, I2C_SCL);
	Wire.setClock(100000);

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

	if (!imu_setup()) {
		Serial.println("MPU setup failed");
		while (true) {
			delay(1000);
		}
	}

	dt = 1.0f / imu_sample_rate_hz;
	Serial.println("sample rate:");
	Serial.println(imu_sample_rate_hz);
	Serial.println("dt");
	Serial.println(dt);
	Serial.println("starting testing in 3s");
	delay(3000);

	while (!get_imu_data(&imu)) {
		delay(1);
	}

	// Derive initial quaternion from gravity vector
    // If rocket is vertical the initial tilt from identity will be small
    f32 roll0  = atan2f(imu.ay, imu.az);
    f32 pitch0 = atan2f(-imu.ax, sqrtf(imu.ay*imu.ay + imu.az*imu.az));
    quat init_q;
    init_q.w =  cosf(roll0/2.0f)*cosf(pitch0/2.0f);
    init_q.x =  sinf(roll0/2.0f)*cosf(pitch0/2.0f);
    init_q.y =  cosf(roll0/2.0f)*sinf(pitch0/2.0f);
    init_q.z = -sinf(roll0/2.0f)*sinf(pitch0/2.0f);
 
    // Initial state — position and velocity zero (launch site origin)
    // Acceleration seeded from first IMU reading rotated to world frame
    MAT(init_state, state_dim, 1);
    mat_clear(&init_state);
    // pos, vel left at zero
    // acc: rotate body-frame accel to world frame using init_q
    f32 a_body[3] = {imu.ax, imu.ay, imu.az};
    f32 a_world[3];
    quat_rotate(&init_q, a_body, a_world);
    init_state.data[a_x] = a_world[0];
    init_state.data[a_y] = a_world[1];
    init_state.data[a_z] = a_world[2];
    // delta_theta left at zero
 
    kalman_filter_init(&init_state, init_q, 1.0f);
    kalman_filter_debug_print_csv_header();
}

void loop() {
	static u32 last_log_ms = 0;

	if (!get_imu_data(&imu)) {
		return;
	}

	// TODO: Fill later
    // z->data[m_gps_x] = gps_x; etc.
 
    // Rotate body-frame accel to world frame and write into z
    kalman_filter_rotate_accel(imu.ax, imu.ay, imu.az);
 
    // Update filter — gyro rates passed as arguments, not via z
    const matrix *s = kalman_filter_update(dt, imu.wx, imu.wy, imu.wz);
 
    const u32 now_ms = millis();
    if (now_ms - last_log_ms >= log_period_ms) {
        last_log_ms = now_ms;
        kalman_filter_debug_print_csv_row(now_ms, imu.wx, imu.wy, imu.wz, s);
    }
}
