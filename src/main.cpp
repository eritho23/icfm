#include <Arduino.h>

#include "base.h"

#include "imu.h"
#include "kalman_filter.h"
#include "controller.h"
#include "servos.h"

#define I2C_SDA 0
#define I2C_SCL 1

f32 dt;
constexpr uint32_t log_period_ms = 50;

static imu_m imu_measurement;

static controller ctrl;

void setup() {
	Serial.begin(115200);
	delay(500);
	Wire.begin(I2C_SDA, I2C_SCL);
	Wire.setClock(100000);

	servos_setup();

	// Serial.println("I2C scan start");
	// u8 found = 0;
	// for (u8 addr = 1; addr < 127; addr++) {
	// 	Wire.beginTransmission(addr);
	// 	u8 err = Wire.endTransmission(true);
	// 	if (err == 0) {
	// 		Serial.print("I2C device at 0x");
	// 		if (addr < 16) {
	// 			Serial.print('0');
	// 		}
	// 		Serial.println(addr, HEX);
	// 		found++;
	// 	}
	// 	delay(2);
	// }
	// Serial.print("I2C scan done, devices found: ");
	// Serial.println(found);

	if (!imu_setup(nullptr)) {
		Serial.println("MPU setup failed");
		while (true) {
			delay(1000);
		}
	}

	dt = 1.0f / imu_sample_rate_hz;

	while (!get_imu_data(&imu_measurement)) {
		delay(1);
	}

	// Derive initial quaternion from gravity vector
	f32 ax_b = imu_measurement.ax;
	f32 ay_b = imu_measurement.ay;
	f32 az_b = imu_measurement.az;

	f32 roll0  = atan2f(ay_b, az_b);
	f32 pitch0 = atan2f(-ax_b, sqrtf(ay_b*ay_b + az_b*az_b));

    quat init_q;
    init_q.w = cosf(roll0/2.0f)*cosf(pitch0/2.0f);
    init_q.x = sinf(roll0/2.0f)*cosf(pitch0/2.0f);
    init_q.y = cosf(roll0/2.0f)*sinf(pitch0/2.0f);
    init_q.z = -sinf(roll0/2.0f)*sinf(pitch0/2.0f);
 
    // Initial state — position and velocity zero (launch site origin)
    // Acceleration seeded from first IMU reading rotated to world frame
    MAT(init_state, state_dim, 1);
    mat_clear(&init_state);
    // pos, vel left at zero
    // acc: rotate body-frame accel to world frame using init_q
	f32 a_body[3] = {imu_measurement.ax, imu_measurement.ay, imu_measurement.az};
    f32 a_world[3];
    quat_rotate(&init_q, a_body, a_world);
    init_state.data[a_x] = a_world[0];
    init_state.data[a_y] = a_world[1];
    init_state.data[a_z] = a_world[2];
    // delta_theta left at zero
 
    kalman_filter_init(&init_state, init_q, 1.0f);
    kalman_filter_debug_print_csv_header();

	// PID tuning (example starting point)
	ctrl.pid_roll = (pid){ .kp=0.8f, .ki=0.0f, .kd=0.08f };
	ctrl.pid_pitch = (pid){ .kp=0.8f, .ki=0.0f, .kd=0.08f };
	ctrl.pid_yaw = (pid){ .kp=0.4f, .ki=0.0f, .kd=0.04f };

	// Desired attitude: straight up (world/body aligned).
	ctrl.q_desired = (quat){1.0f, 0.0f, 0.0f, 0.0f};
	quat_normalise(&ctrl.q_desired);
}

void loop() {
	static u32 last_log_ms = 0;
	static f32 last_fin_angle_deg[4] = {0.0f, 0.0f, 0.0f, 0.0f};

	if (!get_imu_data(&imu_measurement)) {
		return;
	}

	kalman_filter_rotate_accel(imu_measurement.ax, imu_measurement.ay, imu_measurement.az);
	const matrix *s = kalman_filter_update(dt, imu_measurement.wx, imu_measurement.wy, imu_measurement.wz);
	const quat *q = kalman_filter_get_quat();
	controller_update(&ctrl, dt, (quat*)q, (matrix*)s);

	bool fins_changed = false;
	for (int i = 0; i < 4; i++) {
		if (ctrl.fin_angle_deg[i] != last_fin_angle_deg[i]) {
			fins_changed = true;
			last_fin_angle_deg[i] = ctrl.fin_angle_deg[i];
		}
	}
	if (fins_changed) {
		servos_write(ctrl.fin_angle_deg);
	}

	const u32 now_ms = millis();
	if (now_ms - last_log_ms >= log_period_ms) {
		last_log_ms = now_ms;
		kalman_filter_debug_print_csv_row(now_ms, imu_measurement.wx, imu_measurement.wy, imu_measurement.wz, s);
		controller_debug_print_csv_row(now_ms, &ctrl);
	}
}
