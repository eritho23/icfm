#include <Arduino.h>

#include "base.h"

#include "esp32-hal.h"
#include "imu.h"
#include "kalman_filter.h"
#include "controller.h"
#include "servos.h"

#define I2C_SDA 1
#define I2C_SCL 0

f32 dt;
constexpr uint32_t control_log_period_ms = 100;

static imu_m imu_measurement;

static controller ctrl;

typedef enum {
	IDLE = 0,
	CALIBRATION = 1,
	READY = 2,
	LIFTOFF = 3,
	CONTROL = 4,
	APOGEE = 5,
	TOUCHDOWN = 6
} flight_state_t;

static flight_state_t g_state = IDLE;
static flight_state_t g_prev_state = IDLE;
static u32 g_state_enter_ms = 0;

static const matrix *g_last_kf_state = NULL;
static u32 g_last_kf_update_us = 0;

static void set_fins_neutral(void) {
	f32 neutral[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	// servos_write(neutral);
}

static const char *state_name(flight_state_t s) {
	switch (s) {
		case IDLE: return "IDLE";
		case CALIBRATION: return "CALIBRATION_CHECKS";
		case READY: return "READY";
		case LIFTOFF: return "LIFTOFF";
		case CONTROL: return "CONTROL";
		case APOGEE: return "APOGEE";
		case TOUCHDOWN: return "TOUCHDOWN";
		default: return "UNKNOWN";
	}
}

static void enter_state(flight_state_t next) {
	g_prev_state = g_state;
	g_state = next;
	g_state_enter_ms = millis();

	if (next == IDLE || next == READY || next == TOUCHDOWN) {
		set_fins_neutral();
	}

	Serial.print("STATE,");
	Serial.print((int)g_prev_state);
	Serial.print(",");
	Serial.print((int)g_state);
	Serial.print(",");
	Serial.println(state_name(g_state));
}

static b32 serial_read_line(char *out, int max_len) {
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

static b32 cmd_is(const char *cmd, const char *word) {
	return strcmp(cmd, word) == 0;
}

static b32 init_kalman_from_current_imu(void) {
	if (!get_imu_data(&imu_measurement)) {
		return false;
	}

	f32 ax_b = imu_measurement.ax;
	f32 ay_b = imu_measurement.ay;
	f32 az_b = imu_measurement.az;

	quat init_q = {1.0f, 0.0f, 0.0f, 0.0f};

	MAT(init_state, state_dim, 1);
	mat_clear(&init_state);

	f32 a_body[3] = {imu_measurement.ax, imu_measurement.ay, imu_measurement.az};
	f32 a_world[3];
	quat_rotate(&init_q, a_body, a_world);
	init_state.data[a_x] = a_world[0];
	init_state.data[a_y] = a_world[1];
	init_state.data[a_z] = a_world[2];

	if (!kalman_filter_init(&init_state, init_q, 1.0f)) {
		return false;
	}

	return true;
}

void setup() {
	Serial.begin(115200);
	delay(500);
	Wire.begin(I2C_SDA, I2C_SCL);
	Wire.setClock(100000);

	// servos_setup();

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

	// PID gains
	// TODO: Tune
	ctrl.pid_roll = (pid){ .kp=0.8f, .ki=0.0f, .kd=0.08f };
	ctrl.pid_pitch = (pid){ .kp=0.8f, .ki=0.0f, .kd=0.08f };
	ctrl.pid_yaw = (pid){ .kp=0.4f, .ki=0.0f, .kd=0.04f };
	ctrl.pid_roll.integral_limit = 1.0f;
	ctrl.pid_pitch.integral_limit = 1.0f;
	ctrl.pid_yaw.integral_limit = 1.0f;

	// Desired attitude: fixed straight up (world/body aligned).
	// NOTE: This is what we want for a first launch
	ctrl.q_desired = (quat){1.0f, 0.0f, 0.0f, 0.0f};
	quat_normalise(&ctrl.q_desired);

	set_fins_neutral();
	g_state = IDLE;
	g_prev_state = IDLE;
	g_state_enter_ms = millis();

	Serial.println("STATE,0,0,IDLE");
}

void loop() {
	static u32 last_log_ms = 0;
	char cmd[64];

	if (serial_read_line(cmd, (int)sizeof(cmd))) {
		if (cmd_is(cmd, "CALIBRATE")) {
			if (g_state == IDLE || g_state == TOUCHDOWN) {
				g_last_kf_state = NULL;
				g_last_kf_update_us = 0;
				enter_state(CALIBRATION);
			} else {
				Serial.println("ERR,CALIBRATE only valid in IDLE or TOUCHDOWN");
			}
		} else if (cmd_is(cmd, "RESET")) {
			g_last_kf_state = NULL;
			g_last_kf_update_us = 0;
			enter_state(IDLE);
		} else {
			Serial.println("ERR,Unknown command");
		}
	}

	switch (g_state) {
		case IDLE:
			break;

		// FIX: Decide where to start recording flight data
		case CALIBRATION:
			if (!imu_init()) {
				Serial.println("ERR,IMU init failed");
				enter_state(IDLE);
				break;
			}
			if (!imu_calibrate()) {
				Serial.println("ERR,IMU calibration failed");
				enter_state(IDLE);
				break;
			}
			dt = 1.0f / imu_sample_rate_hz;
			if (!init_kalman_from_current_imu()) {
				Serial.println("ERR,Kalman init failed");
				enter_state(IDLE);
				break;
			}
			enter_state(READY);
			break;

		case READY:
			// NOTE: Just go to next state for now
			enter_state(LIFTOFF);

			// TODO: Detect liftoff via gps i suppose, then go to liftoff state
			break;
		case LIFTOFF:
			// NOTE: Just go to next state for now
			enter_state(CONTROL);
			// if (g_imu_ready) {
			// 	get_imu_data(&imu_measurement);
			// }
			break;

		// FIX: Dont read every loop call, use interupt pin
		case CONTROL:
			if (get_imu_data(&imu_measurement)) {
				const u32 now_us = micros();
				if (g_last_kf_update_us == 0) {
					g_last_kf_update_us = now_us;
					break;
				}

				dt = (f32)(now_us - g_last_kf_update_us) * 1e-6f;
				g_last_kf_update_us = now_us;
				if (dt <= 0.0f || dt > 0.1f) {
					dt = 1.0f / imu_sample_rate_hz;
				}

				kalman_filter_rotate_accel(imu_measurement.ax, imu_measurement.ay, imu_measurement.az);
				const matrix *s = kalman_filter_update(dt, imu_measurement.wx, imu_measurement.wy, imu_measurement.wz);
				const quat *q = kalman_filter_get_quat();
				controller_update(&ctrl, dt, (quat*)q, (matrix*)s);
				g_last_kf_state = s;

				// servos_write(ctrl.fin_angle_deg);
			}

			const u32 now_ms = millis();
			if (now_ms - last_log_ms >= control_log_period_ms) {
				last_log_ms = now_ms;

				f32 roll, pitch, yaw;
				kalman_filter_get_euler_deg(&roll, &pitch, &yaw);

				Serial.print("Roll: "); Serial.print(roll, 4);
				Serial.print(" Pitch: "); Serial.print(pitch, 4);
				Serial.print(" Yaw: "); Serial.println(yaw, 4);

				kalman_filter_debug_print_csv_row(now_ms, imu_measurement.wx, imu_measurement.wy, imu_measurement.wz, g_last_kf_state);
				controller_debug_print_csv_row(now_ms, &ctrl);
			}

			break;

		// case APOGEE:
		// 	set_fins_neutral();
		// 	break;
		//
		// case TOUCHDOWN:
		// 	set_fins_neutral();
		// 	break;
		//
		// default:
		// 	enter_state(IDLE);
		// 	break;
	}
}
