#include <Arduino.h>

#include "accelerometer.h"
#include "kalman_filter.h"
#include "base.h"

f32 dt;

#define I2C_SDA 0
#define I2C_SCL 1

constexpr uint32_t log_period = 50;

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

	if (!mpu_setup()) {
		Serial.println("MPU setup failed");
		while (true) {
			delay(1000);
		}
	}

	dt = 1.0f / mpu_sample_rate_hz;
	Serial.println("sample rate:");
	Serial.println(mpu_sample_rate_hz);
	Serial.println("dt");
	Serial.println(dt);
	Serial.println("starting testing in 3s");
	delay(3000);

	acc_meas acc;
	while (!get_mpu_data(&acc)) {
		delay(1);
	}

	MAT(init_state, state_dim, 1);
	mat_clear(&init_state);
	init_state.data[a_x] = acc.ax;
	init_state.data[a_y] = acc.ay;
	init_state.data[a_z] = acc.az;
	init_state.data[roll] = acc.roll;
	init_state.data[pitch] = acc.pitch;
	init_state.data[yaw] = acc.yaw;

	kalman_filter_init(&init_state, 1.0f);
	kalman_filter_debug_print_csv_header();
}

void loop() {
	static u32 last_log_ms = 0;

	acc_meas acc;
	if (!get_mpu_data(&acc)) {
		return;
	}

	matrix *z = kalman_filter_get_z();
	z->data[me_acc_x] = acc.ax;
	z->data[me_acc_y] = acc.ay;
	z->data[me_acc_z] = acc.az;
	z->data[me_roll] = acc.roll;
	z->data[me_pitch] = acc.pitch;
	z->data[me_yaw] = acc.yaw;

	const matrix *s = kalman_filter_update(dt);

	const u32 now_ms = millis();
	if (now_ms - last_log_ms >= log_period ) {
		last_log_ms = now_ms;
		kalman_filter_debug_print_csv_row(now_ms, acc.ax, acc.ay, acc.az, acc.roll, acc.pitch, acc.yaw, s);
	}
}
