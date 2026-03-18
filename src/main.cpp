#include <Arduino.h>

#include "accelerometer.h"
#include "kalman_filter.h"

f32 dt;

void setup() {
	Serial.begin(115200);

	mpu_setup();
	dt = 1.0f / mpu_sample_rate;

	acc_meas *acc = get_mpu_data();

	MAT(init_state, state_dim, 1);
	mat_clear(&init_state);
	init_state.data[a_x] = (f32)acc->ax;
	init_state.data[a_y] = (f32)acc->ay;
	init_state.data[a_z] = (f32)acc->az;
	init_state.data[roll] = (f32)acc->gx;
	init_state.data[pitch] = (f32)acc->gy;
	init_state.data[yaw] = (f32)acc->gz;

	kalman_filter_init(&init_state, 1.0f);
	kalman_filter_debug_print_csv_header();
}

void loop() {
	acc_meas *acc = get_mpu_data();

	matrix *z = kalman_filter_get_z();
	z->data[me_acc_x] = (f32)acc->ax;
	z->data[me_acc_y] = (f32)acc->ay;
	z->data[me_acc_z] = (f32)acc->az;
	z->data[me_roll] = (f32)acc->gx;
	z->data[me_pitch] = (f32)acc->gy;
	z->data[me_yaw] = (f32)acc->gz;

	const matrix *s = kalman_filter_update(dt);
	kalman_filter_debug_print_csv_row(millis(), acc->ax, acc->ay, acc->az, acc->gx, acc->gy, acc->gz, s);
}
