#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Arduino.h>

#include "base.h"
#include "matrix_utils.h"


/*
   These enums help us identity which index is what in the matrices
   since the buffer is row-major.
   */
typedef enum {
	p_x = 0, p_y, p_z,
	v_x, v_y, v_z,
	a_x, a_y, a_z,
	roll, pitch, yaw,
	state_dim // = 12
} state_index;

typedef enum {
	me_gps_x = 0, me_gps_y, me_gps_z,
	me_acc_x, me_acc_y, me_acc_z,
	me_roll, me_pitch, me_yaw,
	me_dim // = 9
} me_index;

b32 kalman_filter_init(const matrix *init_state, f32 init_variance);
matrix* kalman_filter_update(f32 dt);
matrix* kalman_filter_get_z(void);

void kalman_filter_debug_print_csv_header(void);
void kalman_filter_debug_print_csv_row(u32 t_ms, f32 ax, f32 ay, f32 az, f32 roll_meas, f32 pitch_meas, f32 yaw_meas, const matrix *state);

#endif
