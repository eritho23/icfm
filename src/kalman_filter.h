#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "base.h"
#include "matrix_utils.h"

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

extern matrix x_posterior;

b32 kalman_filter_init(matrix *init_state, f32 init_variance);
matrix* kalman_filter_update(const matrix *measurement, f32 dt);
void set_measurement(const matrix *measurement);

#endif
