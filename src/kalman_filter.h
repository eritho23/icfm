#ifndef ICFM_KALMAN_FILTER_H
#define ICFM_KALMAN_FILTER_H

#include <Arduino.h>

#include "../include/base.h"
#include "./bluetooth.h"
#include "./matrix_utils.h"
#include "./quaternion_utils.h"

/*
   These enums help us identity which index is what in the matrices
   since the buffer is row-major (meaning just a 1d array).
*/
typedef enum {
  p_x = 0,
  p_y,
  p_z,
  v_x,
  v_y,
  v_z,
  a_x,
  a_y,
  a_z,
  d_theta,
  d_alpha,
  d_beta,
  state_dim // = 12
} state_index;

typedef enum {
  m_gps_x = 0,
  m_gps_y,
  m_gps_z,
  m_acc_x,
  m_acc_y,
  m_acc_z,
  m_dim // = 6
} measurement_index;

b32 kalman_filter_init(const matrix *init_state, quat init_q,
                       f32 init_variance);
matrix *kalman_filter_update(f32 dt, f32 wx, f32 wy, f32 wz);
void kalman_filter_rotate_accel(f32 ax_b, f32 ay_b, f32 az_b);
void kalman_filter_set_gps_enu(f32 x_east, f32 y_north, f32 z_up);

matrix *kalman_filter_get_z(void);
const quat *kalman_filter_get_quat(void);
void kalman_filter_get_euler_deg(f32 *roll, f32 *pitch, f32 *yaw);

void kalman_filter_debug_print_csv_header(void);
void kalman_filter_debug_print_csv_row(u32 t_ms, f32 wx, f32 wy, f32 wz,
                                       const matrix *state);

#endif
