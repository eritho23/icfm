#ifndef ICFM_CONTROLLER_H
#define ICFM_CONTROLLER_H

#include "../include/base.h"
#include "./bluetooth.h"
#include "./kalman_filter.h"
#include "./quaternion_utils.h"

#define MAX_FIN_DEFLECTION_DEG 30.0f
#define GYRO_LPF_ALPHA 0.546f

typedef struct {
    f32 kp;
    f32 ki;
    f32 kd;
    f32 integral;
    f32 integral_limit;
} pid;

typedef struct {
    pid pid_roll;
    pid pid_pitch;
    pid pid_yaw;
    quat q_desired; // desired attitude (world frame, set by guidance)
    f32 fin_angle_deg[4];
	f32 wx_f, wy_f, wz_f; // low-pass filtered gyro rates (~20 Hz cutoff at 104 Hz)
} controller_t;

quat attitude_error(const quat *q_desired, const quat *q_current);
void controller_update(controller_t *c, f32 dt, quat *q_current, matrix *state, f32 wx, f32 wy, f32 wz);
void controller_reset(controller_t *c);
void controller_debug_print_csv_row(u32 t_ms, const controller_t *c);

#endif
