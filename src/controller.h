#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "base.h"
#include "quaternion_utils.h"
#include "kalman_filter.h"

#define MAX_FIN_DEFLECTION_DEG 5.0f

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
} controller;

quat attitude_error(const quat *q_desired, const quat *q_current);
void controller_update(controller *c, f32 dt, quat *q_current, matrix *state);
void controller_debug_print_csv_row(u32 t_ms, const controller *c);

#endif
