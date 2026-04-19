#include "controller.h"

f32 pid_update(pid *p, f32 error, f32 rate, f32 dt) {
    p->integral += error * dt;

    if (p->integral > p->integral_limit)
        p->integral = p->integral_limit;
    else if (p->integral < -p->integral_limit)
        p->integral = -p->integral_limit;

    // With zero desired angular rate, derivative term should damp measured
    // rate.
    return p->kp * error + p->ki * p->integral - p->kd * rate;
}

void controller_update(controller_t *c, f32 dt, quat *q_current,
                       matrix *state, f32 wx, f32 wy, f32 wz) {
    // TODO: Actually use the state to scale with velocity for example.
    (void)state;

    // Normalize current attitude.
    quat q_curr = *q_current;
    quat_normalise(&q_curr);

    // Body-frame error: rotation the body needs to apply to reach desired.
    // q_err = q_current^-1 * q_desired
    quat q_curr_conj, q_err;
    quat_conjugate(&q_curr_conj, &q_curr);
    quat_mul(&q_err, &q_curr_conj, &c->q_desired);
    quat_normalise(&q_err);

    // Shortest path
    if (q_err.w < 0.0f) {
        q_err.w = -q_err.w;
        q_err.x = -q_err.x;
        q_err.y = -q_err.y;
        q_err.z = -q_err.z;
    }

    // Small-angle approximation: error in radians.
    f32 err_roll  = 2.0f * q_err.x;
    f32 err_pitch = 2.0f * q_err.y;
    f32 err_yaw   = 2.0f * q_err.z;

    // wx/wy/wz are body-frame gyro rates
    f32 u_roll  = pid_update(&c->pid_roll,  err_roll,  wx, dt);
    f32 u_pitch = pid_update(&c->pid_pitch, err_pitch, wy, dt);
    f32 u_yaw   = pid_update(&c->pid_yaw,   err_yaw,   wz, dt);

    // Mixer
    // Pitch pair: fins 0 and 2 (opposite)
    // Yaw pair: fins 1 and 3 (opposite)
    f32 delta[4] = {
         u_pitch + u_roll,  // fin0
         u_yaw   + u_roll,  // fin1
        -u_pitch + u_roll,  // fin2
        -u_yaw   + u_roll,  // fin3
    };

    // Clamp to [-1, 1]
    f32 max_abs = 0.0f;
    for (int i = 0; i < 4; i++) {
        f32 a = fabsf(delta[i]);
        if (a > max_abs) max_abs = a;
    }

    if (max_abs > 1.0f) {
        for (int i = 0; i < 4; i++) delta[i] /= max_abs;
    }

	// Scale to fin angles
    for (int i = 0; i < 4; i++) {
        c->fin_angle_deg[i] = delta[i] * MAX_FIN_DEFLECTION_DEG;
    }
}

void controller_reset(controller_t *c) {
    if (!c) return;

    c->pid_roll.integral = 0.0f;
    c->pid_pitch.integral = 0.0f;
    c->pid_yaw.integral = 0.0f;

    for (int i = 0; i < 4; i++) {
        c->fin_angle_deg[i] = 0.0f;
    }
}

void controller_debug_print_csv_row(u32 t_ms, const controller_t *c) {
    if (!c) {
        return;
    }
    ble_sendf("%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
              (unsigned long)t_ms, c->q_desired.w, c->q_desired.x,
              c->q_desired.y, c->q_desired.z, c->fin_angle_deg[0],
              c->fin_angle_deg[1], c->fin_angle_deg[2], c->fin_angle_deg[3]);
}
