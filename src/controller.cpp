#include "controller.h"

quat attitude_error(const quat *q_desired, const quat *q_current) {
    quat q_conj, q_err;

    quat_conjugate(&q_conj, q_current);
    quat_mul(&q_err, q_desired, &q_conj);
    quat_normalise(&q_err);

    // Ensure shortest path rotation
    if (q_err.w < 0.0f) {
        q_err.w = -q_err.w;
        q_err.x = -q_err.x;
        q_err.y = -q_err.y;
        q_err.z = -q_err.z;
    }

    return q_err;
}

f32 pid_update(pid *p, f32 error, f32 rate, f32 dt) {
    p->integral += error * dt;

	if (p->integral > p->integral_limit) p->integral = p->integral_limit;
	else if (p->integral < -p->integral_limit) p->integral = -p->integral_limit;

    return p->Kp * error + p->Ki * p->integral + p->Kd * rate;
}

void controller_update(controller *c, f32 dt, quat *q_current, matrix *state) {
    static quat q_prev = {1.0f, 0.0f, 0.0f, 0.0f}; // previous filtered quaternion

    // Attitude error in world frame
    quat q_err = attitude_error(&c->q_desired, q_current);
    f32 err_roll  = q_err.x;
    f32 err_pitch = q_err.y;
    f32 err_yaw   = q_err.z;

    // Compute filtered body-frame angular rates from quaternion change
    quat dq;
    quat q_prev_conj;
    quat_conjugate(&q_prev_conj, &q_prev);
    quat_mul(&dq, q_current, &q_prev_conj);

    // Small-angle approximation for derivative
    f32 wx_b = 2.0f * dq.x / dt;
    f32 wy_b = 2.0f * dq.y / dt;
    f32 wz_b = 2.0f * dq.z / dt;

    // Update previous quaternion for next loop
    q_prev = *q_current;

    // PID, roll stays in body frame, pitch/yaw in world frame
    f32 u_roll = pid_update(&c->pid_roll, err_roll, wx_b, dt);
    f32 u_pitch = pid_update(&c->pid_pitch, err_pitch, wy_b, dt);
    f32 u_yaw = pid_update(&c->pid_yaw, err_yaw, wz_b, dt);

    // Rotate pitch/yaw demands from world → body frame
    f32 u_pitchyaw_world[3] = { 0.0f, u_pitch, u_yaw };
    f32 u_pitchyaw_body[3];
    quat_rotate_inv(q_current, u_pitchyaw_world, u_pitchyaw_body);
    f32 u_pitch_body = u_pitchyaw_body[1];
    f32 u_yaw_body = u_pitchyaw_body[2];

    // Mix into fin deflections
    f32 delta[4];
    delta[0] =  u_roll + u_pitch_body;
    delta[1] =  u_roll - u_pitch_body;
    delta[2] = -u_roll + u_yaw_body;
    delta[3] = -u_roll - u_yaw_body;

    // Scale and clamp to servo angle (degrees)
    for (int i = 0; i < 4; i++) {
        c->fin_angle_deg[i] = delta[i] * MAX_FIN_DEFLECTION_DEG;
        if (c->fin_angle_deg[i] >  MAX_FIN_DEFLECTION_DEG) c->fin_angle_deg[i] =  MAX_FIN_DEFLECTION_DEG;
        else if (c->fin_angle_deg[i] < -MAX_FIN_DEFLECTION_DEG) c->fin_angle_deg[i] = -MAX_FIN_DEFLECTION_DEG;
    }
}

void controller_debug_print_csv_row(u32 t_ms, const controller *c) {
    Serial.print(t_ms); Serial.print(',');

    // Desired attitude
    Serial.print(c->q_desired.w, 6); Serial.print(',');
    Serial.print(c->q_desired.x, 6); Serial.print(',');
    Serial.print(c->q_desired.y, 6); Serial.print(',');
    Serial.print(c->q_desired.z, 6); Serial.print(',');

    // Fin angles
    for (int i = 0; i < 4; i++) {
        Serial.print(c->fin_angle_deg[i], 6);
        if (i < 3) Serial.print(',');
    }

    Serial.println();
}
