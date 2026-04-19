#include "controller.h"

#include "bluetooth.h"

static quat g_q_prev = {1, 0, 0, 0};
static b32 g_q_prev_valid = false;

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

  if (p->integral > p->integral_limit)
    p->integral = p->integral_limit;
  else if (p->integral < -p->integral_limit)
    p->integral = -p->integral_limit;

  // With zero desired angular rate, derivative term should damp measured rate.
  return p->kp * error + p->ki * p->integral - p->kd * rate;
}

void controller_update(controller_t *c, f32 dt, quat *q_current,
                       matrix *state) {
  // TODO: Actually use the state to scale with velocity for example
  (void)state;

  // Normalize current attitude
  quat q_curr = *q_current;
  quat_normalise(&q_curr);

  if (!g_q_prev_valid) {
    g_q_prev = q_curr;
    g_q_prev_valid = true;
  }

  // Keep quaternion sign continuous
  f32 dot = q_curr.w * g_q_prev.w + q_curr.x * g_q_prev.x + q_curr.y * g_q_prev.y +
            q_curr.z * g_q_prev.z;
  if (dot < 0.0f) {
    q_curr.w = -q_curr.w;
    q_curr.x = -q_curr.x;
    q_curr.y = -q_curr.y;
    q_curr.z = -q_curr.z;
  }

  // Attitude error, proportional term
  quat q_err = attitude_error(&c->q_desired, &q_curr);

  // Quaternion -> Radians, small-angle approximation
  f32 err_roll = 2.0f * q_err.x;
  f32 err_pitch = 2.0f * q_err.y;
  f32 err_yaw = 2.0f * q_err.z;

  // Angular rates from quaternion delta, derivative term
  quat dq, q_prev_conj;
  quat_conjugate(&q_prev_conj, &g_q_prev);
  quat_mul(&dq, &q_curr, &q_prev_conj);
  quat_normalise(&dq);

  if (dq.w < 0.0f) {
    dq.w = -dq.w;
    dq.x = -dq.x;
    dq.y = -dq.y;
    dq.z = -dq.z;
  }

  f32 wx_b = 2.0f * dq.x / dt;
  f32 wy_b = 2.0f * dq.y / dt;
  f32 wz_b = 2.0f * dq.z / dt;

  g_q_prev = q_curr;

  // PID
  f32 u_roll = pid_update(&c->pid_roll, err_roll, wx_b, dt);
  f32 u_pitch = pid_update(&c->pid_pitch, err_pitch, wy_b, dt);
  f32 u_yaw = pid_update(&c->pid_yaw, err_yaw, wz_b, dt);

  // Rotate pitch/yaw from world -> body
  f32 u_world[3] = {0.0f, u_pitch, u_yaw};
  f32 u_body[3];
  quat_rotate_inv(&q_curr, u_world, u_body);

  f32 u_pitch_b = u_body[1];
  f32 u_yaw_b = u_body[2];

  // Mixer
  // Pitch pair: fins 0 and 2 (opposite)
  // Yaw pair: fins 1 and 3 (opposite)
  f32 delta[4] = {
      u_pitch_b + u_roll,  // fin0
      u_yaw_b + u_roll,    // fin1
      -u_pitch_b + u_roll, // fin2 - opposite to fin0
      -u_yaw_b + u_roll,   // fin3 - opposite to fin1
  };

  // Normalize, i.e. keep commands within -1 to 1
  f32 max_abs = 0.0f;
  for (int i = 0; i < 4; i++) {
    f32 a = fabsf(delta[i]);
    if (a > max_abs)
      max_abs = a;
  }

  if (max_abs > 1.0f) {
    for (int i = 0; i < 4; i++) {
      delta[i] /= max_abs;
    }
  }

  // Scale to fin angles
  for (int i = 0; i < 4; i++) {
    c->fin_angle_deg[i] = delta[i] * MAX_FIN_DEFLECTION_DEG;
  }
}

void controller_reset(controller_t *c) {
  if (!c) {
    return;
  }

  c->pid_roll.integral = 0.0f;
  c->pid_pitch.integral = 0.0f;
  c->pid_yaw.integral = 0.0f;

  for (int i = 0; i < 4; i++) {
    c->fin_angle_deg[i] = 0.0f;
  }

  g_q_prev = (quat){1.0f, 0.0f, 0.0f, 0.0f};
  g_q_prev_valid = false;
}

void controller_debug_print_csv_row(u32 t_ms, const controller_t *c) {
  if (!c) {
    return;
  }
  ble_sendf("%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", (unsigned long)t_ms,
            c->q_desired.w, c->q_desired.x, c->q_desired.y, c->q_desired.z,
            c->fin_angle_deg[0], c->fin_angle_deg[1], c->fin_angle_deg[2],
            c->fin_angle_deg[3]);
}
