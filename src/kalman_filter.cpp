#include "kalman_filter.h"

static const f32 Q_diag[state_dim] = {
    0.01f, 0.01f, 0.01f, // pos
    0.1f,  0.1f,  0.1f,  // vel
    0.3f,  0.3f,  0.3f,  // acc, thrust/drag are highly unpredictable
    1e-3f, 1e-3f, 1e-3f, // delta_theta,
};

static const f32 R_diag[m_dim] = {
    1.5f, 1.5f, 4.0f, // gps (metres)
    0.1f, 0.1f, 0.1f, // acc (m/s^2)
};

typedef struct {
  // estimated state
  MAT(x_prior, state_dim, 1);
  MAT(x_posterior, state_dim, 1);
  MAT(p_prior, state_dim, state_dim);
  MAT(p_posterior, state_dim, state_dim);

  // measurement vector (populated before each update)
  MAT(z, m_dim, 1);

  // constant matrices (built once in init)
  MAT(F, state_dim, state_dim); // state-transition Jacobian
  MAT(H, m_dim, state_dim);     // observation model
  MAT(Q, state_dim, state_dim); // process noise
  MAT(R, m_dim, m_dim);         // measurement noise
  MAT(I, state_dim, state_dim); // identity

  // intermediates (pre-allocated, reused every cycle)
  MAT(K, state_dim, m_dim);       // Kalman gain
  MAT(FP, state_dim, state_dim);  // F * P
  MAT(PHt, state_dim, m_dim);     // P * H^T
  MAT(S, m_dim, m_dim);           // innovation covariance
  MAT(IKH, state_dim, state_dim); // I - K*H
  MAT(innovation, m_dim, 1);
  MAT(Ky, state_dim, 1);

  // attitude reference quaternion
  quat q_ref;

  b32 gps_fresh;

} kalman_state;

static kalman_state kf;

static inline void set_F(state_index r, state_index c, f32 v) {
  kf.F.data[r * state_dim + c] = v;
}

/*
        State transition Jacobian F (12x12).

        p_{k+1} = p_k + v_k*dt + a_k*0.5*dt^2
        v_{k+1} = v_k + a_k*dt
        a_{k+1} = a_k  (random walk - Q_acc drives it)
        delta_theta_{k+1} = (I - [omega x]*dt) * delta_theta_k
*/
static void _build_F(f32 dt, f32 wx, f32 wy, f32 wz) {
  mat_clear(&kf.F);
  const f32 d2 = 0.5f * dt * dt;

  set_F(p_x, p_x, 1);
  set_F(p_x, v_x, dt);
  set_F(p_x, a_x, d2);
  set_F(p_y, p_y, 1);
  set_F(p_y, v_y, dt);
  set_F(p_y, a_y, d2);
  set_F(p_z, p_z, 1);
  set_F(p_z, v_z, dt);
  set_F(p_z, a_z, d2);
  set_F(v_x, v_x, 1);
  set_F(v_x, a_x, dt);
  set_F(v_y, v_y, 1);
  set_F(v_y, a_y, dt);
  set_F(v_z, v_z, 1);
  set_F(v_z, a_z, dt);
  set_F(a_x, a_x, 1);
  set_F(a_y, a_y, 1);
  set_F(a_z, a_z, 1);
  set_F(d_theta, d_theta, 1);
  set_F(d_theta, d_alpha, wz * dt);
  set_F(d_theta, d_beta, -wy * dt);
  set_F(d_alpha, d_theta, -wz * dt);
  set_F(d_alpha, d_alpha, 1);
  set_F(d_alpha, d_beta, wx * dt);
  set_F(d_beta, d_theta, wy * dt);
  set_F(d_beta, d_alpha, -wx * dt);
  set_F(d_beta, d_beta, 1);
}

static void _build_H(b32 use_gps) {
  mat_clear(&kf.H);
  if (use_gps) {
    kf.H.data[(u32)m_gps_x * state_dim + p_x] = 1.0f;
    kf.H.data[(u32)m_gps_y * state_dim + p_y] = 1.0f;
    kf.H.data[(u32)m_gps_z * state_dim + p_z] = 1.0f;
  }
  kf.H.data[(u32)m_acc_x * state_dim + a_x] = 1.0f;
  kf.H.data[(u32)m_acc_y * state_dim + a_y] = 1.0f;
  kf.H.data[(u32)m_acc_z * state_dim + a_z] = 1.0f;
}

static void _build_I(void) {
  mat_clear(&kf.I);
  for (int i = 0; i < state_dim; i++)
    kf.I.data[i * state_dim + i] = 1.0f;
}

static void _mekf_reset(void) {
  quat dq = {
      .w = 1.0f,
      .x = 0.5f * kf.x_posterior.data[d_theta],
      .y = 0.5f * kf.x_posterior.data[d_alpha],
      .z = 0.5f * kf.x_posterior.data[d_beta],
  };
  quat_normalise(&dq);

  quat qn;
  quat_mul(&qn, &kf.q_ref, &dq);
  quat_normalise(&qn);
  kf.q_ref = qn;

  kf.x_posterior.data[d_theta] = 0.0f;
  kf.x_posterior.data[d_alpha] = 0.0f;
  kf.x_posterior.data[d_beta] = 0.0f;
}

static void _predict(f32 dt, f32 wx, f32 wy, f32 wz) {
  // Propagate reference quaternion with gyro measurement
  quat_propagate(&kf.q_ref, wx, wy, wz, dt);

  // x_prior = F * x_posterior
  _build_F(dt, wx, wy, wz);
  mat_mul_nn(&kf.x_prior, &kf.F, &kf.x_posterior);

  // P_prior = F * P_posterior * F^T + Q
  mat_mul_nn(&kf.FP, &kf.F, &kf.p_posterior);
  mat_mul_nt(&kf.p_prior, &kf.FP, &kf.F);
  mat_add(&kf.p_prior, &kf.p_prior, &kf.Q);
}

static void _compute_gain(void) {
  // S = H * P_prior * H^T + R
  mat_mul_nt(&kf.PHt, &kf.p_prior, &kf.H);
  mat_mul_nn(&kf.S, &kf.H, &kf.PHt);
  mat_add(&kf.S, &kf.S, &kf.R);

  // Regularise to guard against numerical singularity
  for (int i = 0; i < m_dim; i++)
    kf.S.data[i * m_dim + i] += 1e-6f;

  mat_inverse(&kf.S, &kf.S);

  // K = P_prior * H^T * S^-1 = PH^T * S^-1
  mat_mul_nn(&kf.K, &kf.PHt, &kf.S);
}

static void _update_state(void) {
  // innovation = z − H * x_prior
  mat_mul_nn(&kf.innovation, &kf.H, &kf.x_prior);
  mat_sub(&kf.innovation, &kf.z, &kf.innovation);

  // x_posterior = x_prior + K * innovation
  mat_mul_nn(&kf.Ky, &kf.K, &kf.innovation);
  mat_add(&kf.x_posterior, &kf.x_prior, &kf.Ky);

  _mekf_reset();
}

// NOTE: Switched back to simple form instead of Joseph form.
static void _update_covariance(void) {
  // IKH = I − K*H
  mat_mul_nn(&kf.IKH, &kf.K, &kf.H);
  mat_sub(&kf.IKH, &kf.I, &kf.IKH);

  // P_posterior = IKH * P_prior
  mat_mul_nn(&kf.p_posterior, &kf.IKH, &kf.p_prior);
}

b32 kalman_filter_init(const matrix *init_state, quat init_q,
                       f32 init_variance) {
  // Initialise all matrices (rows/cols/data pointer)
  mat_create(&kf.x_prior, state_dim, 1, kf.x_prior_buf);
  mat_create(&kf.x_posterior, state_dim, 1, kf.x_posterior_buf);
  mat_create(&kf.p_prior, state_dim, state_dim, kf.p_prior_buf);
  mat_create(&kf.p_posterior, state_dim, state_dim, kf.p_posterior_buf);
  mat_create(&kf.z, m_dim, 1, kf.z_buf);
  mat_create(&kf.F, state_dim, state_dim, kf.F_buf);
  mat_create(&kf.H, m_dim, state_dim, kf.H_buf);
  mat_create(&kf.Q, state_dim, state_dim, kf.Q_buf);
  mat_create(&kf.R, m_dim, m_dim, kf.R_buf);
  mat_create(&kf.I, state_dim, state_dim, kf.I_buf);
  mat_create(&kf.K, state_dim, m_dim, kf.K_buf);
  mat_create(&kf.FP, state_dim, state_dim, kf.FP_buf);
  mat_create(&kf.PHt, state_dim, m_dim, kf.PHt_buf);
  mat_create(&kf.S, m_dim, m_dim, kf.S_buf);
  mat_create(&kf.IKH, state_dim, state_dim, kf.IKH_buf);
  mat_create(&kf.innovation, m_dim, 1, kf.innovation_buf);
  mat_create(&kf.Ky, state_dim, 1, kf.Ky_buf);

  mat_clear(&kf.z);

  // Constant matrices
  mat_diag(&kf.Q, (f32 *)Q_diag, state_dim);
  mat_diag(&kf.R, (f32 *)R_diag, m_dim);
  _build_I();
  _build_H(false);

  // Initial state and quaternion
  mat_copy(&kf.x_posterior, init_state);
  kf.q_ref = init_q;
  quat_normalise(&kf.q_ref);
  kf.gps_fresh = false;

  // Initial covariance: large for translational states, tight for attitude
  // error
  f32 var[state_dim];
  for (int i = 0; i < state_dim; i++)
    var[i] = init_variance;
  var[d_theta] = 1e-4f;
  var[d_alpha] = 1e-4f;
  var[d_beta] = 1e-4f;
  mat_diag(&kf.p_posterior, var, state_dim);

  return true;
}

void kalman_filter_rotate_accel(f32 ax_b, f32 ay_b, f32 az_b) {
  const f32 a_body[3] = {ax_b, ay_b, az_b};
  f32 a_world[3];
  quat_rotate(&kf.q_ref, a_body, a_world);
  kf.z.data[m_acc_x] = a_world[0];
  kf.z.data[m_acc_y] = a_world[1];
  kf.z.data[m_acc_z] = a_world[2];
}

void kalman_filter_set_gps_enu(f32 x_east, f32 y_north, f32 z_up) {
  kf.z.data[m_gps_x] = x_east;
  kf.z.data[m_gps_y] = y_north;
  kf.z.data[m_gps_z] = z_up;
  kf.gps_fresh = true;
}

matrix *kalman_filter_update(f32 dt, f32 wx, f32 wy, f32 wz) {
  _build_H(kf.gps_fresh);
  _predict(dt, wx, wy, wz);
  _compute_gain();
  _update_state();
  _update_covariance();
  kf.gps_fresh = false;
  return &kf.x_posterior;
}

matrix *kalman_filter_get_z(void) { return &kf.z; }
const quat *kalman_filter_get_quat(void) { return &kf.q_ref; }

void kalman_filter_get_euler_deg(f32 *roll, f32 *pitch, f32 *yaw) {
  quat_to_euler_deg(&kf.q_ref, roll, pitch, yaw);
}

void kalman_filter_debug_print_csv_header(void) {
  Serial.println("t_ms,wx,wy,wz,p_x,p_y,p_z,v_x,v_y,v_z,a_x,a_y,a_z,d_theta,d_"
                 "alpha,d_beta,q_w,q_x,q_y,q_z");
}

void kalman_filter_debug_print_csv_row(u32 t_ms, f32 wx, f32 wy, f32 wz,
                                       const matrix *state) {
  Serial.print(t_ms);
  Serial.print(',');
  Serial.print(wx, 6);
  Serial.print(',');
  Serial.print(wy, 6);
  Serial.print(',');
  Serial.print(wz, 6);
  for (int i = 0; i < state_dim; i++) {
    Serial.print(',');
    Serial.print(state->data[i], 6);
  }
  Serial.print(',');
  Serial.print(kf.q_ref.w, 6);
  Serial.print(',');
  Serial.print(kf.q_ref.x, 6);
  Serial.print(',');
  Serial.print(kf.q_ref.y, 6);
  Serial.print(',');
  Serial.print(kf.q_ref.z, 6);
  Serial.println();
}
