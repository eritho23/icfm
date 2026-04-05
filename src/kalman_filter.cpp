#include "kalman_filter.h"

MAT(z, m_dim, 1);
MAT(x_prior, state_dim, 1);
MAT(x_posterior, state_dim, 1);
MAT(p_prior, state_dim, state_dim);
MAT(p_posterior, state_dim, state_dim);

MAT(K, state_dim, m_dim);
MAT(F, state_dim, state_dim);
MAT(H, m_dim, state_dim);
MAT(R, m_dim, m_dim);
MAT(Q, state_dim, state_dim);
MAT(I, state_dim, state_dim);

MAT(FP, state_dim, state_dim);
MAT(PHt, state_dim, m_dim);
MAT(S, m_dim, m_dim);
MAT(IKH, state_dim, state_dim);
MAT(KR, state_dim, m_dim);
MAT(KRKt, state_dim, state_dim);
MAT(innovation, m_dim, 1);
MAT(Ky, state_dim, 1);

static quat q_ref = { 1.0f, 0.0f, 0.0f, 0.0f };

#define SET(mat, r, c, val) (mat).data[(int)(r)*(mat).cols + (int)(c)] = (val)

static const f32 Q_diag[state_dim] = {
    0.01f, 0.01f, 0.01f,  // pos
    0.1f,  0.1f,  0.1f,   // vel
    0.3f,  0.3f,  0.3f,   // acc — high: thrust/drag are highly unpredictable
    1e-3f, 1e-3f, 1e-3f,  // delta_theta,
};

static const f32 R_diag[m_dim] = {
    1.5f, 1.5f, 4.0f,  // gps  (metres)
    0.1f, 0.1f, 0.1f,  // accel (m/s²)
};

// State transition Jacobian F (12x12).
//
// Translational block:
//   p_{k+1} = p_k + v_k*dt + a_k*0.5*dt^2
//   v_{k+1} = v_k + a_k*dt
//   a_{k+1} = a_k  (random walk — Q_acc drives it)
//
// Attitude error block: delta_theta_{k+1} = (I - [omega x]*dt) * delta_theta_k
//
// [omega x] skew:           I - [omega x]*dt:
//   |  0  -wz   wy |          |  1      wz*dt  -wy*dt |
//   |  wz   0  -wx |    =>    | -wz*dt  1       wx*dt |
//   | -wy   wx   0 |          |  wy*dt -wx*dt   1     |
static void build_F(matrix *F, f32 dt, f32 wx, f32 wy, f32 wz) {
    mat_clear(F);

    const f32 d2 = 0.5f * dt * dt;

    SET(*F, p_x, p_x, 1.0f); SET(*F, p_x, v_x, dt); SET(*F, p_x, a_x, d2);
    SET(*F, p_y, p_y, 1.0f); SET(*F, p_y, v_y, dt); SET(*F, p_y, a_y, d2);
    SET(*F, p_z, p_z, 1.0f); SET(*F, p_z, v_z, dt); SET(*F, p_z, a_z, d2);
    SET(*F, v_x, v_x, 1.0f); SET(*F, v_x, a_x, dt);
    SET(*F, v_y, v_y, 1.0f); SET(*F, v_y, a_y, dt);
    SET(*F, v_z, v_z, 1.0f); SET(*F, v_z, a_z, dt);
    SET(*F, a_x, a_x, 1.0f);
    SET(*F, a_y, a_y, 1.0f);
    SET(*F, a_z, a_z, 1.0f);

    SET(*F, d_theta, d_theta,  1.0f);  SET(*F, d_theta, d_alpha,  wz*dt); SET(*F, d_theta, d_beta, -wy*dt);
    SET(*F, d_alpha, d_theta, -wz*dt); SET(*F, d_alpha, d_alpha,  1.0f);  SET(*F, d_alpha, d_beta,  wx*dt);
    SET(*F, d_beta,  d_theta,  wy*dt); SET(*F, d_beta,  d_alpha, -wx*dt); SET(*F, d_beta,  d_beta,  1.0f);
}

static void build_H(void) {
    mat_clear(&H);
    SET(H, m_gps_x, p_x, 1.0f);
    SET(H, m_gps_y, p_y, 1.0f);
    SET(H, m_gps_z, p_z, 1.0f);
    SET(H, m_acc_x, a_x, 1.0f);
    SET(H, m_acc_y, a_y, 1.0f);
    SET(H, m_acc_z, a_z, 1.0f);
}

static void build_I(void) {
    mat_clear(&I);
    for (int i = 0; i < state_dim; i++) SET(I, i, i, 1.0f);
}

static void _mekf_reset(void) {
    quat dq;
    dq.w = 1.0f;
    dq.x = 0.5f * x_posterior.data[d_theta];
    dq.y = 0.5f * x_posterior.data[d_alpha];
    dq.z = 0.5f * x_posterior.data[d_beta];
    quat_normalise(&dq);
    quat qn;
	quat_mul(&qn, &q_ref, &dq);  // q_ref * dq
    quat_normalise(&qn);
    q_ref.w = qn.w; q_ref.x = qn.x; q_ref.y = qn.y; q_ref.z = qn.z;
    x_posterior.data[d_theta] = 0.0f;
    x_posterior.data[d_alpha] = 0.0f;
    x_posterior.data[d_beta]  = 0.0f;
}

static void _prior_estimate(f32 dt, f32 wx, f32 wy, f32 wz) {
    quat_propagate(&q_ref, wx, wy, wz, dt);
    build_F(&F, dt, wx, wy, wz);
    assert(mat_mul(&x_prior, &F, &x_posterior, true, false, false));
}

static void _prior_error_covariance(void) {
    assert(mat_mul(&FP, &F, &p_posterior, true, false, false));
    assert(mat_mul(&p_prior, &FP, &F, true, false, true));
    assert(mat_add(&p_prior, &p_prior, &Q));
}

static void _kalman_gain(void) {
    assert(mat_mul(&PHt, &p_prior, &H, true, false, true));
    assert(mat_mul(&S, &H, &PHt, true, false, false));
    assert(mat_add(&S, &S, &R));
    for (int i = 0; i < m_dim; i++) S.data[i*m_dim + i] += 1e-6f;
    assert(mat_inverse(&S, &S));
    assert(mat_mul(&K, &PHt, &S, true, false, false));
}

static void _posterior_estimate(void) {
    assert(mat_mul(&innovation, &H, &x_prior, true, false, false));
    assert(mat_sub(&innovation, &z, &innovation));
    assert(mat_mul(&Ky, &K, &innovation, true, false, false));
    assert(mat_add(&x_posterior, &x_prior, &Ky));
    _mekf_reset();
}

static void _posterior_error_covariance(void) {
    assert(mat_mul(&IKH, &K, &H, true, false, false));
    assert(mat_sub(&IKH, &I, &IKH));
    assert(mat_mul(&p_posterior, &IKH, &p_prior, true, false, false));
    assert(mat_mul(&p_posterior, &p_posterior, &IKH, true, false, true));
    assert(mat_mul(&KR, &K, &R, true, false, false));
    assert(mat_mul(&KRKt, &KR, &K, true, false, true));
    assert(mat_add(&p_posterior, &p_posterior, &KRKt));
}

b32 kalman_filter_init(const matrix *init_state, quat init_q, f32 init_var) {
    mat_diag(&Q, (f32*)Q_diag, state_dim);
    mat_diag(&R, (f32*)R_diag, m_dim);
    build_I();
    build_H();
    mat_copy(&x_posterior, (matrix*)init_state);
    q_ref.w=init_q.w; q_ref.x=init_q.x; q_ref.y=init_q.y; q_ref.z=init_q.z;
    quat_normalise(&q_ref);
    f32 var[state_dim];
    for (int i = 0; i < state_dim; i++) var[i] = init_var;
    var[d_theta] = 1e-4f;
    var[d_alpha] = 1e-4f;
    var[d_beta]  = 1e-4f;
    mat_diag(&p_posterior, var, state_dim);
    return true;
}

void kalman_filter_rotate_accel(f32 ax_b, f32 ay_b, f32 az_b) {
    const f32 a_body[3] = {ax_b, ay_b, az_b};
    f32 a_world[3];
    quat_rotate(&q_ref, a_body, a_world);
    z.data[m_acc_x] = a_world[0];
    z.data[m_acc_y] = a_world[1];
    z.data[m_acc_z] = a_world[2];
}

matrix* kalman_filter_update(f32 dt, f32 wx, f32 wy, f32 wz) {
    _prior_estimate(dt, wx, wy, wz);
    _prior_error_covariance();
    _kalman_gain();
    _posterior_estimate();
    _posterior_error_covariance();
    return &x_posterior;
}

matrix* kalman_filter_get_z(void) { return &z; }
const quat* kalman_filter_get_quat(void) { return &q_ref; }

void kalman_filter_get_euler_deg(f32 *roll, f32 *pitch, f32 *yaw) {
    quat_to_euler_deg(&q_ref, roll, pitch, yaw);
}

void kalman_filter_debug_print_csv_header(void) {
    Serial.println("t_ms,wx,wy,wz,p_x,p_y,p_z,v_x,v_y,v_z,a_x,a_y,a_z,d_theta,d_alpha,d_beta,q_w,q_x,q_y,q_z");
}

void kalman_filter_debug_print_csv_row(u32 t_ms, f32 wx, f32 wy, f32 wz, const matrix *state) {
    Serial.print(t_ms); Serial.print(',');
    Serial.print(wx, 6); Serial.print(',');
    Serial.print(wy, 6); Serial.print(',');
    Serial.print(wz, 6);
    for (int i = 0; i < state_dim; i++) { Serial.print(','); Serial.print(state->data[i], 6); }
    Serial.print(','); Serial.print(q_ref.w, 6);
    Serial.print(','); Serial.print(q_ref.x, 6);
    Serial.print(','); Serial.print(q_ref.y, 6);
    Serial.print(','); Serial.print(q_ref.z, 6);
    Serial.println();
}
