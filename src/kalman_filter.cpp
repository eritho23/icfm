#include "kalman_filter.h"
#include "matrix_utils.h"


MAT(x_prior, state_dim, 1); // State variable, 12x1 variable matrix
MAT(x_posterior, state_dim, 1);

MAT(z, me_dim, 1); // Measurement

MAT(p_prior, state_dim, state_dim); // Error covariance, 12x12 matrix
MAT(p_posterior, state_dim, state_dim);

MAT(K, state_dim, me_dim);      // Kalman gain, 12x9 matrix
MAT(F, state_dim, state_dim);   // State transition, 12x12 matrix
MAT(H, me_dim, state_dim);      // State-to-meurement, 9x12 matrix
MAT(R, me_dim, me_dim);         // meurement noise, 9x9 matrix
MAT(Q, state_dim, state_dim);   // Process noise, 12x12 matrix
MAT(I, state_dim, state_dim);   // Identity, 12x12 matrix

// tmp matrices for computation
MAT(FP, state_dim, state_dim);     // F*P
MAT(PHt, state_dim, me_dim);       // P*Hᵀ
MAT(S, me_dim, me_dim);            // H*P*Hᵀ + R (innovation covariance)
MAT(IKH, state_dim, state_dim);    // I - K*H
MAT(KR, state_dim, me_dim);        // K*R
MAT(KRKt, state_dim, state_dim);   // K*R*Kᵀ
MAT(innovation, me_dim, 1);        // z - H*x
MAT(Ky, state_dim, 1);             // K * innovation


static const f32 Q_diag[state_dim] = {
    /* pos */ 0.01f, 0.01f, 0.01f,
    /* vel */ 0.1f,  0.1f,  0.1f,
    /* acc */ 1.0f,  1.0f,  1.0f,
    /* att */ 0.01f, 0.01f, 0.01f
};

static const f32 R_diag[me_dim] = {
    /* gps */ 2.0f, 2.0f, 4.0f,
    /* acc */ 0.1f, 0.1f, 0.1f,
    /* att */ 0.01f,0.01f,0.05f
};

static void build_H(void) {
    H.data[me_gps_x*state_dim + p_x] = 1;
    H.data[me_gps_y*state_dim + p_y] = 1;
    H.data[me_gps_z*state_dim + p_z] = 1;
    H.data[me_acc_x*state_dim + a_x] = 1;
    H.data[me_acc_y*state_dim + a_y] = 1;
    H.data[me_acc_z*state_dim + a_z] = 1;
    H.data[me_roll*state_dim + roll] = 1;
    H.data[me_pitch*state_dim + pitch] = 1;
    H.data[me_yaw*state_dim + yaw] = 1;
}

#define SET(r,c,val) F->data[(r)*state_dim + (c)] = (val)
static void build_F(matrix *F, f32 dt) {
    mat_clear(F);

    const f32 d2 = 0.5f * dt * dt;
    const int p[3] = { p_x, p_y, p_z };
    const int v[3] = { v_x, v_y, v_z };
    const int a[3] = { a_x, a_y, a_z };
    const int ang[3] = { roll, pitch, yaw };

    for (int i = 0; i < 3; i++) {
        // p = p + v*dt + a*d2
        SET(p[i], p[i], 1);
        SET(p[i], v[i], dt);
        SET(p[i], a[i], d2);
        // v = v + a*dt
        SET(v[i], v[i], 1);
        SET(v[i], a[i], dt);
        // a = a
        SET(a[i], a[i], 1);
        // angles constant
        SET(ang[i], ang[i], 1);
    }
}
#undef SET


void set_measurement(const matrix *meurement) {
    assert(meurement->rows == me_dim && meurement->cols == 1);
    memcpy(z.data, meurement->data, sizeof(f32) * me_dim);
}

// x_n+1|n = F * x_n|n
static void _prior_estimate(f32 dt) {
	build_F(&F, dt);
    assert(mat_mul(&x_prior, &F, &x_posterior, true, false, false));
}

// P_n+1|n = F * P_n|n * F^T + Q
static void _prior_error_covariance(void) {
    assert(mat_mul(&FP,      &F,  &p_posterior, true, false, false));
    assert(mat_mul(&p_prior, &FP, &F,           true, false, true));
    assert(mat_add(&p_prior, &p_prior, &Q));
}

// K_n = P_n|n-1 * H^T(H * P_n|n-1 * H^T + R_n)^(-1)
static void _kalman_gain(void) {
    assert(mat_mul(&PHt, &p_prior, &H,   true, false, true));
    assert(mat_mul(&S,   &H,       &PHt, true, false, false));
    assert(mat_add(&S,   &S,       &R));
    assert(mat_inverse(&S, &S));
    assert(mat_mul(&K,   &PHt,     &S,   true, false, false));
}

// x_n|n = x_n|n-1 + K_n(z_n - H * x_n|n-1)
static void _posterior_estimate(void) {
    assert(mat_mul(&innovation, &H, &x_prior,    true, false, false));
    assert(mat_sub(&innovation, &z, &innovation));
    assert(mat_mul(&Ky,         &K, &innovation, true, false, false));
    assert(mat_add(&x_posterior, &x_prior, &Ky));
}

// P_n|n = (I - K_n * H) * P_n|n-1 * (I - K_n * H)^T + K_n * R_n * K_n^T   (Joseph form)
static void _posterior_error_covariance(void) {
    assert(mat_mul(&IKH,  &K,   &H,    true, false, false));
    assert(mat_sub(&IKH,  &I,   &IKH));
    assert(mat_mul(&p_posterior, &IKH, &p_prior, true, false, false));
    assert(mat_mul(&p_posterior, &p_posterior, &IKH, true, false, true));
    assert(mat_mul(&KR,   &K,   &R,    true, false, false));
    assert(mat_mul(&KRKt, &KR,  &K,    true, false, true));
    assert(mat_add(&p_posterior, &p_posterior, &KRKt));
}


b32 kalman_filter_init(const matrix *init_state, f32 init_variance) {
    mat_diag(&Q, (f32*)Q_diag, state_dim);
    mat_diag(&R, (f32*)R_diag, me_dim);
    mat_diag(&F, (f32[]){ 1,1,1,1,1,1,1,1,1,1,1,1 }, state_dim);
    mat_diag(&I, (f32[]){ 1,1,1,1,1,1,1,1,1,1,1,1 }, state_dim);
    build_H();

    mat_copy(&x_posterior, (matrix*)init_state);

    f32 variance_diag[state_dim];
    for (int i = 0; i < state_dim; i++) variance_diag[i] = init_variance;
    mat_diag(&p_posterior, variance_diag, state_dim);

    return true;
}

matrix* kalman_filter_update(const matrix *m, f32 dt) {
    _prior_estimate(dt);
    _prior_error_covariance();
    mat_copy(&z, (matrix*)m);
    _kalman_gain();
    _posterior_estimate();
    _posterior_error_covariance();

    return &x_posterior;
}
