#include "kalman_filter.h"
#include "matrix_utils.h"

typedef enum {
    p_x = 0, p_y, p_z,
    v_x, v_y, v_z,
    a_x, a_y, a_z,
    roll, pitch, yaw,
	dim // = 12
} state_index;

typedef enum {
    meas_gps_x = 0, meas_gps_y, meas_gps_z,
    meas_acc_x, meas_acc_y, meas_acc_z,
    meas_roll, meas_pitch, meas_yaw,
    meas_dim // = 9
} meas_index;


MAT(x_prior, dim, 1); // State variable, 12x1 variable matrix
MAT(x_posterior, dim, 1);
MAT(z, meas_dim, 1);

MAT(p_prior, dim, dim); // Error covariance, 12x12 matrix
MAT(p_posterior, dim, dim);
MAT(K, dim, meas_dim);

MAT(F, dim, dim); // State transition, 12x12 matrix
MAT(H, meas_dim, dim); // State-to-measurement, 9x12 matrix
MAT(R, meas_dim, meas_dim); // Measurement noise, 9x9 matrix
MAT(Q, dim, dim); // Process noise, 12x12 matrix
MAT(I, dim, dim); // Identity, 12x12 matrix

MAT(FP,         dim,      dim);      // F*P
MAT(PHt,        dim,      meas_dim); // P*Hᵀ
MAT(S,          meas_dim, meas_dim); // H*P*Hᵀ + R (innovation covariance)
MAT(IKH,        dim,      dim);      // I - K*H
MAT(KR,         dim,      meas_dim); // K*R
MAT(KRKt,       dim,      dim);      // K*R*Kᵀ
MAT(innovation, meas_dim, 1);        // z - H*x
MAT(Ky,         dim,      1);        // K * innovation

static const f32 Q_diag[dim] = {
    /* pos */ 0.01f, 0.01f, 0.01f,
    /* vel */ 0.1f,  0.1f,  0.1f,
    /* acc */ 1.0f,  1.0f,  1.0f,
    /* att */ 0.01f, 0.01f, 0.01f
};

static const f32 R_diag[meas_dim] = {
    /* gps */ 2.0f, 2.0f, 4.0f,
    /* acc */ 0.1f, 0.1f, 0.1f,
    /* att */ 0.01f,0.01f,0.05f
};


static void build_H(void) {
    H.data[meas_gps_x*dim + p_x] = 1;
    H.data[meas_gps_y*dim + p_y] = 1;
    H.data[meas_gps_z*dim + p_z] = 1;
    H.data[meas_acc_x*dim + a_x] = 1;
    H.data[meas_acc_y*dim + a_y] = 1;
    H.data[meas_acc_z*dim + a_z] = 1;
    H.data[meas_roll*dim + roll] = 1;
    H.data[meas_pitch*dim + pitch] = 1;
    H.data[meas_yaw*dim + yaw] = 1;
}

#define SET(r,c,val) F->data[(r)*dim + (c)] = (val)
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

void set_measurement(const matrix *measurement) {
    assert(measurement->rows == meas_dim && measurement->cols == 1);
    memcpy(z.data, measurement->data, sizeof(f32) * meas_dim);
}

static void prior_estimate(f32 dt) {
	build_F(&F, dt);
    assert(mat_mul(&x_prior, &F, &x_posterior, true, false, false));
}

static void _prior_error_covariance(void) {
    assert(mat_mul(&FP,      &F,  &p_posterior, true, false, false));
    assert(mat_mul(&p_prior, &FP, &F,           true, false, true));
    assert(mat_add(&p_prior, &p_prior, &Q));
}

static void _kalman_gain(void) {
    assert(mat_mul(&PHt, &p_prior, &H,   true, false, true));
    assert(mat_mul(&S,   &H,       &PHt, true, false, false));
    assert(mat_add(&S,   &S,       &R));
    assert(mat_inverse(&S, &S));
    assert(mat_mul(&K,   &PHt,     &S,   true, false, false));
}

static void _posterior_estimate(void) {
    assert(mat_mul(&innovation, &H, &x_prior,    true, false, false));
    assert(mat_sub(&innovation, &z, &innovation));
    assert(mat_mul(&Ky,         &K, &innovation, true, false, false));
    assert(mat_add(&x_posterior, &x_prior, &Ky));
}

static void _posterior_error_covariance(void) {
    assert(mat_mul(&IKH,  &K,   &H,    true, false, false));
    assert(mat_sub(&IKH,  &I,   &IKH));
    assert(mat_mul(&p_posterior, &IKH, &p_prior, true, false, false));
    assert(mat_mul(&p_posterior, &p_posterior, &IKH, true, false, true));
    assert(mat_mul(&KR,   &K,   &R,    true, false, false));
    assert(mat_mul(&KRKt, &KR,  &K,    true, false, true));
    assert(mat_add(&p_posterior, &p_posterior, &KRKt));
}


b32 init_kalman_filter(matrix init_state, matrix init_ec) {
	mat_diag(&Q, (f32*)Q_diag, dim);
	mat_diag(&R, (f32*)R_diag, meas_dim);
    mat_diag(&F, (f32[]){ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, dim);
    mat_diag(&I, (f32[]){ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, dim);
    build_H();

    if (!mat_copy(&x_posterior, &init_state)) {
        return false;
    }

    if (init_ec.rows == dim && init_ec.cols == dim) {
        if (!mat_copy(&p_posterior, &init_ec)) {
            return false;
        }
    } else {
        mat_diag(&p_posterior, (f32[]){ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, dim);
    }

    return true;
}

const matrix* kalman_update(const matrix *measurement, f32 dt) {
    prior_estimate(dt);
    _prior_error_covariance();
    set_measurement(measurement);
	_kalman_gain();
    _posterior_estimate();
    _posterior_error_covariance();

    return &x_posterior;
}
