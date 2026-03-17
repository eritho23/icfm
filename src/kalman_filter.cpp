#include "kalman_filter.h"
#include "matrix_utils.h"

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
    roll,
	pitch,
	yaw,
    dim // = 12
} state_index;

typedef enum {
    meas_gps_x = 0,
    meas_gps_y,
    meas_gps_z,
    meas_acc_x,
    meas_acc_y,
    meas_acc_z,
    meas_roll,
    meas_pitch,
    meas_yaw,
    meas_dim // = 9
} meas_index;


MAT(x_prior, dim, 1);
MAT(x_posterior, dim, 1);

MAT(Q, dim, dim); // Process noise, 12x12 matrix
MAT(R, 9, 9); // Measurement noise, 9x9 matrix
MAT(F, dim, dim); // State transition, 12x12 matrix
MAT(H, 9, dim); // State-to-measurement, 12x9 matrix
MAT(P, dim, dim); // Error covariance, 12x12 matrix


static void init_H(void) {
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

void update_F(f32 dt) {
    f32 d2 = 0.5f * dt * dt;
    F.data[p_x*dim + v_x] = dt;  F.data[p_x*dim + a_x] = d2;
    F.data[p_y*dim + v_y] = dt;  F.data[p_y*dim + a_y] = d2;
    F.data[p_z*dim + v_z] = dt;  F.data[p_z*dim + a_z] = d2;
    F.data[v_x*dim + a_x] = dt;
    F.data[v_y*dim + a_y] = dt;
    F.data[v_z*dim + a_z] = dt;
}


b32 init_kalman_filter(matrix *init_state) {
	// TODO: Figure out the right values
    mat_diag(&Q, (f32[]){ 0.01f, 0.01f, 0.01f,  0.1f, 0.1f, 0.1f,  1.0f, 1.0f, 1.0f,  0.01f, 0.01f, 0.01f }, dim);
    mat_diag(&R, (f32[]){ 2.0f,  2.0f,  4.0f,   0.1f, 0.1f, 0.1f,  0.01f, 0.01f, 0.05f }, 9);
    mat_diag(&P, (f32[]){ 1, 1, 1,  1, 1, 1,  1, 1, 1,  1, 1, 1 }, dim);
    mat_diag(&F, (f32[]){ 1, 1, 1,  1, 1, 1,  1, 1, 1,  1, 1, 1 }, dim);
    init_H();

    mat_copy(&x_posterior, init_state);
    return true;
}

b32 prior_estimate(f32 dt) {
    update_F(dt);
    return mat_mul(&x_prior, &F, &x_posterior, true, false, false);
}

b32 prior_error_covariance(void) {
    // TODO: F*P*F^T + Q

    return true;
}

b32 posterior_error_covariance(void) {
    // TODO: compute with Joseph form

	return true;
}
