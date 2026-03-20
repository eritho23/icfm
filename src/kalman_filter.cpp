#include "kalman_filter.h"



/*
	Define matrices we need to compute/set.
	NOTE: state_dim and me_dim define the dimensions N and M respectively.
*/

MAT(z, me_dim, 1); // Measurement, 12x1 matrix

MAT(x_prior, state_dim, 1); MAT(x_posterior, state_dim, 1); // State variable, 12x1 matrix
MAT(p_prior, state_dim, state_dim); MAT(p_posterior, state_dim, state_dim); // Error covariance, 12x12 matrix

MAT(K, state_dim, me_dim);      // Kalman gain, 12x9 matrix
MAT(F, state_dim, state_dim);   // State transition, 12x12 matrix
MAT(H, me_dim, state_dim);      // State-to-meurement, 9x12 matrix
MAT(R, me_dim, me_dim);         // meurement noise, 9x9 matrix
MAT(Q, state_dim, state_dim);   // Process noise, 12x12 matrix
MAT(I, state_dim, state_dim);   // Identity, 12x12 matrix

// Purely tmp matrices for computations
MAT(FP, state_dim, state_dim);     // F*P
MAT(PHt, state_dim, me_dim);       // P*Hᵀ
MAT(S, me_dim, me_dim);            // H*P*Hᵀ + R (innovation covariance)
MAT(IKH, state_dim, state_dim);    // I - K*H
MAT(KR, state_dim, me_dim);        // K*R
MAT(KRKt, state_dim, state_dim);   // K*R*Kᵀ
MAT(innovation, me_dim, 1);        // z - H*x
MAT(Ky, state_dim, 1);             // K * innovation



/*
   Build matrices, these need tuning.
*/

#define SET(mat, r, c, val) (mat).data[(int)(r)*(mat).cols + (int)(c)] = (val)

static const f32 Q_diag[state_dim] = {
    /* pos */ 0.01f,  0.01f,  0.01f,   // raise back up — model drifts in pos
    /* vel */ 0.1f,   0.1f,   0.1f,    // raise back up — velocity changes fast
    /* acc */ 0.5f,   0.5f,   0.5f,    // raise significantly — accel is unpredictable
    /* att */ 0.01f,  0.01f,  0.01f,   // raise back up
};

static const f32 R_diag[me_dim] = {
    /* gps */ 2.0f,  2.0f,  4.0f,     // unchanged
    /* acc */ 0.3f,  0.3f,  0.3f,     // slight trust, but not as low as 0.1
    /* att */ 0.04f, 0.04f, 0.12f,    // moderate trust
};

static void build_H(void) {
	SET(H, me_gps_x, p_x, 1);
	SET(H, me_gps_y, p_y, 1);
	SET(H, me_gps_z, p_z, 1);
	SET(H, me_acc_x, a_x, 1);
	SET(H, me_acc_y, a_y, 1);
	SET(H, me_acc_z, a_z, 1);
	SET(H, me_roll, roll, 1);
	SET(H, me_pitch, pitch, 1);
	SET(H, me_yaw, yaw, 1);
}

static void build_F(matrix *F, f32 dt) {
	mat_clear(F);

	const f32 d2 = 0.5f * dt * dt;
	const int p[3] = { p_x, p_y, p_z };
	const int v[3] = { v_x, v_y, v_z };
	const int a[3] = { a_x, a_y, a_z };
	const int ang[3] = { roll, pitch, yaw };

	for (int i = 0; i < 3; i++) {
		// p = p + v*dt + a*d2
		SET(*F, p[i], p[i], 1);
		SET(*F, p[i], v[i], dt);
		SET(*F, p[i], a[i], d2);
		// v = v + a*dt
		SET(*F, v[i], v[i], 1);
		SET(*F, v[i], a[i], dt);
		// a = a
		SET(*F, a[i], a[i], 1);
		// angles constant
		SET(*F, ang[i], ang[i], 1);
	}
}

static void build_I(void) {
	mat_clear(&I);
	for (int i = 0; i < state_dim; i++)
		SET(I, i, i, 1);
}



/*
   Computation steps
*/

// x_n+1|n = F * x_n|n
static void _prior_estimate(f32 dt) {
	build_F(&F, dt);
	assert(mat_mul(&x_prior, &F, &x_posterior, true, false, false));
}

// P_n+1|n = F * P_n|n * F^T + Q
static void _prior_error_covariance(void) {
	assert(mat_mul(&FP, &F, &p_posterior, true, false, false));
	assert(mat_mul(&p_prior, &FP, &F, true, false, true));
	assert(mat_add(&p_prior, &p_prior, &Q));
}

// K_n = P_n|n-1 * H^T(H * P_n|n-1 * H^T + R_n)^(-1)
static void _kalman_gain(void) {
	assert(mat_mul(&PHt, &p_prior, &H, true, false, true));
	assert(mat_mul(&S, &H, &PHt, true, false, false));
	assert(mat_add(&S, &S, &R));
	assert(mat_inverse(&S, &S));
	assert(mat_mul(&K, &PHt, &S, true, false, false));
}

// x_n|n = x_n|n-1 + K_n(z_n - H * x_n|n-1)
static void _posterior_estimate(void) {
	assert(mat_mul(&innovation, &H, &x_prior, true, false, false));
	assert(mat_sub(&innovation, &z, &innovation));
	assert(mat_mul(&Ky, &K, &innovation, true, false, false));
	assert(mat_add(&x_posterior, &x_prior, &Ky));
}

// P_n|n = (I - K_n * H) * P_n|n-1 * (I - K_n * H)^T + K_n * R_n * K_n^T   (Joseph form)
static void _posterior_error_covariance(void) {
	assert(mat_mul(&IKH, &K, &H, true, false, false));
	assert(mat_sub(&IKH, &I, &IKH));
	assert(mat_mul(&p_posterior, &IKH, &p_prior, true, false, false));
	assert(mat_mul(&p_posterior, &p_posterior, &IKH, true, false, true));
	assert(mat_mul(&KR, &K, &R, true, false, false));
	assert(mat_mul(&KRKt, &KR, &K, true, false, true));
	assert(mat_add(&p_posterior, &p_posterior, &KRKt));
}



b32 kalman_filter_init(const matrix *init_state, f32 init_variance) {
	mat_diag(&Q, (f32*)Q_diag, state_dim);
	mat_diag(&R, (f32*)R_diag, me_dim);
	build_I();
	build_H();

	mat_copy(&x_posterior, (matrix*)init_state);

	f32 variance_diag[state_dim];
	for (int i = 0; i < state_dim; i++) variance_diag[i] = init_variance;
	mat_diag(&p_posterior, variance_diag, state_dim);

	return true;
}

matrix* kalman_filter_update(f32 dt) {
	_prior_estimate(dt);
	_prior_error_covariance();
	_kalman_gain();
	_posterior_estimate();
	_posterior_error_covariance();

	return &x_posterior;
}

matrix* kalman_filter_get_z(void) { return &z; }



/*
	Functions to log via serial (wired)
	TODO: Maybe move to a separate loggin system later that is wireless and more robust.
*/
void kalman_filter_debug_print_csv_header(void) {
	Serial.println("t_ms,ax_mps2,ay_mps2,az_mps2,roll_rad,pitch_rad,yaw_rad,p_x,p_y,p_z,v_x,v_y,v_z,a_x,a_y,a_z,roll,pitch,yaw");
}

void kalman_filter_debug_print_csv_row(u32 t_ms, f32 ax, f32 ay, f32 az, f32 roll_meas, f32 pitch_meas, f32 yaw_meas, const matrix *state) {
	Serial.print(t_ms);
	Serial.print(',');
	Serial.print(ax, 6);
	Serial.print(',');
	Serial.print(ay, 6);
	Serial.print(',');
	Serial.print(az, 6);
	Serial.print(',');
	Serial.print(roll_meas, 6);
	Serial.print(',');
	Serial.print(pitch_meas, 6);
	Serial.print(',');
	Serial.print(yaw_meas, 6);

	for (int i = 0; i < state_dim; i++) {
		Serial.print(',');
		Serial.print(state->data[i], 6);
	}

	Serial.println();
}
