#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "base.h"
#include "matrix_utils.h"

extern matrix x_posterior;

void create_F(matrix *F, f32 dt);
void create_Q(matrix *Q);
void create_H(matrix *H);
void create_R(matrix *R);
void init_P(matrix *P);

b32 init_kalman_filter(matrix init_state, matrix init_ec);
b32 prior_estimate(matrix *out, matrix *F, matrix *x, f32 dt);
b32 prior_ec(matrix *P_out, const matrix *P, const matrix *F, const matrix *Q);

#endif
