#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "base.h"
#include "matrix_utils.h"

extern matrix x_posterior;

b32 init_kalman_filter(matrix init_state, matrix init_ec);
const matrix* kalman_update(const matrix *measurement, f32 dt);
void set_measurement(const matrix *measurement);

#endif
