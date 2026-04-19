#ifndef ICFM_MATRIX_UTILS_H
#define ICFM_MATRIX_UTILS_H

#include "../include/base.h"

typedef struct {
    u32 rows, cols;
    // Row-major
    f32 *data;
} matrix;

#define MAT(name, r, c)                                                        \
    f32 name##_buf[(u32)(r) * (u32)(c)];                                       \
    matrix name

// buf must point to a caller-owned f32[rows * cols] buffer.
void mat_create(matrix *mat, u32 rows, u32 cols, f32 *buf);
void mat_diag(matrix *m, f32 *values, u32 n);
b32 mat_copy(matrix *dst, const matrix *src);
void mat_clear(matrix *mat);
void mat_fill(matrix *mat, f32 x);
void mat_scale(matrix *mat, f32 scale);

f32 mat_sum(matrix *mat);
b32 mat_add(matrix *out, const matrix *a, const matrix *b);
b32 mat_sub(matrix *out, const matrix *a, const matrix *b);

b32 mat_transpose(matrix *out, const matrix *a);
b32 mat_inverse(matrix *out, const matrix *a);

void mat_mul_nn(matrix *out, const matrix *a, const matrix *b); // A * B
void mat_mul_nt(matrix *out, const matrix *a, const matrix *b); // A * B^T
void mat_mul_tn(matrix *out, const matrix *a, const matrix *b); // A^T * B
void mat_mul_tt(matrix *out, const matrix *a, const matrix *b); // A^T * B^T

#endif
