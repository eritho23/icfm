#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H

#include "base.h"

typedef struct {
    u32 rows, cols;
    // Row-major
    f32* data;
} matrix;

// buf must point to a caller-owned f32[rows * cols] buffer
void mat_create(matrix* mat, u32 rows, u32 cols, f32* buf);
b32 mat_copy(matrix* dst, matrix* src);
void mat_clear(matrix* mat);
void mat_fill(matrix* mat, f32 x);
void mat_scale(matrix* mat, f32 scale);
f32 mat_sum(matrix* mat);
u64 mat_argmax(matrix* mat);
b32 mat_add(matrix* out, const matrix* a, const matrix* b);
b32 mat_sub(matrix* out, const matrix* a, const matrix* b);
b32 mat_mul(
    matrix* out, const matrix* a, const matrix* b,
    b8 zero_out, b8 transpose_a, b8 transpose_b
);
b32 mat_transpose(matrix* out, const matrix* a);

#endif
