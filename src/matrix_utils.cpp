#include "matrix_utils.h"

// FIX: Look at how we can optimize some of these

void mat_create(matrix *mat, u32 rows, u32 cols, f32 *buf) {
    mat->rows = rows;
    mat->cols = cols;
    mat->data = buf;
}

void mat_diag(matrix *m, f32 *values, u32 n) {
    mat_clear(m);
    for (u32 i = 0; i < n; i++) {
        m->data[i * m->cols + i] = values[i];
    }
}

b32 mat_copy(matrix *dst, const matrix *src) {
    if (dst->rows != src->rows || dst->cols != src->cols) {
        return false;
    }

    memcpy(dst->data, src->data, sizeof(f32) * dst->rows * dst->cols);

    return true;
}

void mat_clear(matrix *mat) {
    memset(mat->data, 0, sizeof(f32) * mat->rows * mat->cols);
}

void mat_fill(matrix *mat, f32 x) {
    u32 size = mat->rows * mat->cols;

    for (u32 i = 0; i < size; i++) {
        mat->data[i] = x;
    }
}

void mat_scale(matrix *mat, f32 scale) {
    u32 size = mat->rows * mat->cols;

    for (u32 i = 0; i < size; i++) {
        mat->data[i] *= scale;
    }
}

f32 mat_sum(matrix *mat) {
    u32 size = mat->rows * mat->cols;

    f32 sum = 0.0f;
    for (u32 i = 0; i < size; i++) {
        sum += mat->data[i];
    }

    return sum;
}

b32 mat_add(matrix *out, const matrix *a, const matrix *b) {
    if (a->rows != b->rows || a->cols != b->cols) {
        return false;
    }
    if (out->rows != a->rows || out->cols != a->cols) {
        return false;
    }

    u32 size = (u32)out->rows * out->cols;
    for (u32 i = 0; i < size; i++) {
        out->data[i] = a->data[i] + b->data[i];
    }

    return true;
}

b32 mat_sub(matrix *out, const matrix *a, const matrix *b) {
    if (a->rows != b->rows || a->cols != b->cols) {
        return false;
    }
    if (out->rows != a->rows || out->cols != a->cols) {
        return false;
    }

    u32 size = (u32)out->rows * out->cols;
    for (u32 i = 0; i < size; i++) {
        out->data[i] = a->data[i] - b->data[i];
    }

    return true;
}

void mat_mul_nn(matrix *out, const matrix *a, const matrix *b) {
    for (u32 i = 0; i < out->rows; i++)
        for (u32 j = 0; j < out->cols; j++) {
            f32 sum = 0.0f;
            for (u32 k = 0; k < a->cols; k++)
                sum += a->data[k + i*a->cols] * b->data[j + k*b->cols];
            out->data[j + i*out->cols] = sum;
        }
}

void mat_mul_nt(matrix *out, const matrix *a, const matrix *b) {
    for (u32 i = 0; i < out->rows; i++)
        for (u32 j = 0; j < out->cols; j++) {
            f32 sum = 0.0f;
            for (u32 k = 0; k < a->cols; k++)
                sum += a->data[k + i*a->cols] * b->data[k + j*b->cols];
            out->data[j + i*out->cols] = sum;
        }
}

void mat_mul_tn(matrix *out, const matrix *a, const matrix *b) {
    for (u32 i = 0; i < out->rows; i++)
        for (u32 j = 0; j < out->cols; j++) {
            f32 sum = 0.0f;
            for (u32 k = 0; k < a->rows; k++)
                sum += a->data[i + k*a->cols] * b->data[j + k*b->cols];
            out->data[j + i*out->cols] = sum;
        }
}

void mat_mul_tt(matrix *out, const matrix *a, const matrix *b) {
    for (u32 i = 0; i < out->rows; i++)
        for (u32 j = 0; j < out->cols; j++) {
            f32 sum = 0.0f;
            for (u32 k = 0; k < a->rows; k++)
                sum += a->data[i + k*a->cols] * b->data[k + j*b->cols];
            out->data[j + i*out->cols] = sum;
        }
}

b32 mat_transpose(matrix *out, const matrix *a) {
    if (out->rows != a->cols || out->cols != a->rows) {
        return false;
    }

    for (u32 i = 0; i < a->rows; i++) {
        for (u32 j = 0; j < a->cols; j++) {
            out->data[i + j * out->cols] = a->data[j + i * a->cols];
        }
    }

    return true;
}

b32 mat_inverse(matrix *out, const matrix *a) {
    if (a->rows != a->cols) { return false; }
    if (out->rows != a->rows || out->cols != a->cols) { return false; }

    u32 n = a->rows;

    // Build augmented matrix [A | I] as a flat buffer on the stack.
    // For large n you may want a heap allocation instead.
    f32 aug[n * 2 * n];

    for (u32 i = 0; i < n; i++) {
        for (u32 j = 0; j < n; j++) {
            aug[j + i * 2 * n]     = a->data[j + i * n];  // copy A
            aug[(j + n) + i * 2 * n] = (i == j) ? 1.0f : 0.0f;  // identity
        }
    }

    // Gauss-Jordan elimination with partial pivoting
    for (u32 col = 0; col < n; col++) {

        // Find pivot row (largest absolute value in this column)
        u32 pivot = col;
        f32 best  = fabsf(aug[col + col * 2 * n]);
        for (u32 row = col + 1; row < n; row++) {
            f32 val = fabsf(aug[col + row * 2 * n]);
            if (val > best) {
                best  = val;
                pivot = row;
            }
        }

        // Singular (or numerically singular) matrix
        if (best < 1e-6f) { return false; }

        // Swap current row with pivot row
        if (pivot != col) {
            for (u32 j = 0; j < 2 * n; j++) {
                f32 tmp                   = aug[j + col   * 2 * n];
                aug[j + col   * 2 * n]   = aug[j + pivot * 2 * n];
                aug[j + pivot * 2 * n]   = tmp;
            }
        }

        // Scale pivot row so the diagonal becomes 1
        f32 inv_diag = 1.0f / aug[col + col * 2 * n];
        for (u32 j = 0; j < 2 * n; j++) {
            aug[j + col * 2 * n] *= inv_diag;
        }

        // Eliminate this column in all other rows
        for (u32 row = 0; row < n; row++) {
            if (row == col) { continue; }
            f32 factor = aug[col + row * 2 * n];
            for (u32 j = 0; j < 2 * n; j++) {
                aug[j + row * 2 * n] -= factor * aug[j + col * 2 * n];
            }
        }
    }

    // Extract the right half into out
    for (u32 i = 0; i < n; i++) {
        for (u32 j = 0; j < n; j++) {
            out->data[j + i * n] = aug[(j + n) + i * 2 * n];
        }
    }

    return true;
}
