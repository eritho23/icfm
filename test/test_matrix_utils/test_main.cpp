#include <unity.h>
#include "matrix_utils.h"
#include "matrix_utils.cpp"

void setUp(void) {}
void tearDown(void) {}

// Invert a known 2x2 matrix and verify A * A^-1 == I
void test_mat_inverse_2x2(void) {
    // A = [4, 7; 2, 6]  =>  A^-1 = [0.6, -0.7; -0.2, 0.4]
    f32 a_buf[4];
    f32 inv_buf[4];
    f32 result_buf[4];

    matrix a, inv, result;
    mat_create(&a,      2, 2, a_buf);
    mat_create(&inv,    2, 2, inv_buf);
    mat_create(&result, 2, 2, result_buf);

    // Set A values AFTER mat_create (which zeroes the buffer)
    a.data[0] = 4.0f; a.data[1] = 7.0f;
    a.data[2] = 2.0f; a.data[3] = 6.0f;

    // Inverse should succeed
    TEST_ASSERT_TRUE(mat_inverse(&inv, &a));

    // A * A^-1 should equal identity
    TEST_ASSERT_TRUE(mat_mul(&result, &a, &inv, /*zero_out=*/true, false, false));

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, result.data[0]); // [0,0]
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, result.data[1]); // [0,1]
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, result.data[2]); // [1,0]
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, result.data[3]); // [1,1]
}

// Singular matrix (determinant == 0) should return false
void test_mat_inverse_singular(void) {
    f32 a_buf[4] = {1.0f, 2.0f, 2.0f, 4.0f};  // row2 = 2 * row1
    f32 inv_buf[4] = {};

    matrix a, inv;
    mat_create(&a,   2, 2, a_buf);
    mat_create(&inv, 2, 2, inv_buf);

    TEST_ASSERT_FALSE(mat_inverse(&inv, &a));
}

// Non-square matrix should return false
void test_mat_inverse_non_square(void) {
    f32 a_buf[6] = {};
    f32 inv_buf[6] = {};

    matrix a, inv;
    mat_create(&a,   2, 3, a_buf);
    mat_create(&inv, 2, 3, inv_buf);

    TEST_ASSERT_FALSE(mat_inverse(&inv, &a));
}

// -----------------------------------------------------------------------
// mat_transpose
// -----------------------------------------------------------------------

// Transpose a 2x3 matrix and verify each element
void test_mat_transpose_2x3(void) {
    // A = [1 2 3; 4 5 6]  =>  A^T = [1 4; 2 5; 3 6]
    f32 a_buf[6], t_buf[6];
    matrix a, t;
    mat_create(&a, 2, 3, a_buf);
    mat_create(&t, 3, 2, t_buf);

    a.data[0]=1; a.data[1]=2; a.data[2]=3;
    a.data[3]=4; a.data[4]=5; a.data[5]=6;

    TEST_ASSERT_TRUE(mat_transpose(&t, &a));

    // Column-major check: t[row][col] = a[col][row]
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, t.data[0]); // t[0][0]
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 4.0f, t.data[1]); // t[0][1]
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 2.0f, t.data[2]); // t[1][0]
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0f, t.data[3]); // t[1][1]
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 3.0f, t.data[4]); // t[2][0]
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 6.0f, t.data[5]); // t[2][1]
}

// Transposing into a wrongly-sized output should fail
void test_mat_transpose_wrong_size(void) {
    f32 a_buf[6], t_buf[6];
    matrix a, t;
    mat_create(&a, 2, 3, a_buf);
    mat_create(&t, 2, 3, t_buf); // should be 3x2

    TEST_ASSERT_FALSE(mat_transpose(&t, &a));
}

// -----------------------------------------------------------------------
// mat_mul
// -----------------------------------------------------------------------

// A * B  (no transpose)
void test_mat_mul_nn(void) {
    // A = [1 2; 3 4],  B = [5 6; 7 8]
    // C = A*B = [1*5+2*7  1*6+2*8; 3*5+4*7  3*6+4*8] = [19 22; 43 50]
    f32 a_buf[4], b_buf[4], c_buf[4];
    matrix a, b, c;
    mat_create(&a, 2, 2, a_buf);
    mat_create(&b, 2, 2, b_buf);
    mat_create(&c, 2, 2, c_buf);

    a.data[0]=1; a.data[1]=2; a.data[2]=3; a.data[3]=4;
    b.data[0]=5; b.data[1]=6; b.data[2]=7; b.data[3]=8;

    TEST_ASSERT_TRUE(mat_mul(&c, &a, &b, true, false, false));

    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 19.0f, c.data[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 22.0f, c.data[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 43.0f, c.data[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 50.0f, c.data[3]);
}

// A * B^T
void test_mat_mul_nt(void) {
    // A = [1 2; 3 4],  B = [1 0; 0 1] (identity), B^T = B
    // C = A * I = A
    f32 a_buf[4], b_buf[4], c_buf[4];
    matrix a, b, c;
    mat_create(&a, 2, 2, a_buf);
    mat_create(&b, 2, 2, b_buf);
    mat_create(&c, 2, 2, c_buf);

    a.data[0]=1; a.data[1]=2; a.data[2]=3; a.data[3]=4;
    b.data[0]=1; b.data[1]=0; b.data[2]=0; b.data[3]=1; // identity

    TEST_ASSERT_TRUE(mat_mul(&c, &a, &b, true, false, true));

    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, c.data[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f, c.data[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.0f, c.data[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 4.0f, c.data[3]);
}

// A^T * B
void test_mat_mul_tn(void) {
    // A = [1 2; 3 4],  A^T = [1 3; 2 4]
    // B = [1; 1] (2x1 column vector)
    // C = A^T * B = [1*1+3*1; 2*1+4*1] = [4; 6]
    f32 a_buf[4], b_buf[2], c_buf[2];
    matrix a, b, c;
    mat_create(&a, 2, 2, a_buf);
    mat_create(&b, 2, 1, b_buf);
    mat_create(&c, 2, 1, c_buf);

    a.data[0]=1; a.data[1]=2; a.data[2]=3; a.data[3]=4;
    b.data[0]=1; b.data[1]=1;

    TEST_ASSERT_TRUE(mat_mul(&c, &a, &b, true, true, false));

    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 4.0f, c.data[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 6.0f, c.data[1]);
}

// A^T * B^T
void test_mat_mul_tt(void) {
    // A = [1 0; 0 1] (identity),  B = [2 3; 4 5]
    // A^T = I,  B^T = [2 4; 3 5]
    // C = I * B^T = B^T = [2 4; 3 5]
    f32 a_buf[4], b_buf[4], c_buf[4];
    matrix a, b, c;
    mat_create(&a, 2, 2, a_buf);
    mat_create(&b, 2, 2, b_buf);
    mat_create(&c, 2, 2, c_buf);

    a.data[0]=1; a.data[1]=0; a.data[2]=0; a.data[3]=1; // identity
    b.data[0]=2; b.data[1]=3; b.data[2]=4; b.data[3]=5;

    TEST_ASSERT_TRUE(mat_mul(&c, &a, &b, true, true, true));

    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f, c.data[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 4.0f, c.data[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.0f, c.data[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.0f, c.data[3]);
}

// Incompatible dimensions should return false
void test_mat_mul_dimension_mismatch(void) {
    f32 a_buf[6], b_buf[6], c_buf[4];
    matrix a, b, c;
    mat_create(&a, 2, 3, a_buf); // 2x3
    mat_create(&b, 2, 3, b_buf); // 2x3  (inner dims 3 != 2)
    mat_create(&c, 2, 2, c_buf);

    TEST_ASSERT_FALSE(mat_mul(&c, &a, &b, true, false, false));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_mat_inverse_2x2);
    RUN_TEST(test_mat_inverse_singular);
    RUN_TEST(test_mat_inverse_non_square);
    RUN_TEST(test_mat_transpose_2x3);
    RUN_TEST(test_mat_transpose_wrong_size);
    RUN_TEST(test_mat_mul_nn);
    RUN_TEST(test_mat_mul_nt);
    RUN_TEST(test_mat_mul_tn);
    RUN_TEST(test_mat_mul_tt);
    RUN_TEST(test_mat_mul_dimension_mismatch);
    return UNITY_END();
}
