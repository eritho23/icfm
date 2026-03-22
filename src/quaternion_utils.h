#ifndef QUATERNION_UTILS_H
#define QUATERNION_UTILS_H

#include "base.h"

/*
    Quaternion convention: scalar-first [w, x, y, z]
    Unit quaternion represents a rotation.
    Hamilton product convention.
*/

typedef struct {
    f32 w, x, y, z;
} quat;

void quat_normalise(quat *q);
void quat_mul(quat *out, const quat *a, const quat *b);
void quat_conjugate(quat *out, const quat *q);
void quat_rotate(const quat *q, const f32 v[3], f32 out[3]);
void quat_rotate_inv(const quat *q, const f32 v[3], f32 out[3]);

// First-order quaternion kinematic propagation.
// Integrates q_dot = 0.5 * Omega(omega) * q over dt.
// wx/wy/wz: body-frame angular rates, rad/s.
void quat_propagate(quat *q, f32 wx, f32 wy, f32 wz, f32 dt);

#endif
