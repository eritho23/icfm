#include "quaternion_utils.h"

void quat_normalise(quat *q) {
    f32 n = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    if (n < 1e-9f) { q->w=1.0f; q->x=q->y=q->z=0.0f; return; }
    q->w/=n; q->x/=n; q->y/=n; q->z/=n;
}

void quat_mul(quat *out, const quat *a, const quat *b) {
    out->w = a->w*b->w - a->x*b->x - a->y*b->y - a->z*b->z;
    out->x = a->w*b->x + a->x*b->w + a->y*b->z - a->z*b->y;
    out->y = a->w*b->y - a->x*b->z + a->y*b->w + a->z*b->x;
    out->z = a->w*b->z + a->x*b->y - a->y*b->x + a->z*b->w;
}

void quat_conjugate(quat *out, const quat *q) {
    out->w =  q->w;
    out->x = -q->x;
    out->y = -q->y;
    out->z = -q->z;
}

// Rodrigues sandwich product: out = q * [0,v] * q_conj
// Expanded inline to avoid allocating temporaries.
void quat_rotate(const quat *q, const f32 v[3], f32 out[3]) {
    const f32 qw=q->w, qx=q->x, qy=q->y, qz=q->z;
    const f32 tx = 2.0f*(qy*v[2] - qz*v[1]);
    const f32 ty = 2.0f*(qz*v[0] - qx*v[2]);
    const f32 tz = 2.0f*(qx*v[1] - qy*v[0]);
    out[0] = v[0] + qw*tx + qy*tz - qz*ty;
    out[1] = v[1] + qw*ty + qz*tx - qx*tz;
    out[2] = v[2] + qw*tz + qx*ty - qy*tx;
}

void quat_rotate_inv(const quat *q, const f32 v[3], f32 out[3]) {
    quat qc;
    qc.w =  q->w; qc.x = -q->x; qc.y = -q->y; qc.z = -q->z;
    quat_rotate(&qc, v, out);
}

// q_dot = 0.5 * Omega(omega) * q
// Omega(omega) scalar-first:
//   |  0  -wx  -wy  -wz |
//   | wx    0   wz  -wy |
//   | wy  -wz    0   wx |
//   | wz   wy  -wx    0 |
void quat_propagate(quat *q, f32 wx, f32 wy, f32 wz, f32 dt) {
    const f32 s = 0.5f * dt;
    quat qn;
    qn.w = q->w + s*(-wx*q->x - wy*q->y - wz*q->z);
    qn.x = q->x + s*( wx*q->w + wz*q->y - wy*q->z);
    qn.y = q->y + s*( wy*q->w - wz*q->x + wx*q->z);
    qn.z = q->z + s*( wz*q->w + wy*q->x - wx*q->y);
    q->w=qn.w; q->x=qn.x; q->y=qn.y; q->z=qn.z;
    quat_normalise(q);
}

void quat_to_euler_deg(const quat *q, f32 *roll, f32 *pitch, f32 *yaw) {
    f32 sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    f32 cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    *roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / M_PI);

    f32 sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(90.0f, sinp);
    else
        *pitch = asinf(sinp) * (180.0f / M_PI);

    f32 siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    f32 cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    *yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI);
}
