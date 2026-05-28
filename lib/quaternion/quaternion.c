#include "quaternion.h"
#include <math.h>

void quaternion_product(quaternion* q1, quaternion* q2, quaternion* out) {
    float w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    float x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    float y = q1->w * q2->y + q1->y * q2->w + q1->z * q2->x - q1->x * q2->z;
    float z = q1->w * q2->z + q1->z * q2->w + q1->x * q2->y - q1->y * q2->x;

    out->w = w;
    out->x = x;
    out->y = y;
    out->z = z;
}

void quaternion_scalar_product(quaternion* q, float a, quaternion* out) {
    out->w = q->w * a;
    out->x = q->x * a;
    out->y = q->y * a;
    out->z = q->z * a;
}

void quaternion_addition(quaternion* q1, quaternion* q2, quaternion* out) {
    out->w = q1->w + q2->w;
    out->x = q1->x + q2->x;
    out->y = q1->y + q2->y;
    out->z = q1->z + q2->z;
}

void quaternion_subtraction(quaternion* q1, quaternion* q2, quaternion* out) {
    out->w = q1->w - q2->w;
    out->x = q1->x - q2->x;
    out->y = q1->y - q2->y;
    out->z = q1->z - q2->z;
}


float quaternion_norm(quaternion* q) {
    float norm_squared = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;

    return sqrtf(norm_squared);
}

void quaternion_normalize(quaternion* q, quaternion* out) {
    float norm_squared = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;

    if (norm_squared > 0.000001f) {
        float norm_inv = 1.0f / sqrt(norm_squared);

        out->w = q->w * norm_inv;
        out->x = q->x * norm_inv;
        out->y = q->y * norm_inv;
        out->z = q->z * norm_inv;
    } else {
        out->w = 1.0f;
        out->x = 0.0f;
        out->y = 0.0f;
        out->z = 0.0f;
    }
}

void quaternion_recover_axis_angle(quaternion* q, axis_angle* aa) {
    float norm, norm_inv;

    norm = sqrt(q->x * q->x + q->y * q->y + q->z * q->z);
    norm_inv = 1 / norm;
    
    aa->angle = 2 * atan2f(norm, q->w);
    aa->v1 = q->x * norm_inv;
    aa->v2 = q->y * norm_inv;
    aa->v3 = q->z * norm_inv;
}

void compute_gradient_descent_correction(quaternion* q, quaternion* q_acceleration, quaternion* out) {
    float norm = sqrtf(q_acceleration->x*q_acceleration->x + q_acceleration->y*q_acceleration->y + q_acceleration->z*q_acceleration->z);
    if (norm == 0.0f) return;
    float ax = q_acceleration->x / norm;
    float ay = q_acceleration->y / norm;
    float az = q_acceleration->z / norm;

    float _2q0 = 2.0f * q->w;
    float _2q1 = 2.0f * q->x;
    float _2q2 = 2.0f * q->y;
    float _2q3 = 2.0f * q->z;
    float _4q1 = 4.0f * q->x;
    float _4q2 = 4.0f * q->y;
    float _2q1q3 = 2.0f * q->x * q->z;
    float _2q0q2 = 2.0f * q->w * q->y;
    float _2q0q1 = 2.0f * q->w * q->x;
    float _2q2q3 = 2.0f * q->y * q->z;

    float fx = _2q1q3 - _2q0q2 - ax;
    float fy = _2q0q1 + _2q2q3 - ay;
    float fz = 1.0f - 2.0f * (q->x * q->x + q->y * q->y) - az;

    out->w = -_2q2 * fx + _2q1 * fy;
    out->x =  _2q3 * fx + _2q0 * fy - _4q1 * fz;
    out->y = -_2q0 * fx + _2q3 * fy - _4q2 * fz;
    out->z =  _2q1 * fx + _2q2 * fy;

    float out_norm = sqrtf(out->w*out->w + out->x*out->x + out->y*out->y + out->z*out->z);
    if (out_norm > 0.0f) {
        out->w /= out_norm; out->x /= out_norm; out->y /= out_norm; out->z /= out_norm;
    }
}
