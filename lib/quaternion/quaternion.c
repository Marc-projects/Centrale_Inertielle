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
    float angle, v1, v2, v3, norm, norm_inv;

    norm = sqrt(q->x * q->x + q->y * q->y + q->z * q->z);
    norm_inv = 1 / norm;
    
    aa->angle = 2 * atan2f(norm, q->w);
    aa->v1 = q->x * norm_inv;
    aa->v2 = q->y * norm_inv;
    aa->v3 = q->z * norm_inv;
}
