#ifndef QUATERNION_H
#define QUATERNION_H

typedef struct quaternion {
    float w;
    float x;
    float y;
    float z;
} quaternion;

typedef struct axis_angle {
    float v1;
    float v2;
    float v3;
    float angle;
} axis_angle;

void quaternion_product(quaternion* q1, quaternion* q2, quaternion* out);

void quaternion_scalar_product(quaternion* q1, float a, quaternion* out);

void quaternion_addition(quaternion* q1, quaternion* q2, quaternion* out);

void quaternion_normalize(quaternion* q, quaternion* out);

void quaternion_recover_axis_angle(quaternion* q, axis_angle* aa);

#endif