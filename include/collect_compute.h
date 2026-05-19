#ifndef COLLECT_COMPUTE_H
#define COLLECT_COMPUTE_H

#include "../lib/quaternion/quaternion.h"
#include <stdint.h>

#define TIME_PERIOD_CONSTANT_MS 1

void collect_data(float* collected_values);

void compute_data(quaternion* q, float* values, uint8_t dt);

void task_collect_compute(void* pvParameters);

#endif
