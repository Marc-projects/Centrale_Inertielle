#ifndef COLLECT_COMPUTE_H
#define COLLECT_COMPUTE_H

void collect_data(float* collected_values);

void compute_data(quaternion* q, float* values, uint32_t dt);

void task_collect_compute(void* pvParameters);

#endif
