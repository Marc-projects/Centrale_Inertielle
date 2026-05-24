#ifndef COLLECT_COMPUTE_H
#define COLLECT_COMPUTE_H

#include "../lib/quaternion/quaternion.h"
#include <stdint.h>
#include "main.h"

#define TIME_PERIOD_CONSTANT_MS 2
#define NBR_INIT_CYCLE 20

void collect_data(float* collected_values, i2c_master_dev_handle_t device);

void init_bias(float* bias, i2c_master_dev_handle_t device, TickType_t* lastWakeTime, TickType_t period);

void compute_data(quaternion* q, float* values, uint8_t dt, float* bias);

void task_collect_compute(void* pvParameters);

#endif
