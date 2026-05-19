#ifndef FORMAT_SEND_H
#define FORMAT_SEND_H

#include "../lib/quaternion/quaternion.h"

void format_data(quaternion* q, axis_angle* aa);

void send_data(axis_angle* aa);

void task_format_send(void* pvParameters);

#endif
