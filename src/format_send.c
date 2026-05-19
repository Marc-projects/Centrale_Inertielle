#include "../lib/quaternion/quaternion.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../include/format_send.h"

void format_data(quaternion* q, axis_angle* aa) {
    quaternion_recover_axis_angle(q, aa);
}

void send_data(axis_angle* aa) {
    printf("%.2f,%.2f,%.2f,%.2f\n", aa->angle, aa->v1, aa->v2, aa->v3);
}

void task_format_send(void* pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(1000 / 60);
    TickType_t lastWakeTime = xTaskGetTickCount();
    quaternion q_send;
    axis_angle axis_angle_recovered;

    while(1) {
        xQueuePeek( (QueueHandle_t)pvParameters, &q_send, portMAX_DELAY );

        format_data(&q_send, &axis_angle_recovered);

        send_data(&axis_angle_recovered);

        vTaskDelayUntil(&lastWakeTime, period);
    }
}
