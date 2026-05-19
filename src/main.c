#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "../lib/quaternion/quaternion.h"
#include "../include/collect_compute.h"
#include "../include/format_send.h"
#include "../include/i2c.h"
#include "../include/mpu_6050.h"


void app_main(void) {
    QueueHandle_t data_queue = xQueueCreate( 1, sizeof( quaternion ) );
    i2c_master_init();
    xTaskCreate(task_collect_compute, "task_collect_compute", 3072, (void*)data_queue, 23, NULL);
    xTaskCreate(task_format_send, "task_format_send", 3072, (void*)data_queue, 22, NULL);
    mpu6050_init();
}