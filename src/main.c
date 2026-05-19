#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "../lib/quaternion/quaternion.h"
#include "../include/collect_compute.h"
#include "../include/format_send.h"
#include "../include/i2c.h"
#include "../include/mpu_6050.h"
#include "../include/main.h"


void app_main(void) {
    static handle_structure handles;
    QueueHandle_t data_queue = xQueueCreate( 1, sizeof( quaternion ) );
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t mpu6050_dev_handle;

    i2c_master_init(&bus_handle, &mpu6050_dev_handle);
    mpu6050_init(mpu6050_dev_handle);

    handles.bus = bus_handle;
    handles.device = mpu6050_dev_handle;
    handles.queue = data_queue;
    
    xTaskCreate(task_collect_compute, "task_collect_compute", 3072, (void*)&handles, 23, NULL);
    xTaskCreate(task_format_send, "task_format_send", 3072, (void*)data_queue, 22, NULL);
}