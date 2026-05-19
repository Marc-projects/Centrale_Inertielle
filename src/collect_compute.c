#include "../include/collect_compute.h"
#include "../include/mpu_6050.h"
#include "../include/i2c.h"
#include "../lib/quaternion/quaternion.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void collect_data(float* collected_values) {
    const uint8_t reg_addr = MPU6050_ACCEL_XOUT_H;
    uint8_t collected_data[14];
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg_addr, 1, collected_data, 14, pdMS_TO_TICKS(100));
    
    if (ret == ESP_OK) {
        collected_values[0] = ((int16_t)((collected_data[0] << 8) | collected_data[1]))/16384.0f; //accel_x
        collected_values[1] = ((int16_t)((collected_data[2] << 8) | collected_data[3]))/16384.0f; //accel_y
        collected_values[2] = ((int16_t)((collected_data[4] << 8) | collected_data[5]))/16384.0f; //accel_z
        
        collected_values[3]  = ((int16_t)((collected_data[8] << 8) | collected_data[9]))/7509.9f - 0.005f; //gyro_x + biais
        collected_values[4]  = ((int16_t)((collected_data[10] << 8) | collected_data[11]))/7509.9f; //gyro_y
        collected_values[5]  = ((int16_t)((collected_data[12] << 8) | collected_data[13]))/7509.9f + 0.04f; //gyro_z + biais
    }
}

void compute_data(quaternion* q, float* values, uint32_t dt) {
    quaternion q_rotation_speed = {0.0f, values[3], values[4], values[5]}, q_temp;

    quaternion_product(q, &q_rotation_speed, &q_temp);
    quaternion_scalar_product(&q_temp, 1.0f/2.0f * (float)dt * 1e-6, &q_temp);
    quaternion_addition(q, &q_temp, q);
    quaternion_normalize(q, q);
}

void task_collect_compute(void* pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(1);
    TickType_t lastWakeTime = xTaskGetTickCount();
    uint32_t dt;
    float values[6];
    quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};
    
    while(1) {
        dt = 1000;
        collect_data(values);
        
        compute_data(&q, values, dt);
        xQueueOverwrite( (QueueHandle_t)pvParameters, &q );
        
        vTaskDelayUntil(&lastWakeTime, period);
    }
}
