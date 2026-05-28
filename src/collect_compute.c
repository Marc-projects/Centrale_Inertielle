#include "../include/collect_compute.h"
#include "../include/mpu_6050.h"
#include "../include/i2c.h"
#include "../lib/quaternion/quaternion.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../include/main.h"

void collect_data(float* collected_values, i2c_master_dev_handle_t device) {
    const uint8_t reg_addr = MPU6050_ACCEL_XOUT_H;
    uint8_t collected_data[14];
    esp_err_t ret = i2c_master_transmit_receive(
        device, 
        &reg_addr,            // Buffer d'écriture (ce que l'on veut lire)
        1,                    // Taille de l'écriture
        collected_data,       // Buffer de réception
        14,                   // Taille de la lecture (14 octets pour le MPU6050)
        100                   // Timeout en millisecondes
    );
    
    if (ret == ESP_OK) {
        collected_values[0] = ((int16_t)((collected_data[0] << 8) | collected_data[1]))/16384.0f; //accel_x
        collected_values[1] = ((int16_t)((collected_data[2] << 8) | collected_data[3]))/16384.0f; //accel_y
        collected_values[2] = ((int16_t)((collected_data[4] << 8) | collected_data[5]))/16384.0f; //accel_z
        
        collected_values[3]  = ((int16_t)((collected_data[8] << 8) | collected_data[9]))/7509.9f; //gyro_x
        collected_values[4]  = ((int16_t)((collected_data[10] << 8) | collected_data[11]))/7509.9f; //gyro_y
        collected_values[5]  = ((int16_t)((collected_data[12] << 8) | collected_data[13]))/7509.9f; //gyro_z
    }
}

void init_bias(float* bias, i2c_master_dev_handle_t device, TickType_t* lastWakeTime, TickType_t period) {
    int init_count, init_length = NBR_INIT_CYCLE, i;
    float data[6];

    for (init_count = 0; init_count < init_length; init_count++) {
        collect_data(data, device);
        
        for (i = 0; i < 6; i++) {
            if(i == 2) {
                data[i] -= 1.0f;
            }

            bias[i] += data[i];
        }
        vTaskDelayUntil(lastWakeTime, period);
    }

    for (i = 0; i < 6; i++) {
        bias[i] /= (float)init_length;
    }
}

void compute_data(quaternion* q, float* values, uint8_t dt, float* bias) {
    quaternion q_rotation_speed = {0.0f, values[3] - bias[3], values[4] - bias[4], values[5] - bias[5]},
                q_acceleration = {0.0f, values[0] - bias[0], values[1] - bias[1], values[2] - bias[2]},                      
                q_temp, q_gradient = {0.0f, 0.0f, 0.0f, 0.0f};
    static int compteur = 0;

    compteur++;
    if (compteur == 5) {
        compteur = 0;
        float q_norm = quaternion_norm(&q_acceleration);
        if (1.0f - ACCEL_GRAVITY_MARGIN <= q_norm && q_norm <= 1.0f + ACCEL_GRAVITY_MARGIN) {
            compute_gradient_descent_correction(q, &q_acceleration, &q_gradient);
            quaternion_scalar_product(&q_gradient, -GRADIENT_DESCENT_STEP, &q_gradient);
        }
    }

    quaternion_product(q, &q_rotation_speed, &q_temp);
    quaternion_scalar_product(&q_temp, 1.0f/2.0f, &q_temp);
    quaternion_addition(&q_temp, &q_gradient, &q_temp);
    quaternion_scalar_product(&q_temp, (float)dt * 1e-3, &q_temp);
    quaternion_addition(q, &q_temp, q);
    quaternion_normalize(q, q);
}

void task_collect_compute(void* pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(TIME_PERIOD_CONSTANT_MS);
    TickType_t lastWakeTime = xTaskGetTickCount();
    const uint8_t dt = TIME_PERIOD_CONSTANT_MS;
    float values[6], bias[6];
    quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};
    QueueHandle_t queue = ((handle_structure*)pvParameters)->queue;
    i2c_master_dev_handle_t device = ((handle_structure*)pvParameters)->device;

    init_bias(bias, device, &lastWakeTime, period);

    while(1) {
        collect_data(values, device);
        
        compute_data(&q, values, dt, bias);
        xQueueOverwrite( queue, &q );
        
        vTaskDelayUntil(&lastWakeTime, period);
    }
}
