#include "../include/mpu_6050.h"
#include "../include/i2c.h"
#include "driver/i2c.h"

void mpu6050_init(void) {
    uint8_t wake_buf[2] = {MPU6050_PWR_MGMT_1, 0x00};
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, wake_buf, sizeof(wake_buf), pdMS_TO_TICKS(100));
}