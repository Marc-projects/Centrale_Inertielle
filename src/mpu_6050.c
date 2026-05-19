#include "../include/mpu_6050.h"
#include "../include/i2c.h"
#include "driver/i2c_master.h"

void mpu6050_init(i2c_master_dev_handle_t device) {
    uint8_t wake_buf[2] = {MPU6050_PWR_MGMT_1, 0x00};
    i2c_master_transmit(device, wake_buf, sizeof(wake_buf), 100);
}
