#ifndef I2C_H
#define I2C_H

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000

#include "main.h"

void i2c_master_init(i2c_master_bus_handle_t* bus_handle, i2c_master_dev_handle_t* mpu6050_dev_handle);

#endif