#include "../include/i2c.h"
#include "../include/mpu_6050.h"
#include "driver/i2c_master.h"
#include "../include/main.h"

void i2c_master_init(i2c_master_bus_handle_t* bus_handle, i2c_master_dev_handle_t* mpu6050_dev_handle) {
    // 1. Configuration du Bus
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, // Remplace GPIO_PULLUP_ENABLE
    };
    
    // Allocation du bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    // 2. Configuration du Périphérique (MPU6050)
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    // Ajout du MPU6050 au bus configuré
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, mpu6050_dev_handle));
}