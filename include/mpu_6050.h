#ifndef MPU_6050_H
#define MPU_6050_H

#define MPU6050_ADDR                0x68
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_ACCEL_XOUT_H        0x3B

void mpu6050_init(void);

#endif