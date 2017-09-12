#include <stdio.h>
#include <linux/i2c-dev.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include "mpu6050.h"

int initIMU(char *fileName, int address) {
    int fd;
    if ((fd = open(fileName, O_RDWR)) < 0) {
        printf("Failed to open i2c port\n");
        exit(1);
    }
	
    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        printf("Unable to get bus access to talk to slave\n");
        exit(1);
    }

    int8_t power = i2c_smbus_read_byte_data(fd, MPU_POWER1);
    i2c_smbus_write_byte_data(fd, MPU_POWER1, ~(1 << 6) & power);

    return fd;
}

void readIMU(int fd, int16_t* output) {
        output[0] = i2c_smbus_read_byte_data(fd, MPU_TEMP1) << 8 |
                        i2c_smbus_read_byte_data(fd, MPU_TEMP2);

        output[1] = i2c_smbus_read_byte_data(fd, MPU_ACCEL_XOUT1) << 8 |
                    i2c_smbus_read_byte_data(fd, MPU_ACCEL_XOUT2);
        output[2] = i2c_smbus_read_byte_data(fd, MPU_ACCEL_YOUT1) << 8 |
                    i2c_smbus_read_byte_data(fd, MPU_ACCEL_YOUT2);
        output[3] = i2c_smbus_read_byte_data(fd, MPU_ACCEL_ZOUT1) << 8 |
                    i2c_smbus_read_byte_data(fd, MPU_ACCEL_ZOUT2);

        output[4] = i2c_smbus_read_byte_data(fd, MPU_GYRO_XOUT1) << 8 |
                    i2c_smbus_read_byte_data(fd, MPU_GYRO_XOUT2);
        output[5] = i2c_smbus_read_byte_data(fd, MPU_GYRO_YOUT1) << 8 |
                    i2c_smbus_read_byte_data(fd, MPU_GYRO_YOUT2);
        output[6] = i2c_smbus_read_byte_data(fd, MPU_GYRO_ZOUT1) << 8 |
                    i2c_smbus_read_byte_data(fd, MPU_GYRO_ZOUT2);
}

