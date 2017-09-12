#include "mpu6050.h"

int main(int argc, char **argv)
{
    int fd2;
    int fd1;

    int16_t data1[7];
    int16_t data2[7];

    fd1 = initIMU("/dev/i2c-1", 0x68);
    fd2 = initIMU("/dev/i2c-1", 0x69);

    while (1) {
        printf("=============\n");

        readIMU(fd1, data1);
        printf("temp1: %f\n", (float)data1[0] / 340.0f + 36.53);
        printf("accel1 x,y,z: %d, %d, %d\n", (int)data1[1], (int)data1[2], (int)data1[3]);
        printf("gyro1 x,y,z: %d, %d, %d\n\n", (int)data1[4], (int)data1[5], (int)data1[6]);

        printf("temp2: %f\n", (float)data2[0] / 340.0f + 36.53);
        printf("accel2 x,y,z: %d, %d, %d\n", (int)data2[1], (int)data2[2], (int)data2[3]);
        printf("gyro2 x,y,z: %d, %d, %d\n\n", (int)data2[4], (int)data2[5], (int)data2[6]);
        readIMU(fd2, data2);
    }

    return 0;
}

