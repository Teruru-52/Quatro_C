#ifndef _GYRO_H_
#define _GYRO_H_
#include <stdint.h>
// #include "main.h" // これを記述するとエラー

#define ADDRESS           0x68
#define WHO_AM_I          0x75

#define PWR_MGMT_1        0x6B
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C

#define ACCEL_XOUT_H      0x3B
#define ACCEL_XOUT_L      0x3C
#define ACCEL_YOUT_H      0x3D
#define ACCEL_YOUT_L      0x3E

#define GYRO_ZOUT_H       0x47
#define GYRO_ZOUT_L       0x48

typedef struct
{
    float gz, yaw;
} Gyro_Typedef;

uint8_t read_byte(uint8_t reg);
void write_byte(uint8_t reg, uint8_t data);
void GyroInit();
void GyroOffsetCalc();
void GetGyroZ(Gyro_Typedef *gyro);
void GetYaw(Gyro_Typedef *gyro);

#endif // _GYRO_H_