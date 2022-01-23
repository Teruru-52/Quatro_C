#ifndef _GYRO_H_
#define _GYRO_H_
#include "main.h"

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
  float offset, gz, yaw;
} Gyro_Typedef;

typedef struct
{
  float a1, a2, b0, b1, b2;
} IIR_Coeff;

//IIR filter
//7hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.922286512869545,  -0.92519529534950118, 0.00072719561998898304, 0.0014543912399779661, 0.00072719561998898304};

//15hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.8337326589246479,  -0.84653197479202391, 0.003199828966843966, 0.0063996579336879321, 0.003199828966843966};

//30hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.66920314293119312,  -0.71663387350415764, 0.011857682643241156, 0.023715365286482312, 0.011857682643241156};

//60hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.3489677452527946 ,  -0.51398189421967566, 0.041253537241720303, 0.082507074483440607, 0.041253537241720303};

//100hz, 800hz
static IIR_Coeff gyro_fil_coeff = {0.94280904158206336,  -0.33333333333333343, 0.09763107293781749 , 0.19526214587563498 , 0.09763107293781749};

uint8_t read_byte(uint8_t reg);
void write_byte(uint8_t reg, uint8_t data);

void IIRInit();
void GyroInit();
void GyroOffsetCalc(Gyro_Typedef *gyro);
void GetGyroData(Gyro_Typedef *gyro);

#endif // _GYRO_H_
