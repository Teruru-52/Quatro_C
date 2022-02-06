#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "main.h"
#include "gyro.h"
#include "encoder.h"
#include "battery.h"

#define YAW_PID_KP   10
#define YAW_PID_KI   30
#define YAW_PID_KD   1.0

// #define GYRO_PID_KP  2.06
// #define GYRO_PID_KI  0.0285
// #define GYRO_PID_KD  37.3

// PID 1.5V
// #define GYRO_PID_KP  3.68
// #define GYRO_PID_KI  186
// #define GYRO_PID_KD  0.00446

// PID 2.0V 1次の極で求めてしまったもの
#define GYRO_PID_KP  0.639
#define GYRO_PID_KI  15.2
#define GYRO_PID_KD  0

// PID 2.0V
// #define GYRO_PID_KP  2.13
// #define GYRO_PID_KI  9.82
// #define GYRO_PID_KD  0.115

// PIDF 1.5V
// #define GYRO_PID_KP 1.15
// #define GYRO_PID_KI 0.0148
// #define GYRO_PID_KD 20.6

// PIDF 2.0V
// #define GYRO_PID_KP 2.15
// #define GYRO_PID_KI 9.81
// #define GYRO_PID_KD 0.104
// #define D_NUM 0.1604
// #define D_DEN 0.8396

#define VEL_PID_KP  1
#define VEL_PID_KI  0
#define VEL_PID_KD  0

#define MAX_INPUT 1000

#define PID_SAMPLING_TIME   0.001f
// #define D_FILTER_COFF       0.025f //3.98Hz

struct Gyro_Typedef;
struct Encoder_Typedef;
struct Battery_Typedef;

typedef struct
{
    float ts; //sampling time
    float kp1, ki1, kd1, kp2, ki2, kd2;
    float kp3, ki3, kd3;
    float ref, ref2, ref3;
    float vel;
    float u_ang, u_vel;
    int u_pid;
} Control_Typedef;

void PIDControlInit(Control_Typedef *pid);
void AngleControl(Gyro_Typedef *gyro, Control_Typedef *pid);
void AngularVelocityControl(Gyro_Typedef *gyro, Control_Typedef *pid);
void VelocityControl(Encoder_Typedef *encoder, Control_Typedef *pid);
void PIDControl(Control_Typedef *pid, Battery_Typedef *battery);

#endif // _CONTROL_H_
