#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "main.h"
#include "gyro.h"
#include "encoder.h"
#include "battery.h"

#define YAW_PID_KP   30
#define YAW_PID_KI   10
#define YAW_PID_KD   1.0

// #define GYRO_PID_KP  2.06
// #define GYRO_PID_KI  0.0285
// #define GYRO_PID_KD  37.3

#define GYRO_PID_KP  1.28
// #define GYRO_PID_KP  100
#define GYRO_PID_KI  0.0153
#define GYRO_PID_KD  22

#define VEL_PID_KP  1
#define VEL_PID_KI  0
#define VEL_PID_KD  0

#define MAX_INPUT 999

#define PID_SAMPLING_TIME   0.001f
#define D_FILTER_COFF       0.025f //3.98Hz

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
