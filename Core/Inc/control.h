#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "main.h"
#include "gyro.h"
#include "encoder.h"

#define YAW_PID_KP   20
#define YAW_PID_KI   0.1
#define YAW_PID_KD   1.0

#define GYRO_PID_KP  250
#define GYRO_PID_KI  0
#define GYRO_PID_KD  0

#define VEL_PID_KP  1
#define VEL_PID_KI  0
#define VEL_PID_KD  0

#define PID_SAMPLING_TIME   0.001f
#define D_FILTER_COFF       0.025f //3.98Hz

#define STANDARD_INPUT 50

struct Gyro_Typedef;
struct Encoder_Typedef;

typedef struct
{
    float ts; //sampling time
    float kp1, ki1, kd1, kp2, ki2, kd2;
    float kp3, ki3, kd3;
    float ref, ref2, ref3;
    int input, input_vel, input_pid;
} Control_Typedef;

void PIDControlInit(Control_Typedef *pid);
void AngleControl(Gyro_Typedef *gyro, Control_Typedef *pid);
void AngularVelocityControl(Gyro_Typedef *gyro, Control_Typedef *pid);
void VelocityControl(Encoder_Typedef *encoder, Control_Typedef *pid);
void PIDControl(Control_Typedef *pid);

#endif // _CONTROL_H_
