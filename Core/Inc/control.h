#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "main.h"
#include "gyro.h"

#define YAW_PID_KP  20
#define YAW_PID_KI  0
#define YAW_PID_KD  0

#define GYRO_PID_KP  20
#define GYRO_PID_KI  0
#define GYRO_PID_KD  0

#define PID_SAMPLING_TIME   0.001f
#define D_FILTER_COFF       0.025f

struct Gyro_Typedef;

typedef struct
{
    float ts; //sampling time
    float kp1, ki1, kd1, kp2, ki2, kd2;
    float ref, ref2;
    float input;
} Control_Typedef;

void PIDControlInit(Control_Typedef *pid);
void AngleControl(Gyro_Typedef *gyro, Control_Typedef *pid);
void AngularVelocityControl(Gyro_Typedef *gyro, Control_Typedef *pid);

#endif // _CONTROL_H_
