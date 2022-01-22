#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "main.h"
// #include "gyro.h"

#define YAW_PID_KP  20
#define YAW_PID_KI  0
#define YAW_PID_KD  0

#define GYRO_PID_KP  20
#define GYRO_PID_KI  0
#define GYRO_PID_KD  0

#define YAW_REF 0

// typedef struct
// {
//     float pre_error, pre_error2;
//     float sum_error, sum_error2;
//     float angular_vel_ref;
// } Control_Typedef;

// void PIDReset(Control_Typedef *pid);
void AngleControl(Gyro_Typedef *gyro);
void AngularVelocityControl(Gyro_Typedef *gyro);

#endif // _CONTROL_H_