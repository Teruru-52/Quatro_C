#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "main.h"
#include "gyro.h"
#include "encoder.h"
#include "ir_sensor.h"

#define YAW_PID_KP 30
#define YAW_PID_KI 10
#define YAW_PID_KD 1.0

#define GYRO_PID_KP 0.639
#define GYRO_PID_KI 15.2
#define GYRO_PID_KD 0.0

#define VEL_PID_KP 0.166
#define VEL_PID_KI 0.42
// #define VEL_PID_KD  0.0163
#define VEL_PID_KD 0.0

#define MAX_INPUT 1000.0

#define CONTROL_PERIOD 0.001f
#define D_FILTER_COFF 0.025f // 3.98Hz

typedef struct
{
    float kp, ki, kd;
    float ref;
} Control_Typedef;

extern float yaw, gz;
extern float velocity;
extern float bat_vol;

void PIDControlInit(Control_Typedef *pid1, Control_Typedef *pid2, Control_Typedef *pid3);
float AngleControl(Control_Typedef *pid1);
float AngularVelocityControl(Control_Typedef *pid2);
float VelocityControl(Control_Typedef *pid3);
void PartyTrick();
void GoStraight();
void MotorStop();

#endif // _CONTROL_H_
