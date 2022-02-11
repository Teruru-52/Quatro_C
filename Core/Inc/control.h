#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "main.h"
#include "gyro.h"
#include "encoder.h"
#include "battery.h"
#include "flash_memory.h"

#define YAW_PID_KP   30
#define YAW_PID_KI   10
#define YAW_PID_KD   1.0

#define GYRO_PID_KP  0.639
#define GYRO_PID_KI  15.2
#define GYRO_PID_KD  0.0

#define VEL_PID_KP  0.166
#define VEL_PID_KI  0.42
// #define VEL_PID_KD  0.0163
#define VEL_PID_KD  0.0

#define MAX_INPUT 1000.0

#define PID_SAMPLING_TIME   0.001f
#define D_FILTER_COFF       0.025f //3.98Hz

struct Gyro_Typedef;
struct Encoder_Typedef;
struct Battery_Typedef;
struct Data_Typedef;

typedef struct
{
    float ts; //sampling time
    float kp1, ki1, kd1, kp2, ki2, kd2;
    float kp3, ki3, kd3;
    float ref, ref2, ref3;
    float u_ang, u_vel;
    int u_pid_left, u_pid_right;
} Control_Typedef;

typedef struct{
    float m_sequence[128];
} MSequence_Typedef;

void PIDControlInit(Control_Typedef *pid);
void AngleControl(Gyro_Typedef *gyro, Control_Typedef *pid);
void AngularVelocityControl(Gyro_Typedef *gyro, Control_Typedef *pid);
void VelocityControl(Encoder_Typedef *encoder, Control_Typedef *pid);
void PIDControl(Control_Typedef *pid, Battery_Typedef *battery);
void TranslationControl(Battery_Typedef *battery, Data_Typedef *data, Encoder_Typedef *encoder);
void MotorStop();
void MSequenceGen(MSequence_Typedef *msequence);
void MSequenceInput(Data_Typedef *data, Gyro_Typedef *gyro, Battery_Typedef *battery, MSequence_Typedef *msequence);

#endif // _CONTROL_H_
