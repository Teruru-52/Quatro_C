#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "main.h"

#define YAW_PID_KP 20.0f
#define YAW_PID_KI 0.0f
#define YAW_PID_KD 0.0

#define GYRO_PID_KP 3.75f
#define GYRO_PID_KI 15.7f
// #define GYRO_PID_KD 0.213f
#define GYRO_PID_KD 0.0f
#define D_FILTER_COFF 0.383f

#define VEL_PID_KP 0.166f
#define VEL_PID_KI 0.42f
// #define VEL_PID_KD  0.0163
#define VEL_PID_KD 0.0f

#define MAX_INPUT 1000.0f

#define CONTROL_PERIOD 0.01f

#define IR_KP_LEFT 0.5f
#define IR_KI_LEFT 0.1f
#define IR_KP_RIGHT 0.5f
#define IR_KI_RIGHT 0.1f
#define IR_THR_LEFT 3300.0f
#define IR_THR_RIGHT 3000.0f

extern uint32_t ir_fl, ir_fr, ir_bl, ir_br;
extern float yaw, gz;
extern float velocity;
extern float velocityR;
extern float bat_vol;
extern int flag_turn;
extern int flag_sensor;

typedef struct
{
    float kp, ki, kd;
    float ref;
} Control_Typedef;

void PIDControlInit(Control_Typedef *pid1, Control_Typedef *pid2, Control_Typedef *pid3);
void SetReference(Control_Typedef *pid1, float ref_ang);
float AngleControl(Control_Typedef *pid1);
float AngularVelocityControl(Control_Typedef *pid2);
float VelocityControl(Control_Typedef *pid3);
void PositionControl();
void PartyTrick();
void GoStraight();
void Turn();
void DetectFrontWall();
void FrontWallCorrection();
void Back();
void MotorStop();

#endif // _CONTROL_H_
