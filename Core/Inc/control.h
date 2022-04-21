#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "main.h"

// Party Trick
// #define YAW_PID_KP 2.0f
// #define YAW_PID_KI 4.0f
// #define YAW_PID_KD 0.5f

#define YAW_PID_KP 300.0f
#define YAW_PID_KI 100.0f
#define YAW_PID_KD 0.5f

// #define GYRO_PID_KP  18.16f
// #define GYRO_PID_KI  332.3f
#define GYRO_PID_KP  100.0f
#define GYRO_PID_KI  300.0f
#define GYRO_PID_KD  -0.072f
#define D_FILTER_COFF 0.967f

#define VEL_PID_KP 19.72f
#define VEL_PID_KI 52.0f
#define VEL_PID_KD 1.548f
#define D_FILTER_COFF2 0.301f

#define MAX_INPUT 1599.0f

#define CONTROL_PERIOD 0.001f

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
extern bool flag_int;
extern int main_mode;
extern int flag_mode;
extern int m_id;

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
void UpdateReference();
void TurnLeft(Control_Typedef *pid2);
void TurnRight(Control_Typedef *pid2);
void Uturn(Control_Typedef *pid2);
void DetectFrontWall();
void FrontWallCorrection();
void Step_Identification();
void M_Identification();
float M_Sequence(int i);
void Back();
void MotorStop();

#endif // _CONTROL_H_
