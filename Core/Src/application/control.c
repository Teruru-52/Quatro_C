#include "control.h"

static float pre_error;
static float sum_error = 0.0;
static float pre_error2;
static float sum_error2 = 0.0;
static float pre_deriv2;
static float pre_error3;
static float sum_error3 = 0.0;
static float pre_deriv3;

Control_Typedef pid_1, pid_2, pid_3;

// static float dt_recip;  // 1/sampling time

void PIDControlInit(Control_Typedef *pid1, Control_Typedef *pid2, Control_Typedef *pid3)
{
  // Angle Control
  pid1->kp = YAW_PID_KP;
  pid1->ki = YAW_PID_KI;
  pid1->kd = YAW_PID_KD;
  pid1->ref = 0.0;
  // Angular Velocity Control
  pid2->kp = GYRO_PID_KP;
  pid2->ki = GYRO_PID_KI;
  pid2->kd = GYRO_PID_KD;
  pid2->ref = 0.0;
  // Velocity Control
  pid3->kp = VEL_PID_KP;
  pid3->ki = VEL_PID_KI;
  pid3->kd = VEL_PID_KD;
  pid3->ref = 100.0; // [rad/s]
}

void SetReference(Control_Typedef *pid1, float ref_ang)
{
  pid1->ref = ref_ang;
}

float AngleControl(Control_Typedef *pid1)
{
  float error, deriv, vel_ref;
  error = (pid1->ref - yaw) * M_PI / 180;
  sum_error += error * CONTROL_PERIOD;
  deriv = (pre_error - error) / CONTROL_PERIOD;
  vel_ref = pid1->kp * error + pid1->ki * sum_error + pid1->kd * deriv;

  pre_error = error;

  return vel_ref;
}

float AngularVelocityControl(Control_Typedef *pid2)
{
  float error2, deriv2, u_ang;
  pid2->ref = AngleControl(&pid_1);
  error2 = (pid2->ref - gz) * M_PI / 180;
  sum_error2 += error2 * CONTROL_PERIOD;
  deriv2 = (pre_error2 - error2) / CONTROL_PERIOD;
  // deriv2 = pre_deriv2 + (deriv2 - pre_deriv2) * D_FILTER_COFF;
  u_ang = pid2->kp * error2 + pid2->ki * sum_error2 + pid2->kd * deriv2;

  pre_error2 = error2;
  pre_deriv2 = deriv2;

  return u_ang;
}

float VelocityControl(Control_Typedef *pid3)
{
  float error3, deriv3, u_vel;
  error3 = (pid3->ref - velocity); // [rad/s]
  sum_error3 += error3 * CONTROL_PERIOD;
  deriv3 = (pre_error3 - error3) / CONTROL_PERIOD;
  // deriv3 = pre_deriv3 + (deriv3 - pre_deriv3)*D_FILTER_COFF;
  u_vel = pid3->kp * error3 + pid3->ki * sum_error3 + pid3->kd * deriv3;

  pre_error3 = error3;
  pre_deriv3 = deriv3;

  return u_vel;
}

void PartyTrick()
{
  float u, u_ang;
  u = AngularVelocityControl(&pid_2);

  u_ang = (int)(1000.0 / bat_vol * u);

  if (u_ang >= MAX_INPUT)
    u_ang = MAX_INPUT;
  if (u_ang <= -MAX_INPUT)
    u_ang = -MAX_INPUT;

  if (u_ang > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT - u_ang);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT - u_ang);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT + u_ang);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT + u_ang);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT);
  }
}

void GoStraight()
{
  float u_ang, u_vel;
  float u_left, u_right;
  u_ang = AngularVelocityControl(&pid_2);
  u_vel = VelocityControl(&pid_3);
  u_left = (int)(1000.0 / bat_vol * (u_vel - u_ang));
  u_right = (int)(1000.0 / bat_vol * (u_vel + u_ang));

  if (u_left >= MAX_INPUT)
    u_left = MAX_INPUT;
  else if (u_left <= -MAX_INPUT)
    u_left = -MAX_INPUT;

  if (u_right >= MAX_INPUT)
    u_right = MAX_INPUT;
  else if (u_right <= -MAX_INPUT)
    u_right = -MAX_INPUT;

  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT - u_left);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT - u_right);
}

void DetectFrontWall()
{
  if(ir_bl > 2500 && ir_br > 2500){
    MotorStop();
  }
}

void MotorStop()
{
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
}