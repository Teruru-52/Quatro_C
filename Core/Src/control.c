#include "control.h"

static float pre_error;
static float sum_error = 0.0;
static float pre_error2;
static float sum_error2 = 0.0;
static float pre_deriv2;
static float pre_error3;
static float sum_error3 = 0.0;
static float pre_deriv3;

extern int count_idnt;

// static float dt_recip;  // 1/sampling time

void PIDControlInit(Control_Typedef *pid){
  pid->ts = PID_SAMPLING_TIME;
  // Angle Control
  pid->kp1 = YAW_PID_KP;
  pid->ki1 = YAW_PID_KI;
  pid->kd1 = YAW_PID_KD;
  pid->ref = 0.0;
  // Angular Velocity Control
  pid->kp2 = GYRO_PID_KP;
  pid->ki2 = GYRO_PID_KI;
  pid->kd2 = GYRO_PID_KD;
  pid->ref2 = 0.0;
  pid->u_ang = 0.0;
  // Velocity Control
  pid->kp3 = VEL_PID_KP;
  pid->ki3 = VEL_PID_KI;
  pid->kd3 = VEL_PID_KD;
  pid->ref3 = 0.0;
  pid->u_vel = 0.0;

  pid->vel = 0.0;
  pid->u_pid = 0.0;
}

void AngleControl(Gyro_Typedef *gyro, Control_Typedef *pid){
  float error, deriv;
  error = (pid->ref - gyro->yaw) * M_PI / 180;
  sum_error += error*pid->ts;
  deriv = (error - pre_error)/pid->ts;
  pid->ref2 = pid->kp1*error + pid->ki1*sum_error + pid->kd1*deriv;
  
  pre_error = error;
}

void AngularVelocityControl(Gyro_Typedef *gyro, Control_Typedef *pid){
  float error2, deriv2;
  error2 = (pid->ref2 - gyro->gz) * M_PI / 180;
  sum_error2 += error2*pid->ts;
  deriv2 = (error2 - pre_error2)/pid->ts;
  deriv2 = pre_deriv2 + (deriv2 - pre_deriv2)*D_FILTER_COFF;
  pid->u_ang = pid->kp2*error2 + pid->ki2*sum_error2 + pid->kd2*deriv2;

  pre_error2 = error2;
  pre_deriv2 = deriv2;
}

void VelocityControl(Encoder_Typedef *encoder, Control_Typedef *pid){
  float error3, deriv3;
  pid->vel = (encoder->countL + encoder->countR)/2;
  error3 = (pid->ref3 - pid->vel);
  sum_error3 += error3*pid->ts;
  deriv3 = (error3 - pre_error3)/pid->ts;
  deriv3 = pre_deriv3 + (deriv3 - pre_deriv3)*D_FILTER_COFF;
  pid->u_vel = pid->kp3*error3 + pid->ki3*sum_error3 + pid->kd3*deriv3;

  pre_error3 = error3;
  pre_deriv3 = deriv3;
}

void PIDControl(Control_Typedef *pid){
  pid->u_pid = pid->u_ang + pid->u_vel;

  if (pid->u_pid >= MAX_INPUT)
    pid->u_pid = MAX_INPUT;
  if (pid->u_pid <= -MAX_INPUT)
    pid->u_pid = -MAX_INPUT;

  if (pid->u_pid > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT - pid->u_pid);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT - pid->u_pid);
  }
  else{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT + pid->u_pid);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT + pid->u_pid);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT);
  }
}

void RotationControl(Battery_Typedef *battery, Data_Typedef *data, Gyro_Typedef *gyro)
{
  int u_iden = (int)(1023.0 / battery->bat_vol * 1.5); // u_iden = 1.5 [V]
  data->output[count_idnt] = gyro->gz;
  data->input[count_idnt] = u_iden;

  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT - u_iden);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT - u_iden);
}

void MotorStop(){
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
}