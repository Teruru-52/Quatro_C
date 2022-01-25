#include "control.h"

static float pre_error;
static float sum_error = 0.0;
static float pre_error2;
static float sum_error2 = 0.0;
static float pre_deriv2;

// static float dt_recip;  // 1/sampling time

void PIDControlInit(Control_Typedef *pid){
  pid->ts = PID_SAMPLING_TIME;
  pid->kp1 = YAW_PID_KP;
  pid->ki1 = YAW_PID_KI;
  pid->kd1 = YAW_PID_KD;
  pid->kp2 = GYRO_PID_KP;
  pid->ki2 = GYRO_PID_KI;
  pid->kd2 = GYRO_PID_KD;
  pid->ref = 0.0;
  pid->ref2 = 0.0;
  pid->input = 0.0;
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
  pid->input = pid->kp2*error2 + pid->ki2*sum_error2 + pid->kd2*deriv2;

  if(pid->input > 0){
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, STANDARD_INPUT+pid->input);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, STANDARD_INPUT+pid->input);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
  }
  else{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, STANDARD_INPUT-pid->input);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, STANDARD_INPUT-pid->input);
  }
  pre_error2 = error2;
  pre_deriv2 = deriv2;
}
