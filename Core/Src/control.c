#include "main.h"
#include "tim.h"

static float pre_error = 0.0;
static float sum_error = 0.0;
static float angular_vel_ref;

static float pre_error2 = 0.0;
static float sum_error2 = 0.0;

void AngleControl(Gyro_Typedef *gyro){
  float error = YAW_REF - gyro->yaw;
  sum_error += error;
  angular_vel_ref = YAW_PID_KP * error + YAW_PID_KI * sum_error * 0.01 + YAW_PID_KD * (pre_error - error) * 100.0;
  pre_error = error;
}

void AngularVelocityControl(Gyro_Typedef *gyro){
  // AngleControl(Gyro_Typedef *gyro);
  float error2 = angular_vel_ref - gyro->gz;
  sum_error2 += error2;
  int u = GYRO_PID_KP * error2 + GYRO_PID_KI * sum_error2 * 0.01 + GYRO_PID_KD * (pre_error2 - error2) * 100.0;
  if(u > 0){
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 150+u);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 150+u);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
  }
  else{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, -u+150);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, -u+150);
  }
  pre_error2 = error2;
}