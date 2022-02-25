#include "fan_motor.h"
#include "tim.h"

void FanMotorDrive(){
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, FAN_INPUT);
}

void FanMotorBrake(){
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
}