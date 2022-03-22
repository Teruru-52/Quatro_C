#include "function.h"

void StartUp(){
    Speaker();
    ReadFrontIRSensor();
    BatteryCheckOn();
    IR_PWM_Start();
}

void ModeSelect(){
    if (!flag_int)
    {
        EncoderCount();
        sw_cnt = sw_cnt % 16384;

        if (sw_cnt <= 4096)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
        }
        else if (sw_cnt <= 8192)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
        }
        else if (sw_cnt <= 12288)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
        }
        StartMovement();
    }
}

void StartMovement(){
    if (ir_bl > 2700 && ir_br > 2700)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

        BatteryCheckOff();
        GyroInit(); // who_am_i
        IIRInit();
        PIDControlInit(&pid_1, &pid_2, &pid_3);
        GyroOffsetCalc();

        main_mode = 1;
        flag_int = true;
    }
}

void Speaker(){
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 10);
    HAL_Delay(30);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
}

void ExecuteLogger(){
    // printf("%d \r\n", flag_mode);
    // printf("%f, %f \r\n", yaw, gz);
    // printf("%f, %f \r\n", u_left, u_right);
    // printf("%f \r\n", u_turn);
    // printf("%ld, %ld \r\n", ir_fl, ir_fr);
    // printf("%ld, %ld\r\n", ir_bl, ir_br);
    // printf("%d, %d\r\n", (int)(IR_KP_LEFT * (IR_THR_LEFT - (int)ir_bl)), (int)(IR_KP_RIGHT * (IR_THR_RIGHT - (int)ir_br)));
    // printf("%ld, %ld, %ld, %ld \r\n", ir_fl, ir_fr, ir_bl, ir_br);
    // printf("%f, %f \r\n", velocityL, velocityR);
    // printf("%f \r\n", bat_vol);
    // printf("%d \r\n", sw_cnt);
    // printf("%f, %f \r\n", gz_nonfil, gz);

}