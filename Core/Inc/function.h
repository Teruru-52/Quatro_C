#ifndef _FUNCTION_H_
#define _FUNCTION_H_
#include "main.h"
#include "battery.h"
#include "gyro.h"
#include "control.h"
#include "ir_sensor.h"
#include "encoder.h"

extern bool flag_int;
extern int main_mode;
extern Control_Typedef pid_1, pid_2, pid_3;
extern uint16_t sw_cnt;

void StartUp();
void ModeSelect();
void StartMovement();
void Speaker();
void ExecuteLogger();

#endif // _FUNCTION_H_