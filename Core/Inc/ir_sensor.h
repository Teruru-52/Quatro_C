#ifndef _IR_SENSOR_H_
#define _IR_SENSOR_H_
#include "main.h"

#define SAMPLING_COUNT 16

uint32_t ir_fl, ir_fr, ir_bl, ir_br;

float bat_vol;

void IRPwmStart();
void ReadFrontIRSensor();
void ReadBackIRSensor();
void GetIRSensorData();

#endif // _IR_SENSOR_H_
