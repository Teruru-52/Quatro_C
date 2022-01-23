#ifndef _IR_SENSOR_H_
#define _IR_SENSOR_H_
#include "main.h"

typedef struct
{
    uint32_t ir_fl, ir_fr, ir_bl, ir_br;
} IR_SENSOR_Typedef;

typedef struct
{
    float bat_vol;
} Battery_Typedef;

void IRPwmStart();
void ReadFrontIRSensor(IR_SENSOR_Typedef *sensor, Battery_Typedef *battery);
void ReadBackIRSensor(IR_SENSOR_Typedef *sensor);
void GetIRSensorData(IR_SENSOR_Typedef *sensor);

#endif // _IR_SENSOR_H_
