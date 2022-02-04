#ifndef _IR_SENSOR_H_
#define _IR_SENSOR_H_
#include "main.h"
#include "battery.h"

#define SAMPLING_COUNT 16

typedef struct
{
    uint32_t ir_fl, ir_fr, ir_bl, ir_br;
} IR_SENSOR_Typedef;

struct Battery_Typedef;

// typedef struct
// {
//     float w1, w2;
// } FFT_Coeff;

void IRPwmStart();
void ReadFrontIRSensor(IR_SENSOR_Typedef *sensor, Battery_Typedef *battery);
void ReadBackIRSensor(IR_SENSOR_Typedef *sensor);
void GetIRSensorData(IR_SENSOR_Typedef *sensor);

#endif // _IR_SENSOR_H_
