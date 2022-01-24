#ifndef _BATTERY_H_
#define _BATTERY_H_
#include "main.h"

typedef struct
{
    float bat_vol;
} Battery_Typedef;

void BatteryCheckOn(Battery_Typedef *battery);
void BatteryCheckOff();

#endif // _BATTERY_H_