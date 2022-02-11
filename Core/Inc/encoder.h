#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "main.h"

typedef struct{
    int16_t countL, countR;
    float velocityL, velocityR, velocity;
}Encoder_Typedef;

void GetEncoderData(Encoder_Typedef *encoder);

#endif // _ENCODER_H_
