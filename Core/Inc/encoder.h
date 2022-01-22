#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "main.h"

typedef struct{
    int16_t countL, countR;
}Encoder_Typedef;

void GetEncoderL(Encoder_Typedef *encoder);
void GetEncoderR(Encoder_Typedef *encoder);

#endif // _ENCODER_H_