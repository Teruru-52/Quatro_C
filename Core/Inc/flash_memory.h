#ifndef _FLASH_MEMORY_H_
#define _FLASH_MEMORY_H_
#include "main.h"

typedef struct{
    float output[3000], outputL[3000], outputR[3000];
    int input[3000];
}Data_Typedef;

void eraseFlash();
void writeFlash(uint32_t address, uint8_t *data, uint32_t size);
void loadFlash(uint32_t address, uint8_t *data, uint32_t size);

#endif // _FLASH_MEMORY_H_