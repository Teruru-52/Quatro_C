#ifndef _FLASH_MEMORY_H_
#define _FLASH_MEMORY_H_
#include "main.h"

void eraseFlash();
void writeFlash(uint32_t address, uint8_t *data, uint32_t size);
void loadFlash(uint32_t address, uint8_t *data, uint32_t size);

#endif // _FLASH_MEMORY_H_