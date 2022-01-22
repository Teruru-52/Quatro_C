#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1,(uint8_t *)ptr,len,10);
  return len;
}