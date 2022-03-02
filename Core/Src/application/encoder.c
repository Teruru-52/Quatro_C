#include "encoder.h"
#include "tim.h"
#include <stdint.h>

float velocityL, velocityR, velocity;
static const float gear_ratio = 11.0 / 43.0;

void GetEncoderData()
{
  int16_t countl = 0;
  int16_t countr = 0;
  uint16_t enc_buff_l = TIM3->CNT;
  uint16_t enc_buff_r = TIM4->CNT;
  // int16_t enc_buff_l = (int16_t)TIM3->CNT;
  // int16_t enc_buff_r = (int16_t)TIM4->CNT;
  TIM3->CNT = 0;
  TIM4->CNT = 0;

  if( enc_buff_l > 32767 ){
    countl = (int16_t)enc_buff_l*-1;
  } else {
    countl = (int16_t)enc_buff_l;
  }

  if( enc_buff_r > 32767 ){
    countr = (int16_t)enc_buff_r*-1;
  } else {
    countr = (int16_t)enc_buff_r;
  }

  velocityL = (float)(countl) / 4096.0 * 2.0 * M_PI * 1000.0 * gear_ratio; // [rad/s]
  velocityR = (float)(countr) / 4096.0 * 2.0 * M_PI * 1000.0 * gear_ratio; // gear ratio 43:11
  velocity = (velocityL + velocityR) / 2.0;
}