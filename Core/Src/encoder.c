#include "encoder.h"
#include "tim.h"
#include <stdint.h>

void GetEncoderData(Encoder_Typedef *encoder)
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

  encoder->countL = countl;
  encoder->countR = countr;

  encoder->velocityL = (float)(countl) / 4096.0 * 2.0 * M_PI * 1000.0 * 11.0 / 43.0; // [rad/s]
  encoder->velocityR = (float)(countr) / 4096.0 * 2.0 * M_PI * 1000.0 * 11.0 / 43.0; // gear ratio 43:11
  encoder->velocity = (encoder->velocityL + encoder->velocityR) / 2.0;
}