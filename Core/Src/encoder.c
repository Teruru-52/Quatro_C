#include "encoder.h"
#include "tim.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

void GetEncoderL(Encoder_Typedef *encoder)
{
  // int16_t enc_buff = (int16_t)TIM3->CNT;
  // TIM3->CNT = 0;
  // return enc_buff;
  int16_t count = 0;
  uint16_t enc_buff = TIM3->CNT;
  TIM3->CNT = 0;
  if( enc_buff > 32767 ){
    count = (int16_t)enc_buff*-1;
  } else {
    count = (int16_t)enc_buff;
  }

  encoder->countL = count;
}

void GetEncoderR(Encoder_Typedef *encoder)
{
  // int16_t enc_buff = (int16_t)TIM4->CNT;
  // TIM4->CNT = 0;
  // return enc_buff;
  int16_t count = 0;
  uint16_t enc_buff = TIM4->CNT;
  TIM4->CNT = 0;
  if( enc_buff > 32767 ){
    count = (int16_t)enc_buff*-1;
  } else {
    count = (int16_t)enc_buff;
  }

  encoder->countR = count;
}