#include "main.h"
#include "adc.h"

uint16_t dma_f[2];

void ReadFrontIRSensor(IR_SENSOR_Typedef *sensor){
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dma_f, 2);

  sensor->ir_fl = dma_f[0];
  sensor->ir_fr = dma_f[1];
  if (sensor->ir_fl > 2100)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  }
  if (sensor->ir_fr > 2100)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  }
}

uint16_t dma_b[2];

void ReadBackIRSensor(IR_SENSOR_Typedef *sensor){
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)dma_b, 2);

  sensor->ir_bl = dma_b[0];
  sensor->ir_br = dma_b[1];
  if (sensor->ir_bl > 2100)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
  }
  if (sensor->ir_br > 2100)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  }
}