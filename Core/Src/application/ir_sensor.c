#include "ir_sensor.h"

static uint16_t dma_f[3];
static uint16_t dma_b[2];

static uint32_t fl[SAMPLING_COUNT];
static uint32_t fr[SAMPLING_COUNT];
static uint32_t bl[SAMPLING_COUNT];
static uint32_t br[SAMPLING_COUNT];

uint32_t ir_fl, ir_fr, ir_bl, ir_br;
float bat_vol;

void IRPwmStart()
{
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 200);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 200);
}

void ReadFrontIRSensor()
{
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dma_f, 3);

  for (int i = SAMPLING_COUNT - 1; i > 0; i--)
  {
    fl[i] = fl[i - 1];
    fr[i] = fr[i - 1];
  }
  fl[0] = dma_f[0];
  fr[0] = dma_f[1];

  ir_fl = dma_f[0];
  ir_fr = dma_f[1];

  bat_vol = (float)dma_f[2] * 3.3 / 4096 * 3;
}

void ReadBackIRSensor()
{
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)dma_b, 2);

  for (int i = SAMPLING_COUNT - 1; i > 0; i--)
  {
    bl[i] = bl[i - 1];
    br[i] = br[i - 1];
  }
  bl[0] = dma_b[0];
  br[0] = dma_b[1];

  ir_bl = dma_b[0];
  ir_br = dma_b[1];
}

void UpdateIRSensorData()
{
  uint32_t max_fl = 0;
  uint32_t max_fr = 0;
  uint32_t max_bl = 0;
  uint32_t max_br = 0;

  for (int i = SAMPLING_COUNT - 1; i >= 0; i--)
  {
    if (fl[i] > max_fl)
      max_fl = fl[i];
    if (fr[i] > max_fr)
      max_fr = fr[i];
  }
  for (int i = SAMPLING_COUNT - 1; i >= 0; i--)
  {
    if (bl[i] > max_bl)
      max_bl = bl[i];
    if (br[i] > max_br)
      max_br = br[i];
  }

  ir_fl = max_fl;
  ir_fr = max_fr;
  ir_bl = max_bl;
  ir_br = max_br;

  if (ir_fl > 2100)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  if (ir_fr > 2100)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  if (ir_bl > 2100)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  if (ir_br > 2100)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
}
