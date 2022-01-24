#include "ir_sensor.h"

static uint16_t dma_f[3];
static uint16_t dma_b[2];

// static uint32_t fl[32];
// static uint32_t fr[32];
// static uint32_t bl[32];
// static uint32_t br[32];

static uint32_t fl[16];
static uint32_t fr[16];
static uint32_t bl[16];
static uint32_t br[16];

void IRPwmStart()
{
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 200);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 200);
}

void ReadFrontIRSensor(IR_SENSOR_Typedef *sensor, Battery_Typedef *battery)
{
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dma_f, 3);

  uint32_t ave_fl = 0;
  uint32_t ave_fr = 0;

  // FFT shift
  // for(int i = 32; i > 0; i--){
  //   fl[i] = fl[i - 1];
  //   fr[i] = fr[i - 1];
  // }
  // fl[0] = dma_f[0];
  // fr[0] = dma_f[1];

  for (int i = 15; i > 0; i--)
  {
    fl[i] = fl[i - 1];
    fr[i] = fr[i - 1];
  }
  fl[0] = dma_f[0];
  fr[0] = dma_f[1];

  for (int i = 15; i >= 0; i--)
  {
    ave_fl += fl[i];
    ave_fr += fr[i];
  }

  sensor->ir_fl = ave_fl / 16;
  sensor->ir_fr = ave_fr / 16;

  battery->bat_vol = (float)dma_f[2] * 3.3 / 4096 * 3;
}

void ReadBackIRSensor(IR_SENSOR_Typedef *sensor)
{
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)dma_b, 2);

  uint32_t ave_bl = 0;
  uint32_t ave_br = 0;

  for (int i = 15; i > 0; i--)
  {
    bl[i] = bl[i - 1];
    br[i] = br[i - 1];
  }
  bl[0] = dma_b[0];
  br[0] = dma_b[1];

  for (int i = 15; i >= 0; i--)
  {
    ave_bl += bl[i];
    ave_br += br[i];
  }

  sensor->ir_bl = ave_bl / 16;
  sensor->ir_br = ave_br / 16;
}

void GetIRSensorData(IR_SENSOR_Typedef *sensor)
{
  if (sensor->ir_fl > 2100)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  if (sensor->ir_fr > 2100)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  if (sensor->ir_bl > 2100)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  if (sensor->ir_br > 2100)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
}
