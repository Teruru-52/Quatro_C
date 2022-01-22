#include "gyro.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

float gyro_offset;
float yaw = 0;
int counter = 0;

uint8_t read_byte(uint8_t reg)
{
  uint8_t rx_data[2];
  uint8_t tx_data[2];

  tx_data[0] = reg | 0x80;
  tx_data[1] = 0x00;  // dummy

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET);

  return rx_data[1];
}

void write_byte(uint8_t reg, uint8_t data)
{
  uint8_t rx_data[2];
  uint8_t tx_data[2];

  //tx_data[0] = reg & 0x7F;
  tx_data[0] = reg | 0x00;
  tx_data[1] = data;  // write data

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET); //CSピン立ち下げ
  HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET); //CSピン立ち上げ
}

void GyroInit()
{
    uint8_t who_am_i;
    HAL_Delay(100);             // wait start up
    who_am_i = read_byte(WHO_AM_I); // read who am i
    // printf("who_am_i = 0x%x\r\n",who_am_i); // check who am i value
    // error check
    if (who_am_i != 0x70)
    {
        printf("gyro_error");
    }
    HAL_Delay(50);
    write_byte(PWR_MGMT_1, 0x80); // 3. set pwr_might (20MHz)
    HAL_Delay(100);
    write_byte(PWR_MGMT_1, 0x00); // initialization
    HAL_Delay(100);
    write_byte(CONFIG, 0x00); // 4. set config (FSYNCはNC)
    HAL_Delay(100);
    write_byte(GYRO_CONFIG, 0x18); // set gyro config (2000dps)
    HAL_Delay(100);
    // printf("0x%x\r\n", read_byte(0x1B));
}

void GyroOffsetCalc(Gyro_Typedef *gyro)
{
    float sum = 0;
    for (int i = 0; i < 1000; i++)
    {
        GetGyroZ(&gyro);
        sum += gyro->gz;
        HAL_Delay(1);
    }
    gyro_offset = sum / 1000.0;
}

void GetGyroZ(Gyro_Typedef *gyro)
{
    int16_t gz_raw;
    float gz;

    // H:8bit shift, Link h and l
    gz_raw = (int16_t)(((uint16_t)read_byte(GYRO_ZOUT_H) << 8) | (uint16_t)read_byte(GYRO_ZOUT_L));
    // printf("%d\r\n", gyro_z);
    gz = (float)(gz_raw / 16.4); // dps to deg/sec

    gyro->gz = gz - gyro_offset;
}

void GetYaw(Gyro_Typedef *gyro){
    yaw += gyro->gz * 0.001;

    gyro->yaw = yaw;
}