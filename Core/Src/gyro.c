#include "gyro.h"

static float yaw = 0;
bool flag_offset = false;

static float gyro_y_pre[4], gyro_x_pre[4];

Battery_Typedef bat_voltage;

//IIR filter
//7hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.922286512869545,  -0.92519529534950118, 0.00072719561998898304, 0.0014543912399779661, 0.00072719561998898304};

//15hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.8337326589246479,  -0.84653197479202391, 0.003199828966843966, 0.0063996579336879321, 0.003199828966843966};

//30hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.66920314293119312,  -0.71663387350415764, 0.011857682643241156, 0.023715365286482312, 0.011857682643241156};

//60hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.3489677452527946 ,  -0.51398189421967566, 0.041253537241720303, 0.082507074483440607, 0.041253537241720303};

//100hz, 800hz
static IIR_Coeff gyro_fil_coeff = {0.94280904158206336,  -0.33333333333333343, 0.09763107293781749 , 0.19526214587563498 , 0.09763107293781749};

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

void IIRInit()
{
    for (int i = 0; i < 4; i++)
    {
        gyro_y_pre[i] = 0;
        gyro_x_pre[i] = 0;
    }
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
        printf("gyro_error \r\n");
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
    BatteryCheckOn(&bat_voltage);

    int16_t gz_raw;
    float gz;
    float sum = 0;
    for (int i = 0; i < 1000; i++)
    {
        // H:8bit shift, Link h and l
        gz_raw = (int16_t)(((uint16_t)read_byte(GYRO_ZOUT_H) << 8) | (uint16_t)read_byte(GYRO_ZOUT_L));
        // printf("%d\r\n", gz_raw);
        gz = (float)(gz_raw / 16.4); // dps to deg/sec

        float filtered_gyro_z = gyro_fil_coeff.b0 * gz + gyro_fil_coeff.b1 * gyro_x_pre[0] + gyro_fil_coeff.b2 * gyro_x_pre[1] + gyro_fil_coeff.a1 * gyro_y_pre[0] + gyro_fil_coeff.a2 * gyro_y_pre[1];

        // Shift IIR filter state
        for (int j = 1; j > 0; j--)
        {
            gyro_x_pre[j] = gyro_x_pre[j - 1];
            gyro_y_pre[j] = gyro_y_pre[j - 1];
        }
        gyro_x_pre[0] = gz;
        gyro_y_pre[0] = filtered_gyro_z;
        sum += filtered_gyro_z; //filter
        // sum += gz; //nonfilter
        HAL_Delay(1);
    }
    gyro->offset = sum / 1000.0;
    // printf("%f\r\n", gyro_offset);

    // Speaker
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 10);
    HAL_Delay(50);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);

    BatteryCheckOff();

    IIRInit();

    flag_offset = true;
}

void GetGyroData(Gyro_Typedef *gyro)
{
    int16_t gz_raw;
    float gz;

    // H:8bit shift, Link h and l
    gz_raw = (int16_t)(((uint16_t)read_byte(GYRO_ZOUT_H) << 8) | (uint16_t)read_byte(GYRO_ZOUT_L));
    // printf("%d\r\n", gz_raw);
    gz = (float)(gz_raw / 16.4); // dps to deg/sec

    float filtered_gyro_z = gyro_fil_coeff.b0*gz + gyro_fil_coeff.b1*gyro_x_pre[0] + gyro_fil_coeff.b2*gyro_x_pre[1]
                                                    + gyro_fil_coeff.a1*gyro_y_pre[0] + gyro_fil_coeff.a2*gyro_y_pre[1];

    // Shift IIR filter state
    for (int i = 1; i > 0; i--)
    {
        gyro_x_pre[i] = gyro_x_pre[i - 1];
        gyro_y_pre[i] = gyro_y_pre[i - 1];
    }
    gyro_x_pre[0] = gz;
    gyro_y_pre[0] = filtered_gyro_z;

    gyro->gz = filtered_gyro_z - gyro->offset; //filter
    // gyro->gz = gz - gyro->offset;  // nonfilter
    yaw += gyro->gz * 0.001;
    gyro->yaw = yaw;
}