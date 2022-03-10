#include "control.h"

static float pre_error;
static float sum_error = 0.0;
static float pre_error2;
static float sum_error2 = 0.0;
static float pre_deriv2;
static float pre_error3;
static float sum_error3 = 0.0;
static float pre_deriv3;

extern int count_idnt;
extern int count_idntm;
extern int count_mseq;

// static float dt_recip;  // 1/sampling time

void PIDControlInit(Control_Typedef *pid){
  pid->ts = PID_SAMPLING_TIME;
  // Angle Control
  pid->kp1 = YAW_PID_KP;
  pid->ki1 = YAW_PID_KI;
  pid->kd1 = YAW_PID_KD;
  pid->ref = 0.0;
  // Angular Velocity Control
  pid->kp2 = GYRO_PID_KP;
  pid->ki2 = GYRO_PID_KI;
  pid->kd2 = GYRO_PID_KD;
  pid->ref2 = 0.0;
  pid->u_ang = 0.0;
  // Velocity Control
  pid->kp3 = VEL_PID_KP;
  pid->ki3 = VEL_PID_KI;
  pid->kd3 = VEL_PID_KD;
  pid->ref3 = 0.0;
  pid->u_vel = 0.0;

  pid->vel = 0.0;
  pid->u_pid = 0.0;
}

void AngleControl(Gyro_Typedef *gyro, Control_Typedef *pid){
  float error, deriv;
  error = (pid->ref - gyro->yaw) * M_PI / 180;
  sum_error += error*pid->ts;
  deriv = (error - pre_error)/pid->ts;
  pid->ref2 = pid->kp1*error + pid->ki1*sum_error + pid->kd1*deriv;
  
  pre_error = error;
}

void AngularVelocityControl(Gyro_Typedef *gyro, Control_Typedef *pid){
  float error2, deriv2;
  error2 = (pid->ref2 - gyro->gz) * M_PI / 180;
  sum_error2 += error2*pid->ts;
  deriv2 = (error2 - pre_error2)/pid->ts;
  deriv2 = pre_deriv2 + (deriv2 - pre_deriv2)*D_FILTER_COFF;
  pid->u_ang = pid->kp2*error2 + pid->ki2*sum_error2 + pid->kd2*deriv2;

  pre_error2 = error2;
  pre_deriv2 = deriv2;
}

void VelocityControl(Encoder_Typedef *encoder, Control_Typedef *pid){
  float error3, deriv3;
  pid->vel = (encoder->countL + encoder->countR)/2;
  error3 = (pid->ref3 - pid->vel);
  sum_error3 += error3*pid->ts;
  deriv3 = (error3 - pre_error3)/pid->ts;
  deriv3 = pre_deriv3 + (deriv3 - pre_deriv3)*D_FILTER_COFF;
  pid->u_vel = pid->kp3*error3 + pid->ki3*sum_error3 + pid->kd3*deriv3;

  pre_error3 = error3;
  pre_deriv3 = deriv3;
}

void PIDControl(Control_Typedef *pid){
  pid->u_pid = pid->u_ang + pid->u_vel;

  if (pid->u_pid >= MAX_INPUT)
    pid->u_pid = MAX_INPUT;
  if (pid->u_pid <= -MAX_INPUT)
    pid->u_pid = -MAX_INPUT;

  if (pid->u_pid > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT - pid->u_pid);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT - pid->u_pid);
  }
  else{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT + pid->u_pid);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT + pid->u_pid);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT);
  }
}

void RotationControl(Battery_Typedef *battery, Data_Typedef *data, Gyro_Typedef *gyro)
{
  int u_iden = 300.0;
  data->output[count_idnt] = gyro->gz;
  data->input[count_idnt] = u_iden;

  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT - u_iden);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT - u_iden);
}

void MSequenceGen(MSequence_Typedef *msequence) // n = 7, N = 127
{
  msequence->m_sequence[0] = -2;
  msequence->m_sequence[1] = -2;
  msequence->m_sequence[2] = -2;
  msequence->m_sequence[3] = -2;
  msequence->m_sequence[4] = -2;
  msequence->m_sequence[5] = -2;
  msequence->m_sequence[6] = -2;
  msequence->m_sequence[7] = -2;
  msequence->m_sequence[8] = -2;
  msequence->m_sequence[9] = -2;
  msequence->m_sequence[10] = -2;
  msequence->m_sequence[11] = -2;
  msequence->m_sequence[12] = -2;
  msequence->m_sequence[13] = -2;
  msequence->m_sequence[14] = -2;
  msequence->m_sequence[15] = -2;
  msequence->m_sequence[16] = -2;
  msequence->m_sequence[17] = -2;
  msequence->m_sequence[18] = -2;
  msequence->m_sequence[19] = -2;
  msequence->m_sequence[20] = -2;
  msequence->m_sequence[21] = -2;
  msequence->m_sequence[22] = -2;
  msequence->m_sequence[23] = -2;
  msequence->m_sequence[24] = 2;
  msequence->m_sequence[25] = 2;
  msequence->m_sequence[26] = 2;
  msequence->m_sequence[27] = 2;
  msequence->m_sequence[28] = -2;
  msequence->m_sequence[29] = -2;
  msequence->m_sequence[30] = -2;
  msequence->m_sequence[31] = -2;
  msequence->m_sequence[32] = 2;
  msequence->m_sequence[33] = 2;
  msequence->m_sequence[34] = 2;
  msequence->m_sequence[35] = 2;
  msequence->m_sequence[36] = -2;
  msequence->m_sequence[37] = -2;
  msequence->m_sequence[38] = -2;
  msequence->m_sequence[39] = -2;
  msequence->m_sequence[40] = 2;
  msequence->m_sequence[41] = 2;
  msequence->m_sequence[42] = 2;
  msequence->m_sequence[43] = 2;
  msequence->m_sequence[44] = -2;
  msequence->m_sequence[45] = -2;
  msequence->m_sequence[46] = -2;
  msequence->m_sequence[47] = -2;
  msequence->m_sequence[48] = -2;
  msequence->m_sequence[49] = -2;
  msequence->m_sequence[50] = -2;
  msequence->m_sequence[51] = -2;
  msequence->m_sequence[52] = 2;
  msequence->m_sequence[53] = 2;
  msequence->m_sequence[54] = 2;
  msequence->m_sequence[55] = 2;
  msequence->m_sequence[56] = 2;
  msequence->m_sequence[57] = 2;
  msequence->m_sequence[58] = 2;
  msequence->m_sequence[59] = 2;
  msequence->m_sequence[60] = -2;
  msequence->m_sequence[61] = -2;
  msequence->m_sequence[62] = -2;
  msequence->m_sequence[63] = -2;
  msequence->m_sequence[64] = -2;
  msequence->m_sequence[65] = -2;
  msequence->m_sequence[66] = -2;
  msequence->m_sequence[67] = -2;
  msequence->m_sequence[68] = 2;
  msequence->m_sequence[69] = 2;
  msequence->m_sequence[70] = 2;
  msequence->m_sequence[71] = 2;
  msequence->m_sequence[72] = -2;
  msequence->m_sequence[73] = -2;
  msequence->m_sequence[74] = -2;
  msequence->m_sequence[75] = -2;
  msequence->m_sequence[76] = -2;
  msequence->m_sequence[77] = -2;
  msequence->m_sequence[78] = -2;
  msequence->m_sequence[79] = -2;
  msequence->m_sequence[80] = -2;
  msequence->m_sequence[81] = -2;
  msequence->m_sequence[82] = -2;
  msequence->m_sequence[83] = -2;
  msequence->m_sequence[84] = 2;
  msequence->m_sequence[85] = 2;
  msequence->m_sequence[86] = 2;
  msequence->m_sequence[87] = 2;
  msequence->m_sequence[88] = -2;
  msequence->m_sequence[89] = -2;
  msequence->m_sequence[90] = -2;
  msequence->m_sequence[91] = -2;
  msequence->m_sequence[92] = -2;
  msequence->m_sequence[93] = -2;
  msequence->m_sequence[94] = -2;
  msequence->m_sequence[95] = -2;
  msequence->m_sequence[96] = 2;
  msequence->m_sequence[97] = 2;
  msequence->m_sequence[98] = 2;
  msequence->m_sequence[99] = 2;
  msequence->m_sequence[100] = -2;
  msequence->m_sequence[101] = -2;
  msequence->m_sequence[102] = -2;
  msequence->m_sequence[103] = -2;
  msequence->m_sequence[104] = 2;
  msequence->m_sequence[105] = 2;
  msequence->m_sequence[106] = 2;
  msequence->m_sequence[107] = 2;
  msequence->m_sequence[108] = 2;
  msequence->m_sequence[109] = 2;
  msequence->m_sequence[110] = 2;
  msequence->m_sequence[111] = 2;
  msequence->m_sequence[112] = -2;
  msequence->m_sequence[113] = -2;
  msequence->m_sequence[114] = -2;
  msequence->m_sequence[115] = -2;
  msequence->m_sequence[116] = 2;
  msequence->m_sequence[117] = 2;
  msequence->m_sequence[118] = 2;
  msequence->m_sequence[119] = 2;
  msequence->m_sequence[120] = 2;
  msequence->m_sequence[121] = 2;
  msequence->m_sequence[122] = 2;
  msequence->m_sequence[123] = 2;
  msequence->m_sequence[124] = -2;
  msequence->m_sequence[125] = -2;
  msequence->m_sequence[126] = -2;
  msequence->m_sequence[127] = -2;
}

void MSequenceInput(Data_Typedef *data, Gyro_Typedef *gyro, Battery_Typedef *battery, MSequence_Typedef *msequence){
  int u_iden = (int)(1000.0 / battery->bat_vol * msequence->m_sequence[count_mseq]);

   data->output[count_idntm] = gyro->gz;
   data->input[count_idntm] = u_iden;

  if (u_iden >= MAX_INPUT)
    u_iden = MAX_INPUT;
  if (u_iden <= -MAX_INPUT)
    u_iden = -MAX_INPUT;

  if (u_iden > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT - u_iden);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT - u_iden);
  }
  else{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT + u_iden);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT + u_iden);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT);
  }
}

void MotorStop(){
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
}