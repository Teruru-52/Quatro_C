#include "control.h"

static float pre_error;
static float sum_error = 0.0f;
static float pre_error2;
static float sum_error2 = 0.0f;
static float pre_deriv2;
static float pre_error3;
static float sum_error3 = 0.0f;
static float pre_deriv3;
static float filtered_ref = 0.0f;

static const float radious = 0.0125f;
static float pos = 0.0f;

float u_left, u_right;
float u_turn;

Control_Typedef pid_1, pid_2, pid_3;

// static float dt_recip;  // 1/sampling time

void PIDControlInit(Control_Typedef *pid1, Control_Typedef *pid2, Control_Typedef *pid3)
{
  // Angle Control
  pid1->kp = YAW_PID_KP;
  pid1->ki = YAW_PID_KI;
  pid1->kd = YAW_PID_KD;
  pid1->ref = M_PI;
  // Angular Velocity Control
  pid2->kp = GYRO_PID_KP;
  pid2->ki = GYRO_PID_KI;
  pid2->kd = GYRO_PID_KD;
  pid2->ref = 0.0f;
  // Velocity Control
  pid3->kp = VEL_PID_KP;
  pid3->ki = VEL_PID_KI;
  pid3->kd = VEL_PID_KD;
  pid3->ref = 20.0f; // [rad/s]

  yaw = 0.0f;
  pre_error = 0.0f;
  sum_error = 0.0f;
  pre_error2 = 0.0f;
  sum_error2 = 0.0f;
  pre_error3 = 0.0f;
  sum_error3 = 0.0f;
  filtered_ref = 0.0f;
}

void SetReference(Control_Typedef *pid1, float ref_ang)
{
  PIDControlInit(&pid_1, &pid_2, &pid_3);
  pid1->ref = ref_ang;
}

float AngleControl(Control_Typedef *pid1)
{
  float error, deriv, vel_ref;
  error = pid1->ref - yaw;
  sum_error += error * CONTROL_PERIOD;
  deriv = (error - pre_error) / CONTROL_PERIOD;
  vel_ref = pid1->kp * error + pid1->ki * sum_error + pid1->kd * deriv;

  pre_error = error;

  return vel_ref;
}

float AngularVelocityControl(Control_Typedef *pid2)
{
  float error2, deriv2, u_ang;
  pid2->ref = AngleControl(&pid_1);
  error2 = pid2->ref - gz;
  sum_error2 += error2 * CONTROL_PERIOD;
  deriv2 = (error2 - pre_error2) / CONTROL_PERIOD;
  deriv2 = D_FILTER_COFF * pre_deriv2 + (1.0f - D_FILTER_COFF) * deriv2;
  u_ang = pid2->kp * error2 + pid2->ki * sum_error2 + pid2->kd * deriv2;

  pre_error2 = error2;
  pre_deriv2 = deriv2;

  return u_ang;
}

float VelocityControl(Control_Typedef *pid3)
{
  float error3, deriv3, u_vel;
  error3 = (pid3->ref - velocity); // [rad/s]
  sum_error3 += error3 * CONTROL_PERIOD;
  deriv3 = (error3 - pre_error3) / CONTROL_PERIOD;
  deriv3 = D_FILTER_COFF2 * pre_deriv3 + (1.0f - D_FILTER_COFF2) * deriv3;
  u_vel = pid3->kp * error3 + pid3->ki * sum_error3 + pid3->kd * deriv3;

  pre_error3 = error3;
  pre_deriv3 = deriv3;

  return u_vel;
}

void PositionControl(){
  pos += velocity * radious * CONTROL_PERIOD;
  if(pos > 0.15) { // pos > 15[cm]
    MotorStop();
    flag_int = false;
  }
}

void PartyTrick()
{
  float u_ang;
  u_ang = AngularVelocityControl(&pid_2);
  u_turn = u_ang;

  if (u_ang >= MAX_INPUT)
    u_ang = MAX_INPUT;
  if (u_ang <= -MAX_INPUT)
    u_ang = -MAX_INPUT;

  if (u_ang > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT - u_ang);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT - u_ang);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT + u_ang);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT + u_ang);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT);
  }
}

void GoStraight()
{
  float u_ang, u_vel;
  u_ang = AngularVelocityControl(&pid_2);
  u_vel = VelocityControl(&pid_3);
  u_left = (int)(u_vel - u_ang);
  u_right = (int)(u_vel + u_ang);

  if (u_left >= MAX_INPUT)
    u_left = MAX_INPUT;
  else if (u_left <= -MAX_INPUT)
    u_left = -MAX_INPUT;

  if (u_right >= MAX_INPUT)
    u_right = MAX_INPUT;
  else if (u_right <= -MAX_INPUT)
    u_right = -MAX_INPUT;

  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT - u_left);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT - u_right);
}

void DetectFrontWall()
{
  if (ir_bl > 2400 && ir_br > 2400)
  {
    MotorStop();
    // SetReference(&pid_1, M_PI);
    flag_int = false;
  }
}

void FrontWallCorrection(){
  int error_l, error_r;
  int u_left = IR_KP_LEFT * (IR_THR_LEFT - (int)ir_bl);
  int u_right = IR_KP_RIGHT * (IR_THR_RIGHT - (int)ir_br);

  if (u_left > 0){
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT - u_left);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT);
  }
  else{ 
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT + u_left);
  }

  if (u_right > 0) {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT - u_right);
  }
  else{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT + u_right);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT);
  }

  if (u_left >= 300 || u_left <= -300){
    flag_int = false;
    MotorStop();
  }

  if (u_right >= 300 || u_right <= -300){
    flag_int = false;
    MotorStop();
  }
}

void TurnLeft(Control_Typedef *pid2)
{
  float error2, deriv2, u_ang, u_fb, u_ff;
  if(cnt_turn <= 100){
    pid2->ref = 7.854 / 0.1 * cnt_turn * CONTROL_PERIOD;
  }
  else if (cnt_turn > 100 && cnt_turn <= 200){
    pid2->ref = 7.854;
  }
  else{
    pid2->ref = 7.854 - 7.854 / 0.1 * (cnt_turn - 200) * CONTROL_PERIOD;;
  }
  // filtered_ref = 0.75 * filtered_ref + (1.0 - 0.75) * pid2->ref;
  error2 = pid2->ref - gz;
  sum_error2 += error2 * CONTROL_PERIOD;
  deriv2 = (error2 - pre_error2) / CONTROL_PERIOD;
  deriv2 = D_FILTER_COFF * pre_deriv2 + (1.0f - D_FILTER_COFF) * deriv2;
  u_fb = pid2->kp * error2 + pid2->ki * sum_error2 + pid2->kd * deriv2;
  u_ff = 15 * pid2->ref;
  u_ang = u_fb + u_ff;

  pre_error2 = error2;
  pre_deriv2 = deriv2;

  if (u_ang >= MAX_INPUT)
    u_ang = MAX_INPUT;
  if (u_ang <= -MAX_INPUT)
    u_ang = -MAX_INPUT;

  if (u_ang > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT - u_ang);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT - u_ang);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT + u_ang);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT + u_ang);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT);
  }
}

void TurnRight(Control_Typedef *pid2){
  float error2, deriv2, u_ang, u_fb, u_ff;
  if(cnt_turn <= 100){
    pid2->ref = - 7.854 / 0.1 * cnt_turn * CONTROL_PERIOD;
  }
  else if (cnt_turn > 100 && cnt_turn <= 200){
    pid2->ref = - 7.854;
  }
  else{
    pid2->ref = - 7.854 + 7.854 / 0.1 * (cnt_turn - 200) * CONTROL_PERIOD;;
  }
  // filtered_ref = 0.75 * filtered_ref + (1.0 - 0.75) * pid2->ref;
  error2 = pid2->ref - gz;
  sum_error2 += error2 * CONTROL_PERIOD;
  deriv2 = (error2 - pre_error2) / CONTROL_PERIOD;
  deriv2 = D_FILTER_COFF * pre_deriv2 + (1.0f - D_FILTER_COFF) * deriv2;
  u_fb = pid2->kp * error2 + pid2->ki * sum_error2 + pid2->kd * deriv2;
  u_ff = 15 * pid2->ref;
  u_ang = u_fb + u_ff;

  pre_error2 = error2;
  pre_deriv2 = deriv2;

  if (u_ang >= MAX_INPUT)
    u_ang = MAX_INPUT;
  if (u_ang <= -MAX_INPUT)
    u_ang = -MAX_INPUT;

  if (u_ang > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT - u_ang);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT - u_ang);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT + u_ang);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT + u_ang);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT);
  }
}

void Uturn(Control_Typedef *pid2){
  float error2, deriv2, u_ang, u_fb, u_ff;
  if(cnt_turn <= 100){
    pid2->ref = 2 * 7.854 / 0.1 * cnt_turn * CONTROL_PERIOD;
  }
  else if (cnt_turn > 100 && cnt_turn <= 200){
    pid2->ref = 2 * 7.854;
  }
  else{
    pid2->ref = 2 * 7.854 - 2 * 7.854 / 0.1 * (cnt_turn - 200) * CONTROL_PERIOD;;
  }
  // filtered_ref = 0.75 * filtered_ref + (1.0 - 0.75) * pid2->ref;
  error2 = pid2->ref - gz;
  sum_error2 += error2 * CONTROL_PERIOD;
  deriv2 = (error2 - pre_error2) / CONTROL_PERIOD;
  deriv2 = D_FILTER_COFF * pre_deriv2 + (1.0f - D_FILTER_COFF) * deriv2;
  u_fb = pid2->kp * error2 + pid2->ki * sum_error2 + pid2->kd * deriv2;
  u_ff = 2 * pid2->ref;
  u_ang = u_fb + u_ff;

  pre_error2 = error2;
  pre_deriv2 = deriv2;

  if (u_ang >= MAX_INPUT)
    u_ang = MAX_INPUT;
  if (u_ang <= -MAX_INPUT)
    u_ang = -MAX_INPUT;

  if (u_ang > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT - u_ang);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT - u_ang);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT + u_ang);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT + u_ang);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT);
  }
}
void Back(){
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT - 100);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT - 100);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT);
}

void MotorStop()
{
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT);
}