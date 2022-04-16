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
static float filtered_v_ref = 0.0f;
static float filtered_a_ref = 0.0f;

static float dt1 = 0.03f;
static float dt2 = 0.04f;
static float dt3;

static const float radious = 0.0125f;
static float pos = 0.0f;

float u_left, u_right;
float u_turn;

float v_ref = 0.0f;
float a_ref = 0.0f;
float j_ref = 0.0f;
static const float jm = 3500.0f;
static const float am = jm * 0.03f;
static const float vm = 7.35f;
float cnt_turn = 0.0f;

Control_Typedef pid_1, pid_2, pid_3;

// static float dt_recip;  // 1/sampling time

void PIDControlInit(Control_Typedef *pid1, Control_Typedef *pid2, Control_Typedef *pid3)
{
  // Angle Control
  pid1->kp = YAW_PID_KP;
  pid1->ki = YAW_PID_KI;
  pid1->kd = YAW_PID_KD;
  pid1->ref = 0.0f;
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
  float error, deriv, u_ang1;
  error = pid1->ref - yaw;
  sum_error += error * CONTROL_PERIOD;
  deriv = (error - pre_error) / CONTROL_PERIOD;
  u_ang1 = pid1->kp * error + pid1->ki * sum_error + pid1->kd * deriv;

  pre_error = error;

  return u_ang1;
}

float AngularVelocityControl(Control_Typedef *pid2)
{
  float error2, deriv2, u_ang2;
  error2 = pid2->ref - gz;
  sum_error2 += error2 * CONTROL_PERIOD;
  deriv2 = (error2 - pre_error2) / CONTROL_PERIOD;
  deriv2 = D_FILTER_COFF * pre_deriv2 + (1.0f - D_FILTER_COFF) * deriv2;
  u_ang2 = pid2->kp * error2 + pid2->ki * sum_error2 + pid2->kd * deriv2;

  pre_error2 = error2;
  pre_deriv2 = deriv2;

  return u_ang2;
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
  u_ang = AngleControl(&pid_1) + AngularVelocityControl(&pid_2);

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

void UpdateReference(){
  if (cnt_turn <= dt1) {
    v_ref = 0.5f * jm * cnt_turn * cnt_turn;
    a_ref = jm * cnt_turn;
    j_ref = jm;
  }
  else if (cnt_turn <= dt1 + dt2) {
    v_ref = 0.5f * jm * dt1 * dt1 + am * (cnt_turn - 0.030f);
    a_ref = am;
    j_ref = 0.0f;
  }
  else if (cnt_turn <= 2 * dt1 + dt2) {
    v_ref = vm - 0.5f * jm * (0.1f - cnt_turn) * (0.1f - cnt_turn);
    a_ref = am - jm * (cnt_turn - 0.07f);
    j_ref = - jm;
  }
  else if (cnt_turn <= 2 * dt1 + dt2 + dt3) {
    v_ref = vm;
    a_ref = 0.0f;
    j_ref = 0.0f;
  }
  else if (cnt_turn <= 3 * dt1 + dt2 + dt3) {
    v_ref = vm - 0.5f * jm * (cnt_turn - 2 * dt1 - dt2 - dt3) * (cnt_turn - 2 * dt1 - dt2 - dt3);
    a_ref = - jm * (cnt_turn - 2 * dt1 - dt2 - dt3);
    j_ref = - jm;
  }
  else if (cnt_turn <= 3 * dt1 + 2 * dt2 + dt3) {
    v_ref = vm - 0.5f * jm * dt1 * dt1 - am * (cnt_turn - 3 * dt1 - dt2 - dt3);
    a_ref = - am;
    j_ref = 0.0f;
  }
  else if (cnt_turn <= 4 * dt1 + 2 * dt2 + dt3){
    v_ref = 0.5f * jm * (cnt_turn - 4 * dt1 - 2 * dt2 - dt3) * (cnt_turn - 4 * dt1 - 2 * dt2 - dt3);
    a_ref = - am + jm * (cnt_turn - 3 * dt1 - dt2 - dt3);
    j_ref = jm;
  }
  if (cnt_turn >= 4 * dt1 + 2 * dt2 + dt3){
    flag_int = false;
  }
  // filtered_v_ref = 0.2 * filtered_v_ref + (1.0 - 0.2) * v_ref;
  // filtered_a_ref = 0.2 * filtered_a_ref + (1.0 - 0.2) * a_ref;
  cnt_turn += 0.001f;
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
  dt3 = 0.113f;
  float error2, deriv2, u_ang, u_fb, u_ff;
  UpdateReference();

  error2 = v_ref - gz;
  sum_error2 += error2 * CONTROL_PERIOD;
  deriv2 = (error2 - pre_error2) / CONTROL_PERIOD;
  deriv2 = D_FILTER_COFF * pre_deriv2 + (1.0f - D_FILTER_COFF) * deriv2;
  u_fb = pid2->kp * error2 + pid2->ki * sum_error2 + pid2->kd * deriv2;
  u_ff = (0.109f * a_ref + v_ref) / 0.07366f;
  // u_ff = v_ref / 0.07366f;
  // u_ff = (3.2263 * j_ref + 3.10658 * a_ref + v_ref) / 0.11081;
  // u_ff = v_ref / 0.11081;
  // filtered_ref = 0.5 * filtered_ref + (1.0 - 0.5) * (0.20658 * a_ref + v_ref);
  // u_ff = filtered_ref / 0.11081;
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
  dt3 = 0.113f;
  float error2, deriv2, u_ang, u_fb, u_ff;
  UpdateReference();

  error2 = - v_ref - gz;
  sum_error2 += error2 * CONTROL_PERIOD;
  deriv2 = (error2 - pre_error2) / CONTROL_PERIOD;
  deriv2 = D_FILTER_COFF * pre_deriv2 + (1.0f - D_FILTER_COFF) * deriv2;
  u_fb = pid2->kp * error2 + pid2->ki * sum_error2 + pid2->kd * deriv2;
  u_ff = (- 0.109f * a_ref - v_ref) / 0.07366f;
  // u_ff = - v_ref / 0.07366f;
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
  dt3 = 0.326f;
  float error2, deriv2, u_ang, u_fb, u_ff;
  UpdateReference();

  // if (cnt_turn == 2 * dt1 + dt2 + dt3){
  //   pre_error2 = 0;
  //   pre_deriv2 = 0;
  //   sum_error2 = 0;
  // }
  error2 = v_ref - gz;
  sum_error2 += error2 * CONTROL_PERIOD;
  deriv2 = (error2 - pre_error2) / CONTROL_PERIOD;
  deriv2 = D_FILTER_COFF * pre_deriv2 + (1.0f - D_FILTER_COFF) * deriv2;
  u_fb = pid2->kp * error2 + pid2->ki * sum_error2 + pid2->kd * deriv2;
  u_ff = (0.109f * a_ref + v_ref) / 0.07366f;
  // u_ff = v_ref / 0.07366f;
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