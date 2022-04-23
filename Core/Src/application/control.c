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

static float dt3;
static float t1 = 0.03f;
static float t2 = 0.07f;
static float t3 = 0.1f;

static const float radious = 0.0125f;
static float pos = 0.0f;

float u_left, u_right;
float u_turn;

float x_ref = 0.0f;
float v_ref = 0.0f;
float a_ref = 0.0f;
float j_ref = 0.0f;
static const float jm = 3500.0f;
static const float am = jm * 0.03f;
static const float v3 = 7.35f;
float t = 0.0f;

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
  int u;
  u_ang = AngleControl(&pid_1) + AngularVelocityControl(&pid_2);
  u = (int)(1000.0f * u_ang / bat_vol);

  if (u >= MAX_INPUT)
    u = MAX_INPUT;
  if (u <= -MAX_INPUT)
    u = -MAX_INPUT;

  if (u > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT - u);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT - u);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT + u);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT + u);
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
  float x1 = 1.0f / 6.0f * jm * t1 * t1 * t1;
  float v1 = 0.5f * jm * t1 * t1;
  float x2 = x1 + v1 * (t2 - t1) + 0.5f * am * (t2 - t1) * (t2 - t1);
  float v2 = v1 + am * (t2 - t1);
  float x3 = x2 + v3 * t1 - 1.0f / 6.0f * jm * t1 * t1 * t1;
  float x4 = x3 + v3 * dt3;
  float x5 = x4 + (x3 - x2);
  float x6 = x5 + (x2 - x1);

  float t4 = t3 + dt3;
  float t5 = t4 + t1;
  float t6 = t4 + t2;
  float t7 = t4 + t3;

  if (t <= t1) {
    x_ref = 1.0f / 6.0f *jm * t * t * t;
    v_ref = 0.5f * jm * t * t;
    a_ref = jm * t;
    j_ref = jm;
  }
  else if (t <= t2) {
    x_ref = x1 + v1 * (t - t1) + 0.5f * am * (t - t1) * (t - t1);
    v_ref = v1 + am * (t - t1);
    a_ref = am;
    j_ref = 0.0f;
  }
  else if (t <= t3) {
    x_ref = x3 + v3 * (t - t3) - 1.0f / 6.0f * jm * (t - t3) * (t - t3) * (t - t3);
    v_ref = v3 - 0.5f * jm * (t - t3) * (t - t3);
    a_ref = am - jm * (t - t2);
    j_ref = - jm;
  }
  else if (t <= t4) {
    x_ref = x3 + v3 * (t - t3);
    v_ref = v3;
    a_ref = 0.0f;
    j_ref = 0.0f;
  }
  else if (t <= t5) {
    x_ref = x4 + v3 * (t - t4) - 1.0f / 6.0f * jm * (t - t4) * (t - t4) * (t - t4);
    v_ref = v3 - 0.5f * jm * (t - t4) * (t - t4);
    a_ref = - jm * (t - t4);
    j_ref = - jm;
  }
  else if (t <= t6) {
    x_ref = x5 + v2 * (t - t5) - 0.5f * am * (t - t5) * (t - t5);
    v_ref = v2 - am * (t - t5);
    a_ref = - am;
    j_ref = 0.0f;
  }
  else if (t <= t7){
    x_ref = x6 + v1 * (t - t6) - 1.0f / 6.0f * jm * (t - t6) * (t - t6) * (t - t6);
    v_ref = 0.5f * jm * (t - t7) * (t - t7);
    a_ref = - am + jm * (t - t6);
    j_ref = jm;
  }
  else if (t <= t7 + 1.0f){
    if(dt3 == 0.113f){
      x_ref = 3.1415f / 2.0f;
    }
    else {
      x_ref = 3.1415f;
    } 
    v_ref = 0.0f;
    a_ref = 0.0f;
  }

  if (t >= t7 + 1.0f){
    flag_int = false;
  }
  // filtered_v_ref = 0.2 * filtered_v_ref + (1.0 - 0.2) * v_ref;
  // filtered_a_ref = 0.2 * filtered_a_ref + (1.0 - 0.2) * a_ref;
  t += 0.001f;
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
  int u;
  UpdateReference();

  error2 = v_ref - gz;
  sum_error2 += error2 * CONTROL_PERIOD;
  deriv2 = (error2 - pre_error2) / CONTROL_PERIOD;
  deriv2 = D_FILTER_COFF * pre_deriv2 + (1.0f - D_FILTER_COFF) * deriv2;
  u_fb = pid2->kp * error2 + pid2->ki * sum_error2 + pid2->kd * deriv2;
  u_ff = (Tp1 * v_ref + a_ref) / Kp;
  // u_ff = v_ref / Kp;
  u_ang = u_fb + u_ff;
  u = (int)(MAX_INPUT * u_ang / bat_vol);

  pre_error2 = error2;
  pre_deriv2 = deriv2;

  if (u >= MAX_INPUT)
    u = MAX_INPUT;
  if (u <= -MAX_INPUT)
    u = -MAX_INPUT;

  if (u > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT - u);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT - u);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT + u);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT + u);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT);
  }
}

void TurnRight(Control_Typedef *pid2){
  dt3 = 0.113f;
  float error2, deriv2, u_ang, u_fb, u_ff;
  int u;
  UpdateReference();

  error2 = - v_ref - gz;
  sum_error2 += error2 * CONTROL_PERIOD;
  deriv2 = (error2 - pre_error2) / CONTROL_PERIOD;
  deriv2 = D_FILTER_COFF * pre_deriv2 + (1.0f - D_FILTER_COFF) * deriv2;
  u_fb = pid2->kp * error2 + pid2->ki * sum_error2 + pid2->kd * deriv2;
  u_ff = (Tp1 * v_ref + a_ref) / Kp;
  // u_ff = v_ref / Kp;
  u_ang = u_fb + u_ff;
  u = (int)(MAX_INPUT * u_ang / bat_vol);

  pre_error2 = error2;
  pre_deriv2 = deriv2;

  if (u >= MAX_INPUT)
    u = MAX_INPUT;
  if (u <= -MAX_INPUT)
    u = -MAX_INPUT;

  if (u > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT - u);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT - u);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT + u);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT + u);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT);
  }
}

void Uturn(Control_Typedef *pid1, Control_Typedef *pid2){
  dt3 = 0.326f;
  float error, deriv;
  float error2, deriv2, u_ang, u_fb, u_ff;
  int u;
  UpdateReference();

  // if (t == t3 + dt3){
  //   pre_error2 = 0;
  //   pre_deriv2 = 0;
  //   sum_error2 = 0;
  // }
  error = x_ref - yaw;
  sum_error += error * CONTROL_PERIOD;
  deriv = (error - pre_error) / CONTROL_PERIOD;

  error2 = v_ref - gz;
  sum_error2 += error2 * CONTROL_PERIOD;
  deriv2 = (error2 - pre_error2) / CONTROL_PERIOD;
  deriv2 = D_FILTER_COFF * pre_deriv2 + (1.0f - D_FILTER_COFF) * deriv2;
  u_fb = pid2->kp * error2 + pid2->ki * sum_error2 + pid2->kd * deriv2 + pid1->kp * error + pid1->ki * sum_error + pid1->kd * deriv;
  // u_ff = (Tp1 * v_ref + a_ref) / Kp;
  u_ff = Tp1 * v_ref / Kp;
  u_ang = u_fb + u_ff;
  u = (int)(MAX_INPUT * u_ang / bat_vol / 2.0f);

  pre_error2 = error2;
  pre_deriv2 = deriv2;

  if (u >= MAX_INPUT)
    u = MAX_INPUT;
  if (u <= -MAX_INPUT)
    u = -MAX_INPUT;

  if (u > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT - u);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, MAX_INPUT - u);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MAX_INPUT + u);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MAX_INPUT);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, MAX_INPUT + u);
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