#include "race_chassis.h"
#include "hal_zdt.h"
#include "race_io.h"
#include "tim.h"
#include <math.h>
#include <stdlib.h>

#define ABSF(x) ((x) >= 0.0f ? (x) : -(x))

PID mypid = {0};
uint8_t dianji_move_flag = 0U;

static int car_move_speed = 50;
static int a1 = 70;
static int a = 230;
static float current_pos = 0.0f;
static float current_pos1 = 0.0f;
static float yuantai_angle = 60.84f;

static uint16_t servo_pulse_270(float angle)
{
  return (uint16_t)((angle / 270.0f) * 2000.0f + 500.0f);
}

static uint16_t servo_pulse_360(float angle)
{
  return (uint16_t)((angle / 360.0f) * 2000.0f + 500.0f);
}

static void can_filter_start(void)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0;
  filter.FilterIdLow = 0;
  filter.FilterMaskIdHigh = 0;
  filter.FilterMaskIdLow = 0;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  (void)HAL_CAN_ConfigFilter(&hcan1, &filter);
  (void)HAL_CAN_Start(&hcan1);
}

void race_chassis_init(void)
{
  can_filter_start();
  (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 520U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 648U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 838U);
  race_heat_set(0U);

  dianji_move_flag = 1U;
  PIDInit();
}

void race_heat_set(uint16_t compare)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare);
}

void change_A(int a_in)
{
  a = a_in;
}

void change_SNA1(int a_in, int speed)
{
  a1 = a_in;
  car_move_speed = speed;
}

void car_move(int v_x, int v_y, int w)
{
  int speed[5];

  speed[1] = +v_x + v_y - w;
  speed[2] = -v_x + v_y + w;
  speed[3] = -v_x + v_y - w;
  speed[4] = +v_x + v_y + w;

  (void)zdt_vel(2U, speed[1] >= 0 ? 0U : 1U, (uint16_t)(speed[1] >= 0 ? speed[1] : -speed[1]), (uint8_t)a, false);
  delay_ms1(1U);
  (void)zdt_vel(1U, speed[2] >= 0 ? 1U : 0U, (uint16_t)(speed[2] >= 0 ? speed[2] : -speed[2]), (uint8_t)a, false);
  delay_ms1(1U);
  (void)zdt_vel(3U, speed[3] >= 0 ? 0U : 1U, (uint16_t)(speed[3] >= 0 ? speed[3] : -speed[3]), (uint8_t)a, false);
  delay_ms1(1U);
  (void)zdt_vel(4U, speed[4] >= 0 ? 1U : 0U, (uint16_t)(speed[4] >= 0 ? speed[4] : -speed[4]), (uint8_t)a, false);
  delay_ms1(1U);
}

void car_move_distance_x(float x)
{
  int v_x;
  int quanshu;

  if (x == 0.0f)
  {
    return;
  }

  quanshu = (int)(ABSF(x) * 10.5f);
  v_x = (int)((float)car_move_speed * x / ABSF(x));

  (void)zdt_pos(2U, v_x >= 0 ? 0U : 1U, (uint16_t)(v_x >= 0 ? v_x : -v_x), (uint8_t)a1, (uint32_t)quanshu, false, true);
  delay_ms1(5U);
  (void)zdt_pos(1U, v_x >= 0 ? 1U : 0U, (uint16_t)(v_x >= 0 ? v_x : -v_x), (uint8_t)a1, (uint32_t)quanshu, false, true);
  delay_ms1(5U);
  (void)zdt_pos(3U, v_x >= 0 ? 0U : 1U, (uint16_t)(v_x >= 0 ? v_x : -v_x), (uint8_t)a1, (uint32_t)quanshu, false, true);
  delay_ms1(5U);
  (void)zdt_pos(4U, v_x >= 0 ? 1U : 0U, (uint16_t)(v_x >= 0 ? v_x : -v_x), (uint8_t)a1, (uint32_t)quanshu, false, true);
  delay_ms1(5U);
  (void)zdt_sync(0U);
}

void car_move_distance_y(float y)
{
  int v_y;
  int quanshu;

  if (y == 0.0f)
  {
    return;
  }

  quanshu = (int)(ABSF(y) * 10.3f);
  v_y = (int)((float)car_move_speed * y / ABSF(y));

  (void)zdt_pos(2U, v_y >= 0 ? 0U : 1U, (uint16_t)(v_y >= 0 ? v_y : -v_y), (uint8_t)a1, (uint32_t)quanshu, false, true);
  delay_ms1(5U);
  (void)zdt_pos(1U, v_y >= 0 ? 1U : 0U, (uint16_t)(v_y >= 0 ? v_y : -v_y), (uint8_t)a1, (uint32_t)quanshu, false, true);
  delay_ms1(5U);
  (void)zdt_pos(3U, v_y >= 0 ? 0U : 1U, (uint16_t)(v_y >= 0 ? v_y : -v_y), (uint8_t)a1, (uint32_t)quanshu, false, true);
  delay_ms1(5U);
  (void)zdt_pos(4U, v_y >= 0 ? 1U : 0U, (uint16_t)(v_y >= 0 ? v_y : -v_y), (uint8_t)a1, (uint32_t)quanshu, false, true);
  delay_ms1(5U);
  (void)zdt_sync(0U);
}

double car_move_delay(float distance, uint16_t a_of_car, uint16_t speed_of_car, uint8_t direct)
{
  double quanshu;
  double quanshu_fenjie;
  double time_fenjie;
  double time;

  if (a_of_car == 0U)
  {
    a_of_car = 256U;
  }

  quanshu = (direct == 0U) ? ABSF(distance) * 10.5 : ABSF(distance) * 10.3;
  quanshu_fenjie = 320.0 * (double)speed_of_car * (double)speed_of_car / ((double)a_of_car * 6.0);
  time_fenjie = 2000.0 * (double)speed_of_car / (double)a_of_car;

  if (quanshu_fenjie >= quanshu)
  {
    time = time_fenjie * sqrt(quanshu / quanshu_fenjie);
  }
  else
  {
    time = 1000.0 * (quanshu - quanshu_fenjie) / (320.0 * (double)speed_of_car / 6.0) + time_fenjie;
  }

  if (ABSF(distance) <= 2.0f)
  {
    time = 200.0;
  }

  delay_ms1((uint32_t)time);
  return time;
}

static double shengjiang_move_delay(float quanshu, uint16_t accel, uint16_t speed)
{
  double quanshu_fenjie;
  double time_fenjie;
  double time;

  if (accel == 0U)
  {
    accel = 256U;
  }

  quanshu_fenjie = 320.0 * (double)speed * (double)speed / ((double)accel * 6.0);
  time_fenjie = 2000.0 * (double)speed / (double)accel;

  if (quanshu_fenjie >= quanshu)
  {
    time = time_fenjie * sqrt((double)quanshu / quanshu_fenjie);
  }
  else
  {
    time = 1000.0 * ((double)quanshu - quanshu_fenjie) / (320.0 * (double)speed / 6.0) + time_fenjie;
  }

  time *= 0.5;
  if (time <= 200.0)
  {
    time = 200.0;
  }

  return time;
}

void car_move1(float x, float y, int a_in, int car_speed)
{
  (void)x;
  delay_ms1(10U);
  change_SNA1(a_in, car_speed);
  car_move_distance_y(-y);
  (void)car_move_delay(y, (uint16_t)a_in, (uint16_t)car_speed, 1U);
}

void car_move2(float x, float y, int a_in, int car_speed)
{
  (void)y;
  delay_ms1(10U);
  change_SNA1(a_in, car_speed);
  car_move_distance_x(-x);
  (void)car_move_delay(x, (uint16_t)a_in, (uint16_t)car_speed, 0U);
}

void car_move3(float x, float y, int a_in, int car_speed)
{
  (void)x;
  delay_ms1(10U);
  change_SNA1(a_in, car_speed);
  car_move_distance_y(-y);
}

void car_move4(float x, float y, int a_in, int car_speed)
{
  (void)y;
  delay_ms1(10U);
  change_SNA1(a_in, car_speed);
  car_move_distance_x(-x);
}

void car_move_distance(float x, float y, int a_in, int speed_in)
{
  delay_ms1(10U);
  change_SNA1(a_in, speed_in);
  car_move_distance_y(x);
  (void)car_move_delay(x, (uint16_t)a_in, (uint16_t)speed_in, 1U);

  delay_ms1(10U);
  change_SNA1(a_in, speed_in);
  car_move_distance_x(y);
  (void)car_move_delay(y, (uint16_t)a_in, (uint16_t)speed_in, 0U);
}

void PIDInit(void)
{
  mypid.kp = 1.5f;
  mypid.ki = 0.001f;
  mypid.kd = 2.0f;
  mypid.maxIntegral = 7.0f;
  mypid.maxOutput = 180.0f;
}

uint8_t PID_move(int v_x, int v_y, int w, uint32_t time, float target, int pid_choose)
{
  (void)target;
  (void)pid_choose;

  change_A(230);
  car_move(v_x, v_y, w);
  delay_ms1(time);
  car_move(0, 0, 0);
  delay_ms1(100U);
  return 1U;
}

double shengjiang_control(int target_pos, uint16_t speed, uint8_t accel)
{
  int move_distance;
  uint8_t dir;
  uint32_t pulses;
  double time;

  if (target_pos > 135)
  {
    target_pos = 135;
  }
  if (target_pos < 0)
  {
    target_pos = 0;
  }

  move_distance = target_pos - (int)current_pos1;
  dir = (move_distance > 0) ? 0U : 1U;
  pulses = (uint32_t)(abs(move_distance) * 80);

  if (move_distance != 0)
  {
    (void)zdt_pos(5U, dir, speed, accel, pulses, false, false);
  }

  time = shengjiang_move_delay((float)pulses, accel, speed);
  current_pos1 = (float)target_pos;
  return time;
}

double pingtui_control(float target_pos, uint16_t speed, uint8_t accel)
{
  float move_distance;
  uint8_t dir;
  uint32_t pulses;
  double time;

  if (target_pos > 65.0f)
  {
    target_pos = 65.0f;
  }
  if (target_pos < -122.0f)
  {
    target_pos = -122.0f;
  }

  move_distance = target_pos - current_pos;
  dir = (move_distance > 0.0f) ? 0U : 1U;
  pulses = (uint32_t)(ABSF(move_distance) * 25.465f);

  if (move_distance != 0.0f)
  {
    (void)zdt_pos(6U, dir, speed, accel, pulses, false, false);
  }

  time = shengjiang_move_delay((float)pulses, accel, speed);
  current_pos = target_pos;
  return time;
}

void pingtui_reset(void)
{
  (void)pingtui_control(0.0f, 1000U, 200U);
}

void set_yuantai_Angle(float target, float time)
{
  yuantai_angle = target;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, servo_pulse_360(target - 7.9f));
  delay_ms1((uint32_t)time);
}

void set_zhuashou_Angle(float target_abs, float speed)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, servo_pulse_270(target_abs - 1.0f));
  delay_ms1((uint32_t)speed);
}

void set_wukuaipingtai_Angle(float target_abs, float speed)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, servo_pulse_270(target_abs));
  delay_ms1((uint32_t)speed);
}

void set_zhuashou_he(void)
{
  set_zhuashou_Angle(6.0f, 300.0f);
}

void set_zhuashou_kai(void)
{
  set_zhuashou_Angle(45.0f, 300.0f);
}

void set_wukuaipingtai_weizhi(int pos)
{
  if (pos == 0x01)
  {
    set_wukuaipingtai_Angle(20.0f, 300.0f);
  }
  else if (pos == 0x02)
  {
    set_wukuaipingtai_Angle(20.0f + 119.0f, 300.0f);
  }
  else if (pos == 0x03)
  {
    set_wukuaipingtai_Angle(20.0f + 119.0f + 117.0f, 300.0f);
  }
}

int move_all(float target_pos, uint16_t speed, uint8_t accel, float target_pos1, uint16_t speed1, uint8_t accel1, float target, float speed_of_yuntai)
{
  double time1;
  double time2;
  double time3;
  double max_time;

  (void)speed_of_yuntai;
  accel = 230U;
  accel1 = 240U;
  speed1 = 2000U;

  time1 = pingtui_control(target_pos, speed, accel);
  time2 = shengjiang_control((int)target_pos1, speed1, accel1);
  time3 = 70.0 * fabs((double)target - (double)yuantai_angle) / 13.0;
  set_yuantai_Angle(target, (float)time3);

  max_time = time1;
  if (time2 > max_time) max_time = time2;
  if (time3 > max_time) max_time = time3;
  delay_ms1((uint32_t)max_time);
  return (int)max_time;
}

int move_all1(float target_pos, uint16_t speed, uint8_t accel, float target_pos1, uint16_t speed1, uint8_t accel1, float target, float speed_of_yuntai)
{
  (void)speed_of_yuntai;
  accel = 230U;
  accel1 = 240U;
  speed1 = 2000U;

  (void)pingtui_control(target_pos, speed, accel);
  (void)shengjiang_control((int)target_pos1, speed1, accel1);
  set_yuantai_Angle(target, 300.0f);
  return 300;
}
