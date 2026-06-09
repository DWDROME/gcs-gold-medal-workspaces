#include "race_task.h"
#include "race_chassis.h"
#include "race_io.h"
#include <stdio.h>

#define x_center_red       955
#define y_center_red       519
#define x_center_green     955
#define y_center_green     519
#define x_center_blue      955
#define y_center_blue      519
#define YT_PLACE_RACK_DEG  0.0f
#define YT_RIGHT_RING_DEG  90.0f
#define YT_CENTER_RING_DEG 120.0f
#define YT_LEFT_RING_DEG   150.0f

static char tjcstr[100];
static int task[6];
static int ypcode[6];
static float direct_x = 0.0f;
static float direct_y = 0.0f;
static float distance_wukuai = 150.0f;
static float x_origin = 0.0f;
static float y_origin = 0.0f;

static void wait_send_shumeipai(uint8_t message, uint16_t time)
{
  (void)time;
  (void)Serial_GetRxFlag();
  Serial_SendByte(message);
  while (1)
  {
    if (Serial_GetRxFlag() == 1U)
    {
      break;
    }
  }
}

static void wait_packet_98(void)
{
  while (1)
  {
    if (Serial_RxPacket[0] == 0x39U && Serial_RxPacket[1] == 0x38U)
    {
      break;
    }
    if (Serial_GetRxFlag() == 1U &&
        Serial_RxPacket[0] == 0x39U && Serial_RxPacket[1] == 0x38U)
    {
      break;
    }
  }
  Serial_RxPacket[0] = 0x30U;
  Serial_RxPacket[1] = 0x30U;
}

static void clear_ckp(void)
{
  (void)sprintf(tjcstr, "t0.txt=\"  \"");
  HMISends(tjcstr);
  HMISendb(0xffU);
}

static void transfer_position_wukuai(uint8_t task_choose, float distance, uint8_t color)
{
  float bilichi = 0.0f;

  x_origin = (float)((Serial_RxPacket[0] - 0x30U) * 1000U +
                     (Serial_RxPacket[1] - 0x30U) * 100U +
                     (Serial_RxPacket[2] - 0x30U) * 10U +
                     (Serial_RxPacket[3] - 0x30U));
  y_origin = (float)((Serial_RxPacket[4] - 0x30U) * 1000U +
                     (Serial_RxPacket[5] - 0x30U) * 100U +
                     (Serial_RxPacket[6] - 0x30U) * 10U +
                     (Serial_RxPacket[7] - 0x30U));

  if (task_choose == 0x01U)
  {
    bilichi = -0.0035f * (distance + 80.0f) + 1.1967f;
    x_origin -= 322.0f;
    y_origin -= 165.0f;
  }
  else if (task_choose == 0x02U)
  {
    bilichi = -0.0033f * distance + 1.1655f;
    x_origin -= 314.0f;
    y_origin -= 238.0f;
  }

  direct_x = x_origin * bilichi;
  direct_y = y_origin * bilichi;

  if (color == 2U)
  {
    if (Serial_RxPacket[8] == 0x31U) direct_x += distance_wukuai;
    else if (Serial_RxPacket[8] == 0x33U) direct_x -= distance_wukuai;
  }
  else if (color == 1U)
  {
    if (Serial_RxPacket[8] == 0x32U) direct_x -= distance_wukuai;
    else if (Serial_RxPacket[8] == 0x33U) direct_x -= 2.0f * distance_wukuai;
  }
  else if (color == 3U)
  {
    if (Serial_RxPacket[8] == 0x32U) direct_x += distance_wukuai;
    else if (Serial_RxPacket[8] == 0x31U) direct_x += 2.0f * distance_wukuai;
  }

  if (Serial_RxPacket[8] == 0x34U)
  {
    direct_y = 5.0f;
    direct_x = -5.0f;
  }
}

static void transfer_position_sehuan(uint8_t fenbian_choose, uint8_t task_num)
{
  float x_center = 0.0f;
  float y_center = 0.0f;

  x_origin = (float)((Serial_RxPacket[0] - 0x30U) * 1000U +
                     (Serial_RxPacket[1] - 0x30U) * 100U +
                     (Serial_RxPacket[2] - 0x30U) * 10U +
                     (Serial_RxPacket[3] - 0x30U));
  y_origin = (float)((Serial_RxPacket[4] - 0x30U) * 1000U +
                     (Serial_RxPacket[5] - 0x30U) * 100U +
                     (Serial_RxPacket[6] - 0x30U) * 10U +
                     (Serial_RxPacket[7] - 0x30U));

  if (fenbian_choose == 0x01U)
  {
    x_center = 316.0f;
    y_center = 238.0f;
    direct_x = (x_origin - x_center) * 1.1805555f;
    direct_y = (y_origin - y_center) * 1.1805555f;
  }
  else if (fenbian_choose == 0x02U)
  {
    if (Serial_RxPacket[8] == 0x31U)
    {
      x_center = (float)x_center_red;
      y_center = (float)y_center_red;
    }
    else if (Serial_RxPacket[8] == 0x32U)
    {
      x_center = (float)x_center_green;
      y_center = (float)y_center_green;
    }
    else if (Serial_RxPacket[8] == 0x33U)
    {
      x_center = (float)x_center_blue;
      y_center = (float)y_center_blue;
    }
    direct_x = (x_origin - x_center) * 0.29239766f;
    direct_y = (y_origin - y_center) * 0.29239766f;
  }

  if (task_num == 2U)
  {
    if (Serial_RxPacket[8] == 0x31U) direct_x += distance_wukuai;
    else if (Serial_RxPacket[8] == 0x33U) direct_x -= distance_wukuai;
  }
  else if (task_num == 1U)
  {
    if (Serial_RxPacket[8] == 0x32U) direct_x -= distance_wukuai;
    else if (Serial_RxPacket[8] == 0x33U) direct_x -= 2.0f * distance_wukuai;
  }
  else if (task_num == 3U)
  {
    if (Serial_RxPacket[8] == 0x32U) direct_x += distance_wukuai;
    else if (Serial_RxPacket[8] == 0x31U) direct_x += 2.0f * distance_wukuai;
  }

  if (Serial_RxPacket[8] == 0x34U)
  {
    direct_y = 3.0f;
    direct_x = -3.0f;
  }
}

static void fangzhi_green(void)
{
  move_all(-48.0f, 1000U, 200U, 25.0f, 1000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
  move_all(-20.0f, 1000U, 200U, 25.0f, 1000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
  move_all(-20.0f, 1000U, 200U, 25.0f, 1000U, 200U, YT_CENTER_RING_DEG, 13.0f);
  move_all(-50.0f, 1000U, 200U, 132.0f, 1000U, 200U, YT_CENTER_RING_DEG, 13.0f);
  set_zhuashou_kai();
}

static void fangzhi_red(void)
{
  move_all(-48.0f, 1000U, 200U, 25.0f, 1000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
  move_all(4.0f, 1000U, 200U, 25.0f, 1000U, 200U, YT_RIGHT_RING_DEG, 13.0f);
  move_all(4.0f, 1000U, 200U, 132.0f, 1000U, 200U, YT_RIGHT_RING_DEG, 13.0f);
  set_zhuashou_kai();
}

static void fangzhi_blue(void)
{
  move_all(-48.0f, 1000U, 200U, 25.0f, 1000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
  move_all(8.8f, 1000U, 200U, 25.0f, 1000U, 200U, YT_LEFT_RING_DEG, 13.0f);
  move_all(8.8f, 1000U, 200U, 132.0f, 1000U, 200U, YT_LEFT_RING_DEG, 13.0f);
  set_zhuashou_kai();
}

static void fangzhi(uint8_t code)
{
  if (code == 0x01U) fangzhi_red();
  else if (code == 0x02U) fangzhi_green();
  else if (code == 0x03U) fangzhi_blue();
}

static void fangzhi_maduo(uint8_t code)
{
  if (code == 0x01U)
  {
    move_all(-48.0f, 1000U, 200U, 25.0f, 1000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
    move_all(3.0f, 1000U, 200U, 25.0f, 1000U, 200U, YT_RIGHT_RING_DEG, 13.0f);
    move_all(3.0f, 1000U, 200U, 60.0f, 1000U, 200U, YT_RIGHT_RING_DEG, 13.0f);
    set_zhuashou_kai();
  }
  else if (code == 0x02U)
  {
    move_all(-48.0f, 1000U, 200U, 25.0f, 1000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
    move_all(-48.0f, 1000U, 200U, 25.0f, 1000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
    move_all(-120.0f, 1000U, 200U, 25.0f, 1000U, 200U, YT_CENTER_RING_DEG, 13.0f);
    move_all(-50.0f, 1000U, 200U, 60.0f, 1000U, 200U, YT_CENTER_RING_DEG, 13.0f);
    set_zhuashou_kai();
  }
  else if (code == 0x03U)
  {
    move_all(-48.0f, 1000U, 200U, 25.0f, 1000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
    move_all(-48.0f, 1000U, 200U, 25.0f, 1000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
    move_all(-120.0f, 1000U, 200U, 25.0f, 1000U, 200U, YT_LEFT_RING_DEG, 13.0f);
    move_all(3.0f, 1000U, 200U, 60.0f, 1000U, 200U, YT_LEFT_RING_DEG, 13.0f);
    set_zhuashou_kai();
  }
}

static void na_green(void)
{
  move_all(-10.0f, 1000U, 200U, 130.0f, 1000U, 200U, YT_CENTER_RING_DEG, 13.0f);
  move_all(-50.0f, 1000U, 200U, 130.0f, 1000U, 200U, YT_CENTER_RING_DEG, 13.0f);
  set_zhuashou_he();
  move_all(-50.0f, 1000U, 200U, 50.0f, 1000U, 220U, YT_CENTER_RING_DEG, 13.0f);
  move_all(-48.0f, 1000U, 200U, 25.0f, 1000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
  move_all(-48.0f, 1000U, 200U, 33.0f, 1000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
  set_zhuashou_kai();
}

static void na_blue(void)
{
  move_all(3.0f, 1000U, 200U, 110.0f, 1000U, 200U, YT_LEFT_RING_DEG, 13.0f);
  move_all(3.0f, 1000U, 200U, 130.0f, 1000U, 200U, YT_LEFT_RING_DEG, 13.0f);
  set_zhuashou_he();
  move_all(8.0f, 1000U, 200U, 25.0f, 1000U, 220U, YT_LEFT_RING_DEG, 13.0f);
  move_all(-48.0f, 1000U, 200U, 25.0f, 1000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
  move_all(-48.0f, 1000U, 200U, 33.0f, 1000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
  set_zhuashou_kai();
}

static void na_red(void)
{
  move_all(3.0f, 1000U, 200U, 130.0f, 1000U, 200U, YT_RIGHT_RING_DEG, 13.0f);
  set_zhuashou_he();
  move_all(-50.0f, 1000U, 200U, 50.0f, 1000U, 220U, YT_RIGHT_RING_DEG, 13.0f);
  move_all(-48.0f, 1000U, 200U, 25.0f, 1000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
  move_all(-48.0f, 1000U, 200U, 33.0f, 1000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
  set_zhuashou_kai();
}

static void na(uint8_t code)
{
  if (code == 0x01U) na_red();
  else if (code == 0x02U) na_green();
  else if (code == 0x03U) na_blue();
}

static void yuantai_na(uint8_t code)
{
  if (code == 0x01U)
  {
    set_zhuashou_kai();
    move_all(-100.0f, 1000U, 220U, 50.0f, 2000U, 220U, YT_CENTER_RING_DEG, 13.0f);
    set_zhuashou_he();
    move_all(-48.0f, 1000U, 200U, 25.0f, 2000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
    move_all(-48.0f, 1000U, 200U, 33.0f, 2000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
    set_zhuashou_kai();
  }
  else if (code == 0x02U)
  {
    set_zhuashou_kai();
    move_all(63.0f, 1000U, 200U, 48.0f, 2000U, 220U, YT_LEFT_RING_DEG, 13.0f);
    set_zhuashou_he();
    move_all(20.0f, 1000U, 200U, 25.0f, 2000U, 220U, YT_LEFT_RING_DEG, 13.0f);
    move_all(-20.0f, 1000U, 200U, 25.0f, 2000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
    move_all(-48.0f, 1000U, 200U, 25.0f, 2000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
    move_all(-48.0f, 1000U, 200U, 33.0f, 2000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
    set_zhuashou_kai();
  }
  else if (code == 0x03U)
  {
    set_zhuashou_kai();
    move_all(63.0f, 1000U, 200U, 48.0f, 2000U, 220U, YT_RIGHT_RING_DEG, 13.0f);
    set_zhuashou_he();
    move_all(-48.0f, 1000U, 200U, 25.0f, 2000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
    move_all(-48.0f, 1000U, 200U, 33.0f, 2000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
    set_zhuashou_kai();
  }
}

static void yuantai_na1(uint8_t code)
{
  if (code == 0x01U)
  {
    set_zhuashou_kai();
    move_all(40.0f, 1000U, 200U, 0.0f, 2000U, 200U, YT_CENTER_RING_DEG, 13.0f);
  }
  else if (code == 0x02U)
  {
    set_zhuashou_kai();
    move_all(63.0f, 1000U, 200U, 18.0f, 2000U, 220U, YT_LEFT_RING_DEG, 13.0f);
  }
  else if (code == 0x03U)
  {
    set_zhuashou_kai();
    move_all(63.0f, 1000U, 200U, 18.0f, 2000U, 220U, YT_RIGHT_RING_DEG, 13.0f);
  }
}

static void yuantai_na2(uint8_t code)
{
  if (code == 0x01U)
  {
    move_all(-100.0f, 1000U, 220U, 50.0f, 2000U, 220U, YT_CENTER_RING_DEG, 13.0f);
    set_zhuashou_he();
    move_all(-48.0f, 1000U, 200U, 25.0f, 2000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
    move_all(-48.0f, 1000U, 200U, 33.0f, 2000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
    set_zhuashou_kai();
  }
  else if (code == 0x02U)
  {
    move_all(63.0f, 1000U, 200U, 48.0f, 2000U, 220U, YT_LEFT_RING_DEG, 13.0f);
    set_zhuashou_he();
    move_all(20.0f, 1000U, 200U, 25.0f, 2000U, 220U, YT_LEFT_RING_DEG, 13.0f);
    move_all(-20.0f, 1000U, 200U, 25.0f, 2000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
    move_all(-48.0f, 1000U, 200U, 25.0f, 2000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
    move_all(-48.0f, 1000U, 200U, 33.0f, 2000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
    set_zhuashou_kai();
  }
  else if (code == 0x03U)
  {
    move_all(63.0f, 1000U, 200U, 48.0f, 2000U, 220U, YT_RIGHT_RING_DEG, 13.0f);
    set_zhuashou_he();
    move_all(-48.0f, 1000U, 200U, 25.0f, 2000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
    move_all(-48.0f, 1000U, 200U, 33.0f, 2000U, 220U, YT_PLACE_RACK_DEG, 13.0f);
    set_zhuashou_kai();
  }
}

static void race_finish_stop(void)
{
  set_zhuashou_kai();
  move_all(0.0f, 1000U, 200U, 0.0f, 1000U, 200U, 68.4f, 13.0f);
  set_wukuaipingtai_Angle(0.0f, 300.0f);
  race_heat_set(0U);
  car_move(0, 0, 0);
  while (1)
  {
    LED_Toggle1();
    delay_ms1(500U);
  }
}

void race_task_run(void)
{
  mypid.integral = 0.0f;
  (void)Serial_GetRxFlag();
  Serial_SendByte(0x31U);

  car_move2(150.0f, 0.0f, 150, 150);
  car_move1(0.0f, -530.0f, 150, 200);

  while (1)
  {
    if (Serial_GetRxFlag() == 1U)
    {
      (void)sprintf(tjcstr, "t0.txt=\"%d%d%d+%d%d%d \"",
                    Serial_RxPacket[0] - 0x30U, Serial_RxPacket[1] - 0x30U, Serial_RxPacket[2] - 0x30U,
                    Serial_RxPacket[4] - 0x30U, Serial_RxPacket[5] - 0x30U, Serial_RxPacket[6] - 0x30U);
      HMISends(tjcstr);
      HMISendb(0xffU);
      break;
    }
  }

  task[0] = Serial_RxPacket[0] - 0x30U;
  task[1] = Serial_RxPacket[1] - 0x30U;
  task[2] = Serial_RxPacket[2] - 0x30U;
  task[3] = Serial_RxPacket[4] - 0x30U;
  task[4] = Serial_RxPacket[5] - 0x30U;
  task[5] = Serial_RxPacket[6] - 0x30U;

  set_wukuaipingtai_weizhi(0x01);
  race_heat_set(300U);

  move_all1(-120.0f, 1000U, 200U, 0.0f, 2000U, 200U, YT_CENTER_RING_DEG, 13.0f);
  set_zhuashou_kai();
  car_move1(0.0f, -910.0f, 150, 200);
  move_all1(40.0f, 1000U, 200U, 0.0f, 2000U, 200U, YT_CENTER_RING_DEG, 13.0f);

  (void)Serial_GetRxFlag();
  Serial_SendByte(0x32U);
  car_move2(-30.0f, 0.0f, 100, 100);
  while (1)
  {
    while (Serial_GetRxFlag() == 0U) {}
    transfer_position_wukuai(0x01U, 70.0f, 0x00U);
    car_move_distance(direct_x, direct_y, 100, 80);
    if (Serial_RxPacket[8] != 0x34U) break;
    wait_send_shumeipai(0x32U, 15000U);
  }

  move_all(60.0f, 1000U, 200U, 0.0f, 2000U, 200U, YT_CENTER_RING_DEG, 13.0f);
  wait_send_shumeipai(0x33U, 20000U);
  ypcode[0] = Serial_RxPacket[0] - 0x30U;
  ypcode[1] = Serial_RxPacket[1] - 0x30U;
  ypcode[2] = Serial_RxPacket[2] - 0x30U;

  set_wukuaipingtai_weizhi(0x01);
  yuantai_na((uint8_t)ypcode[0]);
  set_wukuaipingtai_weizhi(0x02);
  yuantai_na1((uint8_t)ypcode[1]);
  wait_send_shumeipai(0x35U, 30000U);
  wait_packet_98();
  yuantai_na2((uint8_t)ypcode[1]);
  set_wukuaipingtai_weizhi(0x03);
  yuantai_na1((uint8_t)ypcode[2]);
  wait_send_shumeipai(0x36U, 30000U);
  wait_packet_98();
  yuantai_na2((uint8_t)ypcode[2]);

  race_heat_set(0U);
  set_wukuaipingtai_weizhi(0x01);
  move_all1(-120.0f, 1000U, 200U, 0.0f, 1000U, 200U, YT_CENTER_RING_DEG, 13.0f);
  set_zhuashou_kai();

  PID_move(-30, 0, 0, 400U, 0.0f, 1);
  delay_ms1(200U);
  car_move1(0.0f, 400.0f, 150, 200);
  mypid.integral = 0.0f;
  PID_move(0, 0, 0, 1000U, 90.0f, 0);
  PID_move(0, 0, 0, 1000U, 90.0f, 3);
  car_move1(0.0f, -1700.0f, 150, 250);
  mypid.integral = 0.0f;
  PID_move(0, 0, 0, 1000U, -180.0f, 0);
  PID_move(0, 0, 0, 1000U, -180.0f, 3);

  race_heat_set(300U);
  while (1)
  {
    wait_send_shumeipai(0x37U, 10000U);
    transfer_position_sehuan(0x01U, 0x02U);
    car_move_distance(direct_x, direct_y, 180, 150);
    if (Serial_RxPacket[8] != 0x34U) break;
  }
  PID_move(0, 0, 0, 1000U, -180.0f, 4);

  wait_send_shumeipai(0x38U, 10000U);
  transfer_position_sehuan(0x02U, 0x02U);
  if (Serial_RxPacket[8] == 0x34U) car_move_distance(0.0f, 0.0f, 70, 50);
  else car_move_distance(direct_x, direct_y, 130, 100);

  move_all(-48.0f, 1000U, 200U, 33.0f, 1000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
  set_zhuashou_he();
  fangzhi((uint8_t)task[0]);
  set_wukuaipingtai_weizhi(0x02);
  delay_ms1((uint32_t)shengjiang_control(33, 2000U, 240U));
  move_all(-48.0f, 1000U, 200U, 33.0f, 1000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
  set_zhuashou_he();
  fangzhi((uint8_t)task[1]);
  set_wukuaipingtai_weizhi(0x03);
  delay_ms1((uint32_t)shengjiang_control(33, 2000U, 240U));
  move_all(-48.0f, 1000U, 200U, 33.0f, 2000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
  set_zhuashou_he();
  fangzhi((uint8_t)task[2]);
  set_wukuaipingtai_weizhi(0x01);
  na((uint8_t)task[0]);
  set_wukuaipingtai_weizhi(0x02);
  na((uint8_t)task[1]);
  set_wukuaipingtai_weizhi(0x03);
  na((uint8_t)task[2]);

  set_wukuaipingtai_weizhi(0x01);
  move_all1(-120.0f, 1000U, 200U, 0.0f, 1000U, 200U, YT_CENTER_RING_DEG, 13.0f);
  set_zhuashou_kai();
  PID_move(-30, 0, 0, 300U, -180.0f, 1);
  PID_move(0, 0, 0, 500U, -180.0f, 3);
  car_move1(140.0f, 820.0f, 180, 180);
  delay_ms1(100U);
  mypid.integral = 0.0f;
  PID_move(0, 0, 0, 1000U, 90.0f, 0);
  PID_move(0, 0, 0, 1000U, 90.0f, 3);
  car_move1(140.0f, 760.0f, 180, 150);

  race_heat_set(300U);
  while (1)
  {
    wait_send_shumeipai(0x37U, 10000U);
    transfer_position_sehuan(0x01U, 0x02U);
    car_move_distance(direct_x, direct_y, 180, 150);
    if (Serial_RxPacket[8] != 0x34U) break;
  }
  PID_move(0, 0, 0, 1000U, 90.0f, 4);

  wait_send_shumeipai(0x38U, 10000U);
  transfer_position_sehuan(0x02U, 0x02U);
  if (Serial_RxPacket[8] == 0x34U) car_move_distance(0.0f, 0.0f, 70, 50);
  else car_move_distance(direct_x, direct_y, 130, 100);

  move_all(-48.0f, 1000U, 200U, 33.0f, 1000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
  set_zhuashou_he();
  fangzhi((uint8_t)task[0]);
  set_wukuaipingtai_weizhi(0x02);
  delay_ms1((uint32_t)shengjiang_control(33, 2000U, 240U));
  move_all(-48.0f, 1000U, 200U, 33.0f, 1000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
  set_zhuashou_he();
  fangzhi((uint8_t)task[1]);
  set_wukuaipingtai_weizhi(0x03);
  delay_ms1((uint32_t)shengjiang_control(33, 2000U, 240U));
  move_all(-48.0f, 1000U, 200U, 33.0f, 2000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
  set_zhuashou_he();
  fangzhi((uint8_t)task[2]);

  set_wukuaipingtai_weizhi(0x01);
  move_all1(-120.0f, 1000U, 200U, 0.0f, 1000U, 200U, YT_CENTER_RING_DEG, 13.0f);
  set_zhuashou_kai();
  PID_move(-30, 0, 0, 300U, 90.0f, 1);
  delay_ms1(100U);
  car_move1(140.0f, 870.0f, 180, 180);
  mypid.integral = 0.0f;
  PID_move(0, 0, 0, 1000U, 0.0f, 0);
  PID_move(0, 0, 0, 1000U, 0.0f, 3);
  car_move1(140.0f, 390.0f, 180, 180);

  race_heat_set(300U);
  move_all1(40.0f, 1000U, 200U, 0.0f, 2000U, 200U, YT_CENTER_RING_DEG, 13.0f);
  (void)Serial_GetRxFlag();
  Serial_SendByte(0x61U);
  PID_move(30, 0, 0, 300U, 0.0f, 1);
  delay_ms1(100U);
  while (1)
  {
    while (Serial_GetRxFlag() == 0U) {}
    transfer_position_wukuai(0x01U, 70.0f, 0x00U);
    car_move_distance(direct_x, direct_y, 100, 80);
    if (Serial_RxPacket[8] != 0x34U) break;
    wait_send_shumeipai(0x61U, 15000U);
  }
  move_all(60.0f, 1000U, 200U, 0.0f, 2000U, 200U, YT_CENTER_RING_DEG, 13.0f);
  wait_send_shumeipai(0x34U, 20000U);
  ypcode[0] = Serial_RxPacket[0] - 0x30U;
  ypcode[1] = Serial_RxPacket[1] - 0x30U;
  ypcode[2] = Serial_RxPacket[2] - 0x30U;
  set_wukuaipingtai_weizhi(0x01);
  yuantai_na((uint8_t)ypcode[0]);
  set_wukuaipingtai_weizhi(0x02);
  yuantai_na1((uint8_t)ypcode[1]);
  wait_send_shumeipai(0x62U, 30000U);
  wait_packet_98();
  yuantai_na2((uint8_t)ypcode[1]);
  set_wukuaipingtai_weizhi(0x03);
  yuantai_na1((uint8_t)ypcode[2]);
  wait_send_shumeipai(0x63U, 30000U);
  wait_packet_98();
  yuantai_na2((uint8_t)ypcode[2]);

  race_heat_set(0U);
  set_wukuaipingtai_weizhi(0x01);
  move_all1(-120.0f, 1000U, 200U, 0.0f, 1000U, 200U, YT_CENTER_RING_DEG, 13.0f);
  set_zhuashou_kai();
  PID_move(-30, 0, 0, 400U, 0.0f, 1);
  delay_ms1(200U);
  car_move1(0.0f, 400.0f, 150, 200);
  mypid.integral = 0.0f;
  PID_move(0, 0, 0, 1000U, 90.0f, 0);
  PID_move(0, 0, 0, 1000U, 90.0f, 3);
  car_move1(0.0f, -1700.0f, 150, 250);
  mypid.integral = 0.0f;
  PID_move(0, 0, 0, 1000U, -180.0f, 0);
  PID_move(0, 0, 0, 1000U, -180.0f, 3);

  race_heat_set(300U);
  while (1)
  {
    wait_send_shumeipai(0x37U, 20000U);
    transfer_position_sehuan(0x01U, 0x02U);
    car_move_distance(direct_x, direct_y, 180, 150);
    if (Serial_RxPacket[8] != 0x34U) break;
  }
  PID_move(0, 0, 0, 1000U, -180.0f, 4);
  wait_send_shumeipai(0x38U, 20000U);
  transfer_position_sehuan(0x02U, 0x02U);
  if (Serial_RxPacket[8] == 0x34U) car_move_distance(0.0f, 0.0f, 70, 50);
  else car_move_distance(direct_x, direct_y, 130, 100);

  move_all(-48.0f, 1000U, 200U, 33.0f, 1000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
  set_zhuashou_he();
  fangzhi((uint8_t)task[3]);
  set_wukuaipingtai_weizhi(0x02);
  delay_ms1((uint32_t)shengjiang_control(33, 2000U, 240U));
  move_all(-48.0f, 1000U, 200U, 33.0f, 1000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
  set_zhuashou_he();
  fangzhi((uint8_t)task[4]);
  set_wukuaipingtai_weizhi(0x03);
  delay_ms1((uint32_t)shengjiang_control(33, 2000U, 240U));
  move_all(-48.0f, 1000U, 200U, 33.0f, 2000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
  set_zhuashou_he();
  fangzhi((uint8_t)task[5]);
  set_wukuaipingtai_weizhi(0x01);
  na((uint8_t)task[3]);
  set_wukuaipingtai_weizhi(0x02);
  na((uint8_t)task[4]);
  set_wukuaipingtai_weizhi(0x03);
  na((uint8_t)task[5]);

  set_wukuaipingtai_weizhi(0x01);
  move_all1(-120.0f, 1000U, 200U, 0.0f, 1000U, 200U, YT_CENTER_RING_DEG, 13.0f);
  set_zhuashou_kai();
  PID_move(-30, 0, 0, 300U, -180.0f, 1);
  PID_move(0, 0, 0, 500U, -180.0f, 3);
  car_move1(140.0f, 820.0f, 180, 180);
  delay_ms1(100U);
  mypid.integral = 0.0f;
  PID_move(0, 0, 0, 1000U, 90.0f, 0);
  PID_move(0, 0, 0, 1000U, 90.0f, 3);
  car_move1(140.0f, 760.0f, 180, 150);

  race_heat_set(300U);
  while (1)
  {
    wait_send_shumeipai(0x39U, 10000U);
    transfer_position_wukuai(0x02U, 70.0f, 0x02U);
    car_move_distance(direct_x, direct_y, 180, 150);
    delay_ms1(200U);
    if (Serial_RxPacket[8] != 0x34U) break;
  }
  wait_send_shumeipai(0x39U, 10000U);
  transfer_position_wukuai(0x02U, 70.0f, 0x02U);
  if (Serial_RxPacket[8] == 0x34U) car_move_distance(0.0f, 0.0f, 70, 50);
  else car_move_distance(direct_x, direct_y, 180, 150);

  move_all(-48.0f, 1000U, 200U, 33.0f, 1000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
  set_zhuashou_he();
  fangzhi_maduo((uint8_t)task[3]);
  set_wukuaipingtai_weizhi(0x02);
  delay_ms1((uint32_t)shengjiang_control(33, 2000U, 240U));
  move_all(-48.0f, 1000U, 200U, 33.0f, 1000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
  set_zhuashou_he();
  fangzhi_maduo((uint8_t)task[4]);
  set_wukuaipingtai_weizhi(0x03);
  delay_ms1((uint32_t)shengjiang_control(33, 2000U, 240U));
  move_all(-48.0f, 1000U, 200U, 33.0f, 2000U, 200U, YT_PLACE_RACK_DEG, 13.0f);
  set_zhuashou_he();
  fangzhi_maduo((uint8_t)task[5]);
  set_wukuaipingtai_weizhi(0x01);

  PID_move(-30, 0, 0, 300U, 90.0f, 1);
  car_move1(140.0f, 870.0f, 180, 180);
  move_all1(0.0f, 1000U, 200U, 0.0f, 1000U, 200U, 64.8f, 13.0f);
  mypid.integral = 0.0f;
  PID_move(0, 0, 0, 1000U, 0.0f, 0);
  PID_move(0, 0, 0, 1000U, 0.0f, 3);
  car_move1(140.0f, 1850.0f, 180, 180);
  car_move2(-130.0f, 0.0f, 150, 150);

  race_finish_stop();
}
