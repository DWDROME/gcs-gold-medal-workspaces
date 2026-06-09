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
#define ACT_PING_SPEED 1000U
#define ACT_PING_ACC 200U
#define ACT_LIFT_SPEED 1000U
#define ACT_LIFT_ACC 200U
#define ACT_YT_SPEED 13.0f
#define ACT_X_RACK 50.0f
#define ACT_X_PLACE 10.0f
#define ACT_X_SAFE 0.0f
#define ACT_H_RACK 60.0f
#define ACT_H_PLACE 60.0f
#define ACT_H_TRANSFER 10.0f
#define ACT_H_SAFE 0.0f
#define ACT_HEAT_COMPARE 300U

#define ASCII_0 0x30U
#define ASCII_8 0x38U
#define ASCII_9 0x39U

#define VISION_CMD_TASK_ORDER 0x31U
#define VISION_CMD_FIRST_BLOCK_LOCATE 0x32U
#define VISION_CMD_FIRST_BLOCK_CODES 0x33U
#define VISION_CMD_SECOND_BLOCK_CODES 0x34U
#define VISION_CMD_FIRST_BLOCK_CODE1 0x35U
#define VISION_CMD_FIRST_BLOCK_CODE2 0x36U
#define VISION_CMD_RING_COARSE 0x37U
#define VISION_CMD_RING_FINE 0x38U
#define VISION_CMD_SECOND_BLOCK_LOCATE 0x61U
#define VISION_CMD_SECOND_BLOCK_CODE1 0x62U
#define VISION_CMD_SECOND_BLOCK_CODE2 0x63U
#define VISION_CMD_LAST_BLOCK_LOCATE 0x39U

#define VISION_RESULT_RED 0x31U
#define VISION_RESULT_GREEN 0x32U
#define VISION_RESULT_BLUE 0x33U
#define VISION_RESULT_MISS 0x34U
#define VISION_WAIT_TASK_ORDER_MS 20000U
#define VISION_WAIT_ACK_MS 30000U

#define PLATFORM_POS_1 0x01
#define PLATFORM_POS_2 0x02
#define PLATFORM_POS_3 0x03

#define PID_PROFILE_YAW_TRANSIT 0
#define PID_PROFILE_YAW_LOW_OUTPUT 1
#define PID_PROFILE_YAW_SETTLE_KI_LOW 3
#define PID_PROFILE_YAW_SETTLE_KI_HIGH 4

static char tjcstr[100];
static int task[6];
static int ypcode[6];
static float direct_x = 0.0f;
static float direct_y = 0.0f;
static float distance_wukuai = 150.0f;
static float x_origin = 0.0f;
static float y_origin = 0.0f;

static void race_host_timeout_stop(void)
{
  race_actuator_stop_all();
  car_move(0, 0, 0);
  while (1)
  {
    LED_Toggle1();
    delay_ms1(200U);
  }
}

static uint8_t wait_rx_ms(uint32_t time)
{
  uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < time)
  {
    if (Serial_GetRxFlag() == 1U)
    {
      return 1U;
    }
  }
  return 0U;
}

static void wait_or_stop(uint32_t time)
{
  if (wait_rx_ms(time) == 0U)
  {
    race_host_timeout_stop();
  }
}

static void send_wait_or_stop(uint8_t message, uint16_t time)
{
  (void)Serial_GetRxFlag();
  Serial_SendByte(message);
  wait_or_stop(time);
}

static void wait_packet_98(void)
{
  uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < VISION_WAIT_ACK_MS)
  {
    if (Serial_RxPacket[0] == ASCII_9 && Serial_RxPacket[1] == ASCII_8)
    {
      break;
    }
    if (Serial_GetRxFlag() == 1U &&
        Serial_RxPacket[0] == ASCII_9 && Serial_RxPacket[1] == ASCII_8)
    {
      break;
    }
  }

  if (Serial_RxPacket[0] != ASCII_9 || Serial_RxPacket[1] != ASCII_8)
  {
    race_host_timeout_stop();
  }

  Serial_RxPacket[0] = ASCII_0;
  Serial_RxPacket[1] = ASCII_0;
}

static void transfer_position_wukuai(uint8_t task_choose, float distance, uint8_t color)
{
  float bilichi = 0.0f;

  x_origin = (float)((Serial_RxPacket[0] - ASCII_0) * 1000U +
                     (Serial_RxPacket[1] - ASCII_0) * 100U +
                     (Serial_RxPacket[2] - ASCII_0) * 10U +
                     (Serial_RxPacket[3] - ASCII_0));
  y_origin = (float)((Serial_RxPacket[4] - ASCII_0) * 1000U +
                     (Serial_RxPacket[5] - ASCII_0) * 100U +
                     (Serial_RxPacket[6] - ASCII_0) * 10U +
                     (Serial_RxPacket[7] - ASCII_0));

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
    if (Serial_RxPacket[8] == VISION_RESULT_RED) direct_x += distance_wukuai;
    else if (Serial_RxPacket[8] == VISION_RESULT_BLUE) direct_x -= distance_wukuai;
  }
  else if (color == 1U)
  {
    if (Serial_RxPacket[8] == VISION_RESULT_GREEN) direct_x -= distance_wukuai;
    else if (Serial_RxPacket[8] == VISION_RESULT_BLUE) direct_x -= 2.0f * distance_wukuai;
  }
  else if (color == 3U)
  {
    if (Serial_RxPacket[8] == VISION_RESULT_GREEN) direct_x += distance_wukuai;
    else if (Serial_RxPacket[8] == VISION_RESULT_RED) direct_x += 2.0f * distance_wukuai;
  }

  if (Serial_RxPacket[8] == VISION_RESULT_MISS)
  {
    direct_y = 5.0f;
    direct_x = -5.0f;
  }
}

static void transfer_position_sehuan(uint8_t fenbian_choose, uint8_t task_num)
{
  float x_center = 0.0f;
  float y_center = 0.0f;

  x_origin = (float)((Serial_RxPacket[0] - ASCII_0) * 1000U +
                     (Serial_RxPacket[1] - ASCII_0) * 100U +
                     (Serial_RxPacket[2] - ASCII_0) * 10U +
                     (Serial_RxPacket[3] - ASCII_0));
  y_origin = (float)((Serial_RxPacket[4] - ASCII_0) * 1000U +
                     (Serial_RxPacket[5] - ASCII_0) * 100U +
                     (Serial_RxPacket[6] - ASCII_0) * 10U +
                     (Serial_RxPacket[7] - ASCII_0));

  if (fenbian_choose == 0x01U)
  {
    x_center = 316.0f;
    y_center = 238.0f;
    direct_x = (x_origin - x_center) * 1.1805555f;
    direct_y = (y_origin - y_center) * 1.1805555f;
  }
  else if (fenbian_choose == 0x02U)
  {
    if (Serial_RxPacket[8] == VISION_RESULT_RED)
    {
      x_center = (float)x_center_red;
      y_center = (float)y_center_red;
    }
    else if (Serial_RxPacket[8] == VISION_RESULT_GREEN)
    {
      x_center = (float)x_center_green;
      y_center = (float)y_center_green;
    }
    else if (Serial_RxPacket[8] == VISION_RESULT_BLUE)
    {
      x_center = (float)x_center_blue;
      y_center = (float)y_center_blue;
    }
    direct_x = (x_origin - x_center) * 0.29239766f;
    direct_y = (y_origin - y_center) * 0.29239766f;
  }

  if (task_num == 2U)
  {
    if (Serial_RxPacket[8] == VISION_RESULT_RED) direct_x += distance_wukuai;
    else if (Serial_RxPacket[8] == VISION_RESULT_BLUE) direct_x -= distance_wukuai;
  }
  else if (task_num == 1U)
  {
    if (Serial_RxPacket[8] == VISION_RESULT_GREEN) direct_x -= distance_wukuai;
    else if (Serial_RxPacket[8] == VISION_RESULT_BLUE) direct_x -= 2.0f * distance_wukuai;
  }
  else if (task_num == 3U)
  {
    if (Serial_RxPacket[8] == VISION_RESULT_GREEN) direct_x += distance_wukuai;
    else if (Serial_RxPacket[8] == VISION_RESULT_RED) direct_x += 2.0f * distance_wukuai;
  }

  if (Serial_RxPacket[8] == VISION_RESULT_MISS)
  {
    direct_y = 3.0f;
    direct_x = -3.0f;
  }
}

static float yaw_for_code(uint8_t code)
{
  if (code == 0x01U)
  {
    return YT_RIGHT_RING_DEG;
  }
  if (code == 0x02U)
  {
    return YT_CENTER_RING_DEG;
  }
  return YT_LEFT_RING_DEG;
}

static void act_pose(float ping, float lift, float yaw)
{
  (void)move_all(ping, ACT_PING_SPEED, ACT_PING_ACC,
                 lift, ACT_LIFT_SPEED, ACT_LIFT_ACC,
                 yaw, ACT_YT_SPEED);
}

static void act_zero_before_chain(void)
{
  race_actuator_stop_all();
  race_pingtui_clear_zero();
  (void)race_shengjiang_homezero();
  set_zhuashou_kai();
  set_wukuaipingtai_weizhi(PLATFORM_POS_1);
  act_pose(ACT_X_SAFE, ACT_H_SAFE, YT_PLACE_RACK_DEG);
}

static void act_safe_transfer(void)
{
  act_pose(ACT_X_SAFE, ACT_H_TRANSFER, YT_PLACE_RACK_DEG);
}

static void act_pick_current_platform(void)
{
  set_zhuashou_kai();
  act_pose(ACT_X_RACK, ACT_H_RACK, YT_PLACE_RACK_DEG);
  set_zhuashou_he();
  act_pose(ACT_X_RACK, ACT_H_TRANSFER, YT_PLACE_RACK_DEG);
}

static void act_release_to_yaw(float yaw)
{
  act_pose(ACT_X_PLACE, ACT_H_TRANSFER, yaw);
  act_pose(ACT_X_PLACE, ACT_H_PLACE, yaw);
  set_zhuashou_kai();
  act_pose(ACT_X_PLACE, ACT_H_TRANSFER, yaw);
  act_safe_transfer();
}

static void act_pick_from_yaw(float yaw)
{
  set_zhuashou_kai();
  act_pose(ACT_X_PLACE, ACT_H_TRANSFER, yaw);
  act_pose(ACT_X_PLACE, ACT_H_PLACE, yaw);
  set_zhuashou_he();
  act_pose(ACT_X_PLACE, ACT_H_TRANSFER, yaw);
  act_pose(ACT_X_RACK, ACT_H_TRANSFER, YT_PLACE_RACK_DEG);
  act_pose(ACT_X_RACK, ACT_H_RACK, YT_PLACE_RACK_DEG);
  set_zhuashou_kai();
  act_safe_transfer();
}

static void act_current_platform_to_yaw(uint8_t code)
{
  act_pick_current_platform();
  act_release_to_yaw(yaw_for_code(code));
}

static void act_pre_yaw(uint8_t code)
{
  set_zhuashou_kai();
  act_pose(ACT_X_SAFE, ACT_H_TRANSFER, yaw_for_code(code));
}

static void fangzhi(uint8_t code)
{
  act_release_to_yaw(yaw_for_code(code));
}

static void fangzhi_maduo(uint8_t code)
{
  act_release_to_yaw(yaw_for_code(code));
}

static void na(uint8_t code)
{
  act_pick_from_yaw(yaw_for_code(code));
}

static void yuantai_na(uint8_t code)
{
  act_current_platform_to_yaw(code);
}

static void yuantai_na1(uint8_t code)
{
  act_pre_yaw(code);
}

static void yuantai_na2(uint8_t code)
{
  act_current_platform_to_yaw(code);
}

static void race_finish_stop(void)
{
  race_actuator_reset_all();
  race_heat_set(0U);
  car_move(0, 0, 0);
  while (1)
  {
    LED_Toggle1();
    delay_ms1(500U);
  }
}

static void race_stage_read_task_order(void)
{
  mypid.integral = 0.0f;
  (void)Serial_GetRxFlag();
  Serial_SendByte(VISION_CMD_TASK_ORDER);

  car_move2(150.0f, 0.0f, 150, 150);
  car_move1(0.0f, -530.0f, 150, 200);

  if (wait_rx_ms(VISION_WAIT_TASK_ORDER_MS) == 0U)
  {
    race_host_timeout_stop();
  }
  (void)snprintf(tjcstr, sizeof(tjcstr), "t0.txt=\"%d%d%d+%d%d%d \"",
                 Serial_RxPacket[0] - ASCII_0, Serial_RxPacket[1] - ASCII_0, Serial_RxPacket[2] - ASCII_0,
                 Serial_RxPacket[4] - ASCII_0, Serial_RxPacket[5] - ASCII_0, Serial_RxPacket[6] - ASCII_0);
  HMISends(tjcstr);
  HMISendb(0xffU);

  task[0] = Serial_RxPacket[0] - ASCII_0;
  task[1] = Serial_RxPacket[1] - ASCII_0;
  task[2] = Serial_RxPacket[2] - ASCII_0;
  task[3] = Serial_RxPacket[4] - ASCII_0;
  task[4] = Serial_RxPacket[5] - ASCII_0;
  task[5] = Serial_RxPacket[6] - ASCII_0;
}

static void race_stage_prepare_actuators(void)
{
  act_zero_before_chain();
  race_heat_set(ACT_HEAT_COMPARE);

  act_safe_transfer();
  set_zhuashou_kai();
  car_move1(0.0f, -910.0f, 150, 200);
  act_pose(ACT_X_SAFE, ACT_H_TRANSFER, YT_CENTER_RING_DEG);
}

static void race_stage_pick_first_platform_blocks(void)
{
  (void)Serial_GetRxFlag();
  Serial_SendByte(VISION_CMD_FIRST_BLOCK_LOCATE);
  car_move2(-30.0f, 0.0f, 100, 100);
  while (1)
  {
    wait_or_stop(15000U);
    transfer_position_wukuai(0x01U, 70.0f, 0x00U);
    car_move_distance(direct_x, direct_y, 100, 80);
    if (Serial_RxPacket[8] != VISION_RESULT_MISS) break;
    send_wait_or_stop(VISION_CMD_FIRST_BLOCK_LOCATE, 15000U);
  }

  act_pose(ACT_X_SAFE, ACT_H_TRANSFER, YT_CENTER_RING_DEG);
  send_wait_or_stop(VISION_CMD_FIRST_BLOCK_CODES, 20000U);
  ypcode[0] = Serial_RxPacket[0] - ASCII_0;
  ypcode[1] = Serial_RxPacket[1] - ASCII_0;
  ypcode[2] = Serial_RxPacket[2] - ASCII_0;

  set_wukuaipingtai_weizhi(PLATFORM_POS_1);
  yuantai_na((uint8_t)ypcode[0]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_2);
  yuantai_na1((uint8_t)ypcode[1]);
  send_wait_or_stop(VISION_CMD_FIRST_BLOCK_CODE1, 30000U);
  wait_packet_98();
  yuantai_na2((uint8_t)ypcode[1]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_3);
  yuantai_na1((uint8_t)ypcode[2]);
  send_wait_or_stop(VISION_CMD_FIRST_BLOCK_CODE2, 30000U);
  wait_packet_98();
  yuantai_na2((uint8_t)ypcode[2]);
}

static void race_stage_first_ring_task012(void)
{
  race_heat_set(0U);
  set_wukuaipingtai_weizhi(PLATFORM_POS_1);
  act_safe_transfer();
  set_zhuashou_kai();

  PID_move(-30, 0, 0, 400U, 0.0f, PID_PROFILE_YAW_LOW_OUTPUT);
  delay_ms1(200U);
  car_move1(0.0f, 400.0f, 150, 200);
  mypid.integral = 0.0f;
  PID_move(0, 0, 0, 1000U, 90.0f, PID_PROFILE_YAW_TRANSIT);
  PID_move(0, 0, 0, 1000U, 90.0f, PID_PROFILE_YAW_SETTLE_KI_LOW);
  car_move1(0.0f, -1700.0f, 150, 250);
  mypid.integral = 0.0f;
  PID_move(0, 0, 0, 1000U, -180.0f, PID_PROFILE_YAW_TRANSIT);
  PID_move(0, 0, 0, 1000U, -180.0f, PID_PROFILE_YAW_SETTLE_KI_LOW);

  race_heat_set(ACT_HEAT_COMPARE);
  while (1)
  {
    send_wait_or_stop(VISION_CMD_RING_COARSE, 10000U);
    transfer_position_sehuan(0x01U, 0x02U);
    car_move_distance(direct_x, direct_y, 180, 150);
    if (Serial_RxPacket[8] != VISION_RESULT_MISS) break;
  }
  PID_move(0, 0, 0, 1000U, -180.0f, PID_PROFILE_YAW_SETTLE_KI_HIGH);

  send_wait_or_stop(VISION_CMD_RING_FINE, 10000U);
  transfer_position_sehuan(0x02U, 0x02U);
  if (Serial_RxPacket[8] == VISION_RESULT_MISS) car_move_distance(0.0f, 0.0f, 70, 50);
  else car_move_distance(direct_x, direct_y, 130, 100);

  act_pick_current_platform();
  fangzhi((uint8_t)task[0]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_2);
  act_pick_current_platform();
  fangzhi((uint8_t)task[1]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_3);
  act_pick_current_platform();
  fangzhi((uint8_t)task[2]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_1);
  na((uint8_t)task[0]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_2);
  na((uint8_t)task[1]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_3);
  na((uint8_t)task[2]);
}

static void race_stage_second_ring_task012(void)
{
  set_wukuaipingtai_weizhi(PLATFORM_POS_1);
  act_safe_transfer();
  set_zhuashou_kai();
  PID_move(-30, 0, 0, 300U, -180.0f, PID_PROFILE_YAW_LOW_OUTPUT);
  PID_move(0, 0, 0, 500U, -180.0f, PID_PROFILE_YAW_SETTLE_KI_LOW);
  car_move1(140.0f, 820.0f, 180, 180);
  delay_ms1(100U);
  mypid.integral = 0.0f;
  PID_move(0, 0, 0, 1000U, 90.0f, PID_PROFILE_YAW_TRANSIT);
  PID_move(0, 0, 0, 1000U, 90.0f, PID_PROFILE_YAW_SETTLE_KI_LOW);
  car_move1(140.0f, 760.0f, 180, 150);

  race_heat_set(ACT_HEAT_COMPARE);
  while (1)
  {
    send_wait_or_stop(VISION_CMD_RING_COARSE, 10000U);
    transfer_position_sehuan(0x01U, 0x02U);
    car_move_distance(direct_x, direct_y, 180, 150);
    if (Serial_RxPacket[8] != VISION_RESULT_MISS) break;
  }
  PID_move(0, 0, 0, 1000U, 90.0f, PID_PROFILE_YAW_SETTLE_KI_HIGH);

  send_wait_or_stop(VISION_CMD_RING_FINE, 10000U);
  transfer_position_sehuan(0x02U, 0x02U);
  if (Serial_RxPacket[8] == VISION_RESULT_MISS) car_move_distance(0.0f, 0.0f, 70, 50);
  else car_move_distance(direct_x, direct_y, 130, 100);

  act_pick_current_platform();
  fangzhi((uint8_t)task[0]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_2);
  act_pick_current_platform();
  fangzhi((uint8_t)task[1]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_3);
  act_pick_current_platform();
  fangzhi((uint8_t)task[2]);
}

static void race_stage_pick_second_platform_blocks(void)
{
  set_wukuaipingtai_weizhi(PLATFORM_POS_1);
  act_safe_transfer();
  set_zhuashou_kai();
  PID_move(-30, 0, 0, 300U, 90.0f, PID_PROFILE_YAW_LOW_OUTPUT);
  delay_ms1(100U);
  car_move1(140.0f, 870.0f, 180, 180);
  mypid.integral = 0.0f;
  PID_move(0, 0, 0, 1000U, 0.0f, PID_PROFILE_YAW_TRANSIT);
  PID_move(0, 0, 0, 1000U, 0.0f, PID_PROFILE_YAW_SETTLE_KI_LOW);
  car_move1(140.0f, 390.0f, 180, 180);

  race_heat_set(ACT_HEAT_COMPARE);
  act_pose(ACT_X_SAFE, ACT_H_TRANSFER, YT_CENTER_RING_DEG);
  (void)Serial_GetRxFlag();
  Serial_SendByte(VISION_CMD_SECOND_BLOCK_LOCATE);
  PID_move(30, 0, 0, 300U, 0.0f, PID_PROFILE_YAW_LOW_OUTPUT);
  delay_ms1(100U);
  while (1)
  {
    wait_or_stop(15000U);
    transfer_position_wukuai(0x01U, 70.0f, 0x00U);
    car_move_distance(direct_x, direct_y, 100, 80);
    if (Serial_RxPacket[8] != VISION_RESULT_MISS) break;
    send_wait_or_stop(VISION_CMD_SECOND_BLOCK_LOCATE, 15000U);
  }
  act_pose(ACT_X_SAFE, ACT_H_TRANSFER, YT_CENTER_RING_DEG);
  send_wait_or_stop(VISION_CMD_SECOND_BLOCK_CODES, 20000U);
  ypcode[0] = Serial_RxPacket[0] - ASCII_0;
  ypcode[1] = Serial_RxPacket[1] - ASCII_0;
  ypcode[2] = Serial_RxPacket[2] - ASCII_0;
  set_wukuaipingtai_weizhi(PLATFORM_POS_1);
  yuantai_na((uint8_t)ypcode[0]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_2);
  yuantai_na1((uint8_t)ypcode[1]);
  send_wait_or_stop(VISION_CMD_SECOND_BLOCK_CODE1, 30000U);
  wait_packet_98();
  yuantai_na2((uint8_t)ypcode[1]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_3);
  yuantai_na1((uint8_t)ypcode[2]);
  send_wait_or_stop(VISION_CMD_SECOND_BLOCK_CODE2, 30000U);
  wait_packet_98();
  yuantai_na2((uint8_t)ypcode[2]);
}

static void race_stage_third_ring_task345(void)
{
  race_heat_set(0U);
  set_wukuaipingtai_weizhi(PLATFORM_POS_1);
  act_safe_transfer();
  set_zhuashou_kai();
  PID_move(-30, 0, 0, 400U, 0.0f, PID_PROFILE_YAW_LOW_OUTPUT);
  delay_ms1(200U);
  car_move1(0.0f, 400.0f, 150, 200);
  mypid.integral = 0.0f;
  PID_move(0, 0, 0, 1000U, 90.0f, PID_PROFILE_YAW_TRANSIT);
  PID_move(0, 0, 0, 1000U, 90.0f, PID_PROFILE_YAW_SETTLE_KI_LOW);
  car_move1(0.0f, -1700.0f, 150, 250);
  mypid.integral = 0.0f;
  PID_move(0, 0, 0, 1000U, -180.0f, PID_PROFILE_YAW_TRANSIT);
  PID_move(0, 0, 0, 1000U, -180.0f, PID_PROFILE_YAW_SETTLE_KI_LOW);

  race_heat_set(ACT_HEAT_COMPARE);
  while (1)
  {
    send_wait_or_stop(VISION_CMD_RING_COARSE, 20000U);
    transfer_position_sehuan(0x01U, 0x02U);
    car_move_distance(direct_x, direct_y, 180, 150);
    if (Serial_RxPacket[8] != VISION_RESULT_MISS) break;
  }
  PID_move(0, 0, 0, 1000U, -180.0f, PID_PROFILE_YAW_SETTLE_KI_HIGH);
  send_wait_or_stop(VISION_CMD_RING_FINE, 20000U);
  transfer_position_sehuan(0x02U, 0x02U);
  if (Serial_RxPacket[8] == VISION_RESULT_MISS) car_move_distance(0.0f, 0.0f, 70, 50);
  else car_move_distance(direct_x, direct_y, 130, 100);

  act_pick_current_platform();
  fangzhi((uint8_t)task[3]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_2);
  act_pick_current_platform();
  fangzhi((uint8_t)task[4]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_3);
  act_pick_current_platform();
  fangzhi((uint8_t)task[5]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_1);
  na((uint8_t)task[3]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_2);
  na((uint8_t)task[4]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_3);
  na((uint8_t)task[5]);
}

static void race_stage_fourth_ring_task345(void)
{
  set_wukuaipingtai_weizhi(PLATFORM_POS_1);
  act_safe_transfer();
  set_zhuashou_kai();
  PID_move(-30, 0, 0, 300U, -180.0f, PID_PROFILE_YAW_LOW_OUTPUT);
  PID_move(0, 0, 0, 500U, -180.0f, PID_PROFILE_YAW_SETTLE_KI_LOW);
  car_move1(140.0f, 820.0f, 180, 180);
  delay_ms1(100U);
  mypid.integral = 0.0f;
  PID_move(0, 0, 0, 1000U, 90.0f, PID_PROFILE_YAW_TRANSIT);
  PID_move(0, 0, 0, 1000U, 90.0f, PID_PROFILE_YAW_SETTLE_KI_LOW);
  car_move1(140.0f, 760.0f, 180, 150);

  race_heat_set(ACT_HEAT_COMPARE);
  while (1)
  {
    send_wait_or_stop(VISION_CMD_LAST_BLOCK_LOCATE, 10000U);
    transfer_position_wukuai(0x02U, 70.0f, 0x02U);
    car_move_distance(direct_x, direct_y, 180, 150);
    delay_ms1(200U);
    if (Serial_RxPacket[8] != VISION_RESULT_MISS) break;
  }
  send_wait_or_stop(VISION_CMD_LAST_BLOCK_LOCATE, 10000U);
  transfer_position_wukuai(0x02U, 70.0f, 0x02U);
  if (Serial_RxPacket[8] == VISION_RESULT_MISS) car_move_distance(0.0f, 0.0f, 70, 50);
  else car_move_distance(direct_x, direct_y, 180, 150);

  act_pick_current_platform();
  fangzhi_maduo((uint8_t)task[3]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_2);
  act_pick_current_platform();
  fangzhi_maduo((uint8_t)task[4]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_3);
  act_pick_current_platform();
  fangzhi_maduo((uint8_t)task[5]);
  set_wukuaipingtai_weizhi(PLATFORM_POS_1);
}

static void race_stage_finish_drive(void)
{
  PID_move(-30, 0, 0, 300U, 90.0f, PID_PROFILE_YAW_LOW_OUTPUT);
  car_move1(140.0f, 870.0f, 180, 180);
  act_safe_transfer();
  mypid.integral = 0.0f;
  PID_move(0, 0, 0, 1000U, 0.0f, PID_PROFILE_YAW_TRANSIT);
  PID_move(0, 0, 0, 1000U, 0.0f, PID_PROFILE_YAW_SETTLE_KI_LOW);
  car_move1(140.0f, 1850.0f, 180, 180);
  car_move2(-130.0f, 0.0f, 150, 150);
}

void race_task_run(void)
{
  race_stage_read_task_order();
  race_stage_prepare_actuators();
  race_stage_pick_first_platform_blocks();
  race_stage_first_ring_task012();
  race_stage_second_ring_task012();
  race_stage_pick_second_platform_blocks();
  race_stage_third_ring_task345();
  race_stage_fourth_ring_task345();
  race_stage_finish_drive();
  race_finish_stop();
}
