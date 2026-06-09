#include "motor_ack_test.h"

#include "hal_zdt.h"
#include "imu.h"
#include "race_chassis.h"
#include "race_io.h"
#include "tim.h"
#include "usart.h"
#include "wit_c_sdk.h"
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define TUNER_UART huart5
#define CMD_MAX_LEN 80U
#define RX_RING_SIZE 16U
#define CAN_EVENTS_PER_POLL 4U
#define MOVE_MAX_SPEED 300
#define MOVE_MAX_MS 10000
#define LLM_DEFAULT_SPEED 40
#define LLM_MIN_SPEED 0
#define LLM_MAX_SPEED 100
#define LLM_SAMPLE_MS 50U
#define PID_SCALE 1000
#define ATT_KP_MAX (6 * PID_SCALE)
#define ATT_KI_MAX (100)
#define ATT_KD_MAX (6 * PID_SCALE)
#define ATT_OUTPUT_MAX 25.0f
#define ATT_DEFAULT_KP (1100)
#define ATT_DEFAULT_KI (0)
#define ATT_DEFAULT_KD (0)
#define VEL_ACC_DEFAULT 160U
#define VEL_ACC_MAX 255U
#define SERVO_PWM_MIN 500U
#define SERVO_PWM_MAX 2500U
#define SERVO_PWM_STEP 10U
#define SERVO_STEP_DELAY_MS 20U
#define ZHUASHOU_CLOSE_DEG 25.0f
#define ZHUASHOU_OPEN_DEG 50.0f
#define SERVO_CH2_DEFAULT 862U
#define SERVO_CH3_DEFAULT 500U
#define SERVO_CH4_DEFAULT 500U
#define SHENGJIANG_MIN_POS 0
#define SHENGJIANG_MAX_POS 60
#define SHENGJIANG_MAX_SPEED 3000
#define SHENGJIANG_DEFAULT_SPEED 1000
#define SHENGJIANG_DEFAULT_ACC 120
#define SHENGJIANG_ADDR 5U
#define SHENGJIANG_UPZERO_DEFAULT_MS 500U
#define SHENGJIANG_UPZERO_MAX_MS 3000U
#define SHENGJIANG_UPZERO_DEFAULT_SPEED 30U
#define SHENGJIANG_UPZERO_DEFAULT_ACC 20U
#define PINGTUI_MIN_POS 0
#define PINGTUI_MAX_POS 65
#define PINGTUI_MAX_SPEED 3000
#define PINGTUI_DEFAULT_SPEED 1000
#define PINGTUI_DEFAULT_ACC 120
#define PINGTUI_ADDR 6U
#define ZDT_HOME_VEL 30U
#define ZDT_HOME_TIMEOUT_MS 10000U
#define ZDT_HOME_SHENGJIANG_TIMEOUT_MS 3000U
#define ZDT_HOME_SENSORLESS_VEL 300U
#define ZDT_HOME_PINGTUI_CURRENT_MA 800U
#define ZDT_HOME_PINGTUI_TIME_MS 60U
#define ZDT_HOME_SHENGJIANG_CURRENT_MA 750U
#define ZDT_HOME_SHENGJIANG_TIME_MS 30U
#define ZDT_HOME_STATUS_REPLY 0x3BU
#define ZDT_HOME_STATUS_GOING 0x04U
#define ZDT_HOME_STATUS_FAIL 0x08U
#define ZDT_STOP_REPEAT 3U
#define ZDT_STOP_DELAY_MS 5U
#define ACT_PING_SPEED 1000U
#define ACT_PING_ACC 200U
#define ACT_LIFT_SPEED 1000U
#define ACT_LIFT_ACC 200U
#define ACT_YT_SPEED 13.0f
#define ACT_YT_PLACE_RACK_DEG 0.0f
#define ACT_YT_RIGHT_RING_DEG 90.0f
#define ACT_YT_CENTER_RING_DEG 120.0f
#define ACT_YT_LEFT_RING_DEG 150.0f
#define ACT_YT_SAFE_DEG 0.0f
#define ACT_X_RACK 50.0f
#define ACT_X_PLACE 10.0f
#define ACT_H_RACK 60.0f
#define ACT_H_PLACE 60.0f
#define ACT_H_TRANSFER 10.0f
#define ACT_X_SAFE 0.0f
#define ACT_H_SAFE 0
#define ACT_HEAT_COMPARE 300U
#ifndef ACT_PREZERO_PINGTUI_PHYSICAL_HOME
#define ACT_PREZERO_PINGTUI_PHYSICAL_HOME 1U
#endif

typedef struct
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
} motor_can_frame_t;

static volatile uint8_t s_line_ready;
static volatile uint8_t s_line_len;
static volatile uint8_t s_line2_ready;
static volatile uint8_t s_line2_len;
static volatile uint8_t s_rx_pos;
static volatile uint8_t s_rx_overflow;
static volatile uint8_t s_overflow_ready;
static uint8_t s_rx_byte;
static char s_rx_line[CMD_MAX_LEN];
static char s_line[CMD_MAX_LEN];
static char s_line2[CMD_MAX_LEN];

static volatile uint8_t s_can_rx_head;
static volatile uint8_t s_can_rx_tail;
static volatile uint32_t s_can_rx_overflow;
static motor_can_frame_t s_can_rx_ring[RX_RING_SIZE];

static uint32_t s_move_stop_ms;
static uint8_t s_move_active;
static uint8_t s_vel_acc;
static uint16_t s_servo_ch2;
static uint16_t s_servo_ch3;
static uint16_t s_servo_ch4;
static uint8_t s_servo_pwm_started;
static const char *s_move_done_msg;

static uint8_t s_llm_active;
static int32_t s_llm_drive_speed;
static float s_llm_target_yaw;
static float s_llm_feedback_yaw;
static float s_llm_error;
static float s_llm_output;
static uint32_t s_llm_next_sample_ms;

static zdt_ret_t zdt_home_cfg_sensorless(uint8_t addr, uint8_t dir, uint32_t timeout_ms,
                                         uint16_t current_ma, uint16_t time_ms);

static void txs(const char *s)
{
  uint16_t len = 0U;

  if (s == 0)
  {
    return;
  }

  while (s[len] != '\0')
  {
    len++;
  }
  if (len != 0U)
  {
    (void)HAL_UART_Transmit(&TUNER_UART, (uint8_t *)s, len, 100U);
  }
}

static void txu(uint32_t value)
{
  char tmp[10];
  uint8_t n = 0U;

  if (value == 0U)
  {
    txs("0");
    return;
  }

  while (value > 0U && n < sizeof(tmp))
  {
    tmp[n++] = (char)('0' + value % 10U);
    value /= 10U;
  }
  while (n > 0U)
  {
    char c[2] = {tmp[--n], '\0'};
    txs(c);
  }
}

static void txi(int32_t value)
{
  if (value < 0)
  {
    txs("-");
    txu((uint32_t)(-value));
    return;
  }
  txu((uint32_t)value);
}

static void tx_fixed3(int32_t value)
{
  uint32_t mag;
  uint32_t whole;
  uint32_t frac;

  if (value < 0)
  {
    txs("-");
    mag = (uint32_t)(-value);
  }
  else
  {
    mag = (uint32_t)value;
  }

  whole = mag / PID_SCALE;
  frac = mag % PID_SCALE;
  txu(whole);
  txs(".");
  txs(frac < 100U ? "0" : "");
  txs(frac < 10U ? "0" : "");
  txu(frac);
}

static void tx_float3(float value)
{
  int32_t scaled;

  if (value >= 0.0f)
  {
    scaled = (int32_t)(value * (float)PID_SCALE + 0.5f);
  }
  else
  {
    scaled = (int32_t)(value * (float)PID_SCALE - 0.5f);
  }
  tx_fixed3(scaled);
}

static char hex_nibble(uint8_t value)
{
  value &= 0x0FU;
  return (value < 10U) ? (char)('0' + value) : (char)('A' + value - 10U);
}

static void tx_hex8(uint8_t value)
{
  char buf[3];

  buf[0] = hex_nibble((uint8_t)(value >> 4));
  buf[1] = hex_nibble(value);
  buf[2] = '\0';
  txs(buf);
}

static void tx_hex32(uint32_t value)
{
  txs("0x");
  tx_hex8((uint8_t)(value >> 24));
  tx_hex8((uint8_t)(value >> 16));
  tx_hex8((uint8_t)(value >> 8));
  tx_hex8((uint8_t)value);
}

static void trim_left(char **s)
{
  while (**s == ' ' || **s == '\t')
  {
    (*s)++;
  }
}

static void trim_right(char *s)
{
  uint16_t len = 0U;

  while (s[len] != '\0')
  {
    len++;
  }
  while (len > 0U && (s[len - 1U] == ' ' || s[len - 1U] == '\t'))
  {
    s[len - 1U] = '\0';
    len--;
  }
}

static bool starts_with(const char *s, const char *prefix)
{
  return strncmp(s, prefix, strlen(prefix)) == 0;
}

static bool parse_i32(char **cursor, int32_t *out)
{
  char *end;
  long value;

  trim_left(cursor);
  errno = 0;
  value = strtol(*cursor, &end, 10);
  if (end == *cursor || errno == ERANGE ||
      value < (long)INT32_MIN || value > (long)INT32_MAX)
  {
    return false;
  }

  *out = (int32_t)value;
  *cursor = end;
  return true;
}

static bool parse_float(char **cursor, float *out)
{
  char *end;
  float value;

  trim_left(cursor);
  errno = 0;
  value = strtof(*cursor, &end);
  if (end == *cursor || errno == ERANGE || !isfinite(value))
  {
    return false;
  }

  *out = value;
  *cursor = end;
  return true;
}

static bool parse_scaled(char **cursor, int32_t *out)
{
  char *s = *cursor;
  int32_t sign = 1;
  int32_t whole = 0;
  int32_t frac = 0;
  int32_t scale = PID_SCALE / 10;
  uint8_t digits = 0U;

  trim_left(&s);
  if (*s == '-')
  {
    sign = -1;
    s++;
  }
  else if (*s == '+')
  {
    s++;
  }

  while (*s >= '0' && *s <= '9')
  {
    whole = whole * 10 + (int32_t)(*s - '0');
    s++;
    digits = 1U;
  }

  if (*s == '.')
  {
    s++;
    while (*s >= '0' && *s <= '9')
    {
      if (scale > 0)
      {
        frac += (int32_t)(*s - '0') * scale;
        scale /= 10;
      }
      s++;
      digits = 1U;
    }
  }

  if (digits == 0U)
  {
    return false;
  }

  *out = sign * (whole * PID_SCALE + frac);
  *cursor = s;
  return true;
}

static uint8_t wheel_forward_dir(uint8_t id)
{
  switch (id)
  {
    case 1U:
    case 2U:
      return 0U;
    case 3U:
    case 4U:
      return 1U;
    default:
      return 0U;
  }
}

static bool speed_valid(int32_t speed)
{
  return speed >= -MOVE_MAX_SPEED && speed <= MOVE_MAX_SPEED;
}

static void wheel_vel(uint8_t id, int32_t speed)
{
  uint8_t dir = wheel_forward_dir(id);

  if (speed < 0)
  {
    dir = (uint8_t)(dir == 0U ? 1U : 0U);
    speed = -speed;
  }
  (void)zdt_vel(id, dir, (uint16_t)speed, s_vel_acc, false);
}

static void move_vel(int32_t vx, int32_t vy, int32_t w)
{
  int32_t speed[5];

  speed[1] = +vx + vy - w;
  speed[2] = -vx + vy + w;
  speed[3] = -vx + vy - w;
  speed[4] = +vx + vy + w;

  wheel_vel(2U, speed[1]);
  delay_ms1(1U);
  wheel_vel(1U, speed[2]);
  delay_ms1(1U);
  wheel_vel(3U, speed[3]);
  delay_ms1(1U);
  wheel_vel(4U, speed[4]);
  delay_ms1(1U);
}

static uint16_t servo_pulse_270(float angle)
{
  return (uint16_t)((angle / 270.0f) * 2000.0f + 500.0f);
}

static uint16_t clamp_servo_pwm_i32(int32_t value)
{
  if (value < (int32_t)SERVO_PWM_MIN)
  {
    return SERVO_PWM_MIN;
  }
  if (value > (int32_t)SERVO_PWM_MAX)
  {
    return SERVO_PWM_MAX;
  }
  return (uint16_t)value;
}

static uint16_t clamp_servo_pwm(uint16_t value)
{
  return clamp_servo_pwm_i32((int32_t)value);
}

static uint16_t yuantai_pwm_from_angle(float angle)
{
  int32_t pwm = (int32_t)(((angle - 7.9f) / 360.0f) * 2000.0f + 500.0f);
  return clamp_servo_pwm_i32(pwm);
}

static void servo_apply(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, s_servo_ch2);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, s_servo_ch3);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, s_servo_ch4);

  if (s_servo_pwm_started == 0U)
  {
    (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    s_servo_pwm_started = 1U;
  }
}

static void servo_dump(void)
{
  txs("# SERVO CH2=");
  txu(s_servo_ch2);
  txs(" CH3=");
  txu(s_servo_ch3);
  txs(" CH4=");
  txu(s_servo_ch4);
  txs("\r\n");
}

static void servo_set_pwm(uint8_t channel, uint16_t value)
{
  value = clamp_servo_pwm(value);
  if (channel == 2U)
  {
    s_servo_ch2 = value;
  }
  else if (channel == 3U)
  {
    s_servo_ch3 = value;
  }
  else if (channel == 4U)
  {
    s_servo_ch4 = value;
  }
  servo_apply();
}

static void servo_set_pwm_slow(uint8_t channel, uint16_t value)
{
  uint16_t current;

  value = clamp_servo_pwm(value);
  if (channel == 2U)
  {
    current = s_servo_ch2;
  }
  else if (channel == 3U)
  {
    current = s_servo_ch3;
  }
  else if (channel == 4U)
  {
    current = s_servo_ch4;
  }
  else
  {
    return;
  }

  while (current != value)
  {
    if (current < value)
    {
      uint16_t next = current + SERVO_PWM_STEP;
      current = (next > value) ? value : next;
    }
    else
    {
      uint16_t next = current - SERVO_PWM_STEP;
      current = (next < value) ? value : next;
    }
    servo_set_pwm(channel, current);
    delay_ms1(SERVO_STEP_DELAY_MS);
  }
}

static void emit_can_frame(const motor_can_frame_t *frame)
{
  uint32_t id;
  int32_t value;
  uint8_t i;

  id = (frame->header.IDE == CAN_ID_EXT) ? frame->header.ExtId : frame->header.StdId;
  txs("# CAN RX ");
  txs((frame->header.IDE == CAN_ID_EXT) ? "EXT id=" : "STD id=");
  tx_hex32(id);
  if (frame->header.IDE == CAN_ID_EXT)
  {
    txs(" addr=");
    txu(id >> 8);
    txs(" pack=");
    txu(id & 0xFFU);
  }
  txs(" dlc=");
  txu(frame->header.DLC);
  txs(" data=");
  for (i = 0U; i < frame->header.DLC && i < 8U; i++)
  {
    if (i != 0U)
    {
      txs(" ");
    }
    tx_hex8(frame->data[i]);
  }

  if (frame->header.DLC >= 4U && frame->data[0] == 0x35U)
  {
    value = ((int32_t)frame->data[2] << 8) | (int32_t)frame->data[3];
    if (frame->data[1] != 0U)
    {
      value = -value;
    }
    txs(" vel_rpm=");
    txi(value);
  }
  else if (frame->header.DLC >= 6U && frame->data[0] == 0x36U)
  {
    value = ((int32_t)frame->data[2] << 24) |
            ((int32_t)frame->data[3] << 16) |
            ((int32_t)frame->data[4] << 8) |
            ((int32_t)frame->data[5]);
    if (frame->data[1] != 0U)
    {
      value = -value;
    }
    txs(" pos_pul=");
    txi(value);
  }
  txs("\r\n");
}

static void update_llm_feedback(const motor_can_frame_t *frame)
{
  (void)frame;
}

static bool pop_can_frame(motor_can_frame_t *out)
{
  __disable_irq();
  if (s_can_rx_head == s_can_rx_tail)
  {
    __enable_irq();
    return false;
  }
  *out = s_can_rx_ring[s_can_rx_tail];
  s_can_rx_tail = (uint8_t)((s_can_rx_tail + 1U) % RX_RING_SIZE);
  __enable_irq();
  return true;
}

static void emit_can_events(void)
{
  motor_can_frame_t frame;
  uint8_t emitted = 0U;
  uint32_t overflow;

  __disable_irq();
  overflow = s_can_rx_overflow;
  s_can_rx_overflow = 0U;
  __enable_irq();

  if (overflow != 0U)
  {
    txs("# CAN RX overflow=");
    txu(overflow);
    txs("\r\n");
  }

  while (emitted < CAN_EVENTS_PER_POLL && pop_can_frame(&frame))
  {
    update_llm_feedback(&frame);
    emit_can_frame(&frame);
    emitted++;
  }
}

static void drain_can_for_llm(void)
{
  motor_can_frame_t frame;
  uint32_t overflow;

  __disable_irq();
  overflow = s_can_rx_overflow;
  s_can_rx_overflow = 0U;
  __enable_irq();
  (void)overflow;

  while (pop_can_frame(&frame))
  {
    update_llm_feedback(&frame);
  }
}

static void stop_all_motion(void)
{
  move_vel(0, 0, 0);
  s_move_active = 0U;
  s_llm_active = 0U;
}

static void zdt_stop_repeat(uint8_t addr)
{
  uint8_t i;

  for (i = 0U; i < ZDT_STOP_REPEAT; i++)
  {
    (void)zdt_stop(addr, false);
    delay_ms1(ZDT_STOP_DELAY_MS);
  }
}

static void zdt_home_stop_repeat(uint8_t addr)
{
  (void)zdt_home_abort(addr);
  delay_ms1(ZDT_STOP_DELAY_MS);
  zdt_stop_repeat(addr);
}

static void zdt_home_status_read(uint8_t addr)
{
  (void)zdt_home_params_read(addr);
  delay_ms1(5U);
  (void)zdt_read(addr, ZDT_S_ORG);
}

static void shengjiang_upzero_run(uint32_t ms, uint16_t speed, uint8_t acc)
{
  (void)zdt_clear_stall(SHENGJIANG_ADDR);
  delay_ms1(20U);
  (void)zdt_en(SHENGJIANG_ADDR, true, false);
  delay_ms1(50U);
  (void)zdt_vel(SHENGJIANG_ADDR, 1U, speed, acc, false);
  delay_ms1(ms);
  zdt_stop_repeat(SHENGJIANG_ADDR);
  delay_ms1(120U);
  (void)zdt_clear_pos(SHENGJIANG_ADDR);
  delay_ms1(20U);
  shengjiang_set_current_pos(0.0f);
}

static bool pop_home_status(uint8_t addr, uint8_t *status)
{
  motor_can_frame_t frame;
  uint32_t id;

  while (pop_can_frame(&frame))
  {
    id = (frame.header.IDE == CAN_ID_EXT) ? frame.header.ExtId : frame.header.StdId;
    if (frame.header.IDE == CAN_ID_EXT &&
        (uint8_t)(id >> 8) == addr &&
        frame.header.DLC >= 3U &&
        frame.data[0] == ZDT_HOME_STATUS_REPLY)
    {
      *status = frame.data[1];
      return true;
    }
  }
  return false;
}

static bool wait_home_idle(uint8_t addr, uint32_t timeout_ms, uint8_t *last_status)
{
  uint32_t start = HAL_GetTick();
  uint8_t status = ZDT_HOME_STATUS_GOING;

  *last_status = status;
  while ((uint32_t)(HAL_GetTick() - start) < timeout_ms)
  {
    (void)zdt_read(addr, ZDT_S_ORG);
    delay_ms1(80U);
    while (pop_home_status(addr, &status))
    {
      *last_status = status;
      if ((status & ZDT_HOME_STATUS_GOING) == 0U)
      {
        return true;
      }
    }
    delay_ms1(120U);
  }
  return false;
}

static bool shengjiang_homezero_run(uint8_t dir, uint16_t current_ma, uint16_t time_ms,
                                    uint32_t timeout_ms, uint8_t *status)
{
  bool idle;

  (void)zdt_clear_stall(SHENGJIANG_ADDR);
  delay_ms1(20U);
  (void)zdt_en(SHENGJIANG_ADDR, true, false);
  delay_ms1(50U);
  (void)zdt_home_cfg_sensorless(SHENGJIANG_ADDR, dir, timeout_ms, current_ma, time_ms);
  delay_ms1(20U);
  (void)zdt_home_trigger(SHENGJIANG_ADDR, 2U, false);
  idle = wait_home_idle(SHENGJIANG_ADDR, timeout_ms, status);
  zdt_home_stop_repeat(SHENGJIANG_ADDR);
  if (!idle || ((*status & ZDT_HOME_STATUS_FAIL) != 0U))
  {
    return false;
  }

  (void)zdt_clear_pos(SHENGJIANG_ADDR);
  delay_ms1(20U);
  shengjiang_set_current_pos(0.0f);
  return true;
}

static void shengjiang_homezero_default(const char *tag)
{
  bool ok;

  txs("# ");
  txs(tag);
  txs(" SJ SAFEZERO START soft_return");
  txs("\r\n");

  ok = race_shengjiang_homezero();

  txs("# ");
  txs(tag);
  txs(" SJ SAFEZERO ");
  txs(ok ? "done" : "fail");
  txs(ok ? " driver_pos=0 software_pos=0.000\r\n" : " driver_pos_unknown software_pos_kept\r\n");
}

static void pingtui_homezero_default(const char *tag)
{
  uint8_t status = 0U;
  bool ok;

  txs("# ");
  txs(tag);
  txs(" PUSH HOMEZERO START mode=2 dir=1 current=");
  txu(ZDT_HOME_PINGTUI_CURRENT_MA);
  txs(" time=");
  txu(ZDT_HOME_PINGTUI_TIME_MS);
  txs(" timeout=");
  txu(ZDT_HOME_TIMEOUT_MS);
  txs("\r\n");

  (void)zdt_clear_stall(PINGTUI_ADDR);
  delay_ms1(20U);
  (void)zdt_en(PINGTUI_ADDR, true, false);
  delay_ms1(50U);
  (void)zdt_home_cfg_sensorless(PINGTUI_ADDR, 1U,
                                ZDT_HOME_TIMEOUT_MS,
                                ZDT_HOME_PINGTUI_CURRENT_MA,
                                ZDT_HOME_PINGTUI_TIME_MS);
  delay_ms1(20U);
  (void)zdt_home_trigger(PINGTUI_ADDR, 2U, false);
  ok = wait_home_idle(PINGTUI_ADDR, ZDT_HOME_TIMEOUT_MS, &status);
  zdt_home_stop_repeat(PINGTUI_ADDR);
  if (!ok || ((status & ZDT_HOME_STATUS_FAIL) != 0U))
  {
    ok = false;
  }
  else
  {
    delay_ms1(120U);
    (void)zdt_clear_pos(PINGTUI_ADDR);
    delay_ms1(20U);
    pingtui_set_current_pos(0.0f);
  }

  txs("# ");
  txs(tag);
  txs(" PUSH HOMEZERO ");
  txs(ok ? "done" : "timeout_or_fail");
  txs(" status=");
  tx_hex8(status);
  txs(ok ? " driver_pos=0 software_pos=0.000\r\n" : " driver_pos_unknown software_pos_kept\r\n");
}

static void act_pingtui_prezero(const char *tag)
{
#if ACT_PREZERO_PINGTUI_PHYSICAL_HOME
  pingtui_homezero_default(tag);
#else
  double wait_ms;

  txs("# ");
  txs(tag);
  txs(" PUSH SAFEZERO START soft_return\r\n");
  wait_ms = pingtui_control(0.0f, ACT_PING_SPEED, ACT_PING_ACC);
  delay_ms1((uint32_t)wait_ms + 200U);
  race_pingtui_clear_zero();
  txs("# ");
  txs(tag);
  txs(" PUSH SAFEZERO done driver_pos=0 software_pos=0.000\r\n");
#endif
}

static zdt_ret_t zdt_home_cfg_sensorless(uint8_t addr, uint8_t dir, uint32_t timeout_ms,
                                         uint16_t current_ma, uint16_t time_ms)
{
  return zdt_home_params_write(addr, false, 2U, dir, ZDT_HOME_VEL, timeout_ms,
                               ZDT_HOME_SENSORLESS_VEL, current_ma, time_ms, false);
}

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static void llm_reset_state(void)
{
  mypid.error = 0.0f;
  mypid.lastError = 0.0f;
  mypid.integral = 0.0f;
  mypid.output = 0.0f;
  mypid.maxIntegral = 7.0f;
  mypid.maxOutput = ATT_OUTPUT_MAX;
  s_llm_feedback_yaw = fAngle[2];
  s_llm_error = 0.0f;
  s_llm_output = 0.0f;
}

static void emit_llm_csv(void)
{
  txu(HAL_GetTick());
  txs(",");
  tx_float3(s_llm_target_yaw);
  txs(",");
  tx_float3(s_llm_feedback_yaw);
  txs(",");
  tx_float3(s_llm_output);
  txs(",");
  tx_float3(s_llm_error);
  txs(",");
  tx_float3(mypid.kp);
  txs(",");
  tx_float3(mypid.ki);
  txs(",");
  tx_float3(mypid.kd);
  txs("\r\n");
}

static float normalize_yaw(float yaw)
{
  while (yaw > 180.0f)
  {
    yaw -= 360.0f;
  }
  while (yaw < -180.0f)
  {
    yaw += 360.0f;
  }
  return yaw;
}

static bool imu_data_ready(void)
{
  if (imu_rx_count() < 100U || imu_update_count() < 10U)
  {
    return false;
  }

  return imu_angle_update_count() > 0U;
}

static void llm_apply_control(void)
{
  imu_scan(fAcc, fGyro, fAngle);
  s_llm_feedback_yaw = fAngle[2];
  PID_Calc(&mypid, s_llm_target_yaw, s_llm_feedback_yaw);
  s_llm_error = mypid.error;
  s_llm_output = mypid.output;
  move_vel(0, s_llm_drive_speed, (int32_t)s_llm_output);
  emit_llm_csv();
}

static void llm_poll(uint32_t now)
{
  if (s_llm_active == 0U)
  {
    return;
  }

  drain_can_for_llm();
  if ((int32_t)(now - s_llm_next_sample_ms) >= 0)
  {
    llm_apply_control();
    s_llm_next_sample_ms = now + LLM_SAMPLE_MS;
  }
}

static void handle_enc(char *params)
{
  uint8_t id;

  trim_left(&params);
  if (*params != '\0')
  {
    txs("# ERROR: ENC format invalid\r\n");
    return;
  }

  txs("# ENC query\r\n");
  for (id = 1U; id <= 4U; id++)
  {
    (void)zdt_read(id, ZDT_S_VEL);
    (void)zdt_read(id, ZDT_S_CPOS);
  }
}

static void handle_imu(char *params)
{
  trim_left(&params);
  if (*params != '\0')
  {
    txs("# ERROR: IMU format invalid\r\n");
    return;
  }

  imu_scan(fAcc, fGyro, fAngle);
  txs("# IMU rx=");
  txu(imu_rx_count());
  txs(" update=");
  txu(imu_update_count());
  txs(" angle_update=");
  txu(imu_angle_update_count());
  txs(" acc=");
  tx_float3(fAcc[0]);
  txs(",");
  tx_float3(fAcc[1]);
  txs(",");
  tx_float3(fAcc[2]);
  txs(" gyro=");
  tx_float3(fGyro[0]);
  txs(",");
  tx_float3(fGyro[1]);
  txs(",");
  tx_float3(fGyro[2]);
  txs(" angle=");
  tx_float3(fAngle[0]);
  txs(",");
  tx_float3(fAngle[1]);
  txs(",");
  tx_float3(fAngle[2]);
  txs("\r\n");
}

static void handle_imu_raw(char *params)
{
  trim_left(&params);
  if (*params != '\0')
  {
    txs("# ERROR: IMURAW format invalid\r\n");
    return;
  }

  txs("# IMURAW ax=");
  txi(sReg[AX]);
  txs(" ay=");
  txi(sReg[AY]);
  txs(" az=");
  txi(sReg[AZ]);
  txs(" gx=");
  txi(sReg[GX]);
  txs(" gy=");
  txi(sReg[GY]);
  txs(" gz=");
  txi(sReg[GZ]);
  txs(" roll=");
  txi(sReg[Roll]);
  txs(" pitch=");
  txi(sReg[Pitch]);
  txs(" yaw=");
  txi(sReg[Yaw]);
  txs("\r\n");
}

static void handle_imu_cfg(char *params)
{
  int32_t ret;

  trim_left(&params);
  if (*params != '\0')
  {
    txs("# ERROR: IMUCFG format invalid\r\n");
    return;
  }

  ret = WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE);
  txs("# IMUCFG ret=");
  txi(ret);
  txs(" content=ACC,GYRO,ANGLE\r\n");
}

static void handle_imu_read(char *params)
{
  int32_t ret_acc;
  int32_t ret_gyro;
  int32_t ret_angle;

  trim_left(&params);
  if (*params != '\0')
  {
    txs("# ERROR: IMUREAD format invalid\r\n");
    return;
  }

  ret_acc = WitReadReg(AX, 3U);
  delay_ms1(50U);
  ret_gyro = WitReadReg(GX, 3U);
  delay_ms1(50U);
  ret_angle = WitReadReg(Roll, 3U);
  delay_ms1(50U);
  imu_scan(fAcc, fGyro, fAngle);

  txs("# IMUREAD ret=");
  txi(ret_acc);
  txs(",");
  txi(ret_gyro);
  txs(",");
  txi(ret_angle);
  txs("\r\n");
  handle_imu_raw("");
  handle_imu("");
}

static void handle_wheel(char *params)
{
  char *cursor = params;
  int32_t id;
  int32_t speed;
  int32_t time_ms;
  uint8_t dir;
  char fb;

  if (!parse_i32(&cursor, &id))
  {
    txs("# ERROR: WHEEL format invalid\r\n");
    return;
  }
  trim_left(&cursor);
  fb = *cursor++;
  if (!parse_i32(&cursor, &speed) || !parse_i32(&cursor, &time_ms))
  {
    txs("# ERROR: WHEEL format invalid\r\n");
    return;
  }
  trim_left(&cursor);
  if (*cursor != '\0' || id < 1 || id > 4 || speed < 1 || speed > MOVE_MAX_SPEED ||
      time_ms < 50 || time_ms > MOVE_MAX_MS)
  {
    txs("# ERROR: WHEEL params rejected\r\n");
    return;
  }

  if (fb == 'F' || fb == 'f')
  {
    dir = wheel_forward_dir((uint8_t)id);
  }
  else if (fb == 'B' || fb == 'b')
  {
    dir = (uint8_t)(wheel_forward_dir((uint8_t)id) == 0U ? 1U : 0U);
  }
  else
  {
    txs("# ERROR: WHEEL format invalid\r\n");
    return;
  }

  (void)zdt_vel((uint8_t)id, dir, (uint16_t)speed, 230U, false);
  s_move_stop_ms = HAL_GetTick() + (uint32_t)time_ms;
  s_move_active = 1U;
  s_move_done_msg = "# WHEEL done\r\n";
  txs("# WHEEL start\r\n");
}

static void handle_move(char *params)
{
  char *cursor = params;
  int32_t vx;
  int32_t vy;
  int32_t w;
  int32_t time_ms;

  if (!parse_i32(&cursor, &vx) || !parse_i32(&cursor, &vy) ||
      !parse_i32(&cursor, &w) || !parse_i32(&cursor, &time_ms))
  {
    txs("# ERROR: MOVE format invalid\r\n");
    return;
  }
  trim_left(&cursor);
  if (*cursor != '\0' || !speed_valid(vx) || !speed_valid(vy) || !speed_valid(w) ||
      time_ms < 50 || time_ms > MOVE_MAX_MS)
  {
    txs("# ERROR: MOVE params rejected\r\n");
    return;
  }

  stop_all_motion();
  move_vel(vx, vy, w);
  s_llm_active = 0U;
  s_move_stop_ms = HAL_GetTick() + (uint32_t)time_ms;
  s_move_active = 1U;
  s_move_done_msg = "# MOVE done\r\n";
  txs("# MOVE start\r\n");
}

static bool parse_pulse_params(char *params, int32_t *pulses, int32_t *speed, int32_t *acc, char *fb)
{
  char *cursor = params;

  trim_left(&cursor);
  *fb = *cursor++;
  if (*fb == '\0' || !parse_i32(&cursor, pulses) ||
      !parse_i32(&cursor, speed) || !parse_i32(&cursor, acc))
  {
    return false;
  }
  trim_left(&cursor);
  return *cursor == '\0' &&
         (*fb == 'F' || *fb == 'f' || *fb == 'B' || *fb == 'b') &&
         *pulses >= 1 && *pulses <= 200000 &&
         *speed >= 1 && *speed <= MOVE_MAX_SPEED &&
         *acc >= 0 && *acc <= 255;
}

static void handle_pulse(char *params)
{
  int32_t pulses;
  int32_t speed;
  int32_t acc;
  uint8_t id;
  uint8_t dir;
  char fb;

  if (!parse_pulse_params(params, &pulses, &speed, &acc, &fb))
  {
    txs("# ERROR: PULSE format invalid\r\n");
    return;
  }

  stop_all_motion();
  txs("# PULSE start\r\n");
  for (id = 1U; id <= 4U; id++)
  {
    dir = wheel_forward_dir(id);
    if (fb == 'B' || fb == 'b')
    {
      dir = (uint8_t)(dir == 0U ? 1U : 0U);
    }
    (void)zdt_pos(id, dir, (uint16_t)speed, (uint8_t)acc, (uint32_t)pulses, false, true);
    delay_ms1(5U);
  }
  (void)zdt_sync(0U);
}

static void handle_pulse_one(char *params)
{
  char *cursor = params;
  int32_t id;
  int32_t pulses;
  int32_t speed;
  int32_t acc;
  uint8_t dir;
  char fb;

  if (!parse_i32(&cursor, &id))
  {
    txs("# ERROR: PULSE1 format invalid\r\n");
    return;
  }
  trim_left(&cursor);
  fb = *cursor++;
  if (!parse_i32(&cursor, &pulses) || !parse_i32(&cursor, &speed) || !parse_i32(&cursor, &acc))
  {
    txs("# ERROR: PULSE1 format invalid\r\n");
    return;
  }
  trim_left(&cursor);
  if (*cursor != '\0' || id < 1 || id > 4 ||
      pulses < 1 || pulses > 200000 || speed < 1 || speed > MOVE_MAX_SPEED ||
      acc < 0 || acc > 255 || (fb != 'F' && fb != 'f' && fb != 'B' && fb != 'b'))
  {
    txs("# ERROR: PULSE1 params rejected\r\n");
    return;
  }

  dir = wheel_forward_dir((uint8_t)id);
  if (fb == 'B' || fb == 'b')
  {
    dir = (uint8_t)(dir == 0U ? 1U : 0U);
  }

  stop_all_motion();
  txs("# PULSE1 start\r\n");
  (void)zdt_pos((uint8_t)id, dir, (uint16_t)speed, (uint8_t)acc, (uint32_t)pulses, false, false);
}

static bool parse_set_pid(char *params, int32_t *kp, int32_t *ki, int32_t *kd)
{
  char *cursor = params;
  uint8_t got_p = 0U;
  uint8_t got_i = 0U;
  uint8_t got_d = 0U;

  while (1)
  {
    trim_left(&cursor);
    if (*cursor == '\0')
    {
      break;
    }

    if ((cursor[0] == 'P' || cursor[0] == 'p') && cursor[1] == ':')
    {
      cursor += 2;
      if (!parse_scaled(&cursor, kp))
      {
        return false;
      }
      got_p = 1U;
    }
    else if ((cursor[0] == 'I' || cursor[0] == 'i') && cursor[1] == ':')
    {
      cursor += 2;
      if (!parse_scaled(&cursor, ki))
      {
        return false;
      }
      got_i = 1U;
    }
    else if ((cursor[0] == 'D' || cursor[0] == 'd') && cursor[1] == ':')
    {
      cursor += 2;
      if (!parse_scaled(&cursor, kd))
      {
        return false;
      }
      got_d = 1U;
    }
    else if ((cursor[0] == 'K' || cursor[0] == 'k') &&
             (cursor[1] == 'P' || cursor[1] == 'p') && cursor[2] == ':')
    {
      cursor += 3;
      if (!parse_scaled(&cursor, kp))
      {
        return false;
      }
      got_p = 1U;
    }
    else if ((cursor[0] == 'K' || cursor[0] == 'k') &&
             (cursor[1] == 'I' || cursor[1] == 'i') && cursor[2] == ':')
    {
      cursor += 3;
      if (!parse_scaled(&cursor, ki))
      {
        return false;
      }
      got_i = 1U;
    }
    else if ((cursor[0] == 'K' || cursor[0] == 'k') &&
             (cursor[1] == 'D' || cursor[1] == 'd') && cursor[2] == ':')
    {
      cursor += 3;
      if (!parse_scaled(&cursor, kd))
      {
        return false;
      }
      got_d = 1U;
    }
    else
    {
      return false;
    }
  }

  return got_p != 0U && got_i != 0U && got_d != 0U;
}

static void apply_llm_pid(int32_t kp, int32_t ki, int32_t kd)
{
  kp = clamp_i32(kp, 0, ATT_KP_MAX);
  ki = clamp_i32(ki, 0, ATT_KI_MAX);
  kd = clamp_i32(kd, 0, ATT_KD_MAX);
  mypid.kp = (float)kp / (float)PID_SCALE;
  mypid.ki = (float)ki / (float)PID_SCALE;
  mypid.kd = (float)kd / (float)PID_SCALE;
  llm_reset_state();
}

static void apply_default_llm_pid(void)
{
  apply_llm_pid(ATT_DEFAULT_KP, ATT_DEFAULT_KI, ATT_DEFAULT_KD);
}

static void apply_default_vel_acc(void)
{
  s_vel_acc = VEL_ACC_DEFAULT;
}

static void emit_status(void)
{
  txs("# MOTOR TEST Ready UART5 115200 mode=");
  txs((s_llm_active != 0U) ? "LLM" : "MANUAL");
  txs(" target_yaw=");
  tx_float3(s_llm_target_yaw);
  txs(" yaw=");
  tx_float3(s_llm_feedback_yaw);
  txs(" speed=");
  txi(s_llm_drive_speed);
  txs(" acc=");
  txu(s_vel_acc);
  txs(" pid=");
  tx_float3(mypid.kp);
  txs(",");
  tx_float3(mypid.ki);
  txs(",");
  tx_float3(mypid.kd);
  txs(" output=");
  tx_float3(s_llm_output);
  txs("\r\n");
}

static void handle_set(char *params)
{
  int32_t kp = (int32_t)(mypid.kp * (float)PID_SCALE);
  int32_t ki = (int32_t)(mypid.ki * (float)PID_SCALE);
  int32_t kd = (int32_t)(mypid.kd * (float)PID_SCALE);

  if (!parse_set_pid(params, &kp, &ki, &kd))
  {
    txs("# ERROR: SET format invalid\r\n");
    return;
  }
  apply_llm_pid(kp, ki, kd);
  txs("# SET OK P:");
  tx_float3(mypid.kp);
  txs(" I:");
  tx_float3(mypid.ki);
  txs(" D:");
  tx_float3(mypid.kd);
  txs("\r\n");
  emit_llm_csv();
}

static void handle_acc(char *params)
{
  char *cursor = params;
  int32_t acc;

  if (!parse_i32(&cursor, &acc))
  {
    txs("# ERROR: ACC format invalid\r\n");
    return;
  }
  trim_left(&cursor);
  if (*cursor != '\0' || acc < 0 || acc > (int32_t)VEL_ACC_MAX)
  {
    txs("# ERROR: ACC params rejected\r\n");
    return;
  }
  s_vel_acc = (uint8_t)acc;
  txs("# ACC OK value=");
  txu(s_vel_acc);
  txs("\r\n");
}

static void handle_servo(char *params)
{
  char *cursor = params;
  int32_t channel;
  int32_t pwm;

  trim_left(&cursor);
  if (*cursor == '\0')
  {
    servo_dump();
    return;
  }

  if (!parse_i32(&cursor, &channel) || !parse_i32(&cursor, &pwm))
  {
    txs("# ERROR: SERVO format invalid\r\n");
    return;
  }
  trim_left(&cursor);
  if (*cursor != '\0' || channel < 2 || channel > 4 ||
      pwm < (int32_t)SERVO_PWM_MIN || pwm > (int32_t)SERVO_PWM_MAX)
  {
    txs("# ERROR: SERVO params rejected\r\n");
    return;
  }

  servo_set_pwm((uint8_t)channel, (uint16_t)pwm);
  servo_dump();
}

static void handle_angle_servo(char *params, char kind)
{
  char *cursor = params;
  float angle;
  uint16_t pwm;

  if (!parse_float(&cursor, &angle))
  {
    txs("# ERROR: angle format invalid\r\n");
    return;
  }
  trim_left(&cursor);
  if (*cursor != '\0')
  {
    txs("# ERROR: angle format invalid\r\n");
    return;
  }

  if (kind == 'Y')
  {
    pwm = yuantai_pwm_from_angle(angle);
    servo_set_pwm_slow(4U, pwm);
    txs("# YT angle=");
  }
  else if (kind == 'P')
  {
    pwm = servo_pulse_270(angle);
    servo_set_pwm_slow(3U, pwm);
    txs("# PT angle=");
  }
  else
  {
    pwm = servo_pulse_270(angle - 1.0f);
    servo_set_pwm(2U, pwm);
    txs("# ZH angle=");
  }
  tx_float3(angle);
  txs(" pwm=");
  txu(pwm);
  txs("\r\n");
  servo_dump();
}

static void handle_pid(char *params)
{
  char *cursor = params;
  int32_t kp;
  int32_t ki;
  int32_t kd;

  if (!parse_scaled(&cursor, &kp) || !parse_scaled(&cursor, &ki) || !parse_scaled(&cursor, &kd))
  {
    txs("# ERROR: PID format invalid\r\n");
    return;
  }
  trim_left(&cursor);
  if (*cursor != '\0')
  {
    txs("# ERROR: PID format invalid\r\n");
    return;
  }
  apply_llm_pid(kp, ki, kd);
  txs("# PID OK\r\n");
  emit_llm_csv();
}

static void handle_shengjiang(char *params)
{
  char *cursor = params;
  int32_t pos;
  int32_t speed = SHENGJIANG_DEFAULT_SPEED;
  int32_t acc = SHENGJIANG_DEFAULT_ACC;
  double wait_ms;

  trim_left(&cursor);
  if (*cursor == '\0' || *cursor == '?')
  {
    txs("# SJ pos=");
    tx_float3(shengjiang_current_pos());
    txs(" range=0..135 default_speed=");
    txu(SHENGJIANG_DEFAULT_SPEED);
    txs(" default_acc=");
    txu(SHENGJIANG_DEFAULT_ACC);
    txs(" home=UP/UPZERO/HOMEZERO/HCFG/HSTAT/HOME/HOME3/HSTOP/CLEAR/UNSTALL\r\n");
    return;
  }

  if (starts_with(cursor, "STOP"))
  {
    cursor += 4;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: SJ format invalid\r\n");
      return;
    }
    zdt_home_stop_repeat(SHENGJIANG_ADDR);
    txs("# SJ STOP addr=5 home_abort=1 repeat=3\r\n");
    return;
  }

  if (starts_with(cursor, "EN"))
  {
    cursor += 2;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: SJ format invalid\r\n");
      return;
    }
    (void)zdt_en(SHENGJIANG_ADDR, true, false);
    txs("# SJ EN addr=5\r\n");
    return;
  }

  if (starts_with(cursor, "DIS"))
  {
    cursor += 3;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: SJ format invalid\r\n");
      return;
    }
    (void)zdt_en(SHENGJIANG_ADDR, false, false);
    txs("# SJ DIS addr=5\r\n");
    return;
  }

  if (starts_with(cursor, "READ"))
  {
    cursor += 4;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: SJ format invalid\r\n");
      return;
    }
    (void)zdt_read(SHENGJIANG_ADDR, ZDT_S_CPOS);
    delay_ms1(5U);
    (void)zdt_read(SHENGJIANG_ADDR, ZDT_S_VEL);
    delay_ms1(5U);
    (void)zdt_read(SHENGJIANG_ADDR, ZDT_S_FLAG);
    txs("# SJ READ addr=5\r\n");
    return;
  }

  if (starts_with(cursor, "HSTAT"))
  {
    cursor += 5;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: SJ format invalid\r\n");
      return;
    }
    zdt_home_status_read(SHENGJIANG_ADDR);
    txs("# SJ HSTAT addr=5 read=0x22,0x3B\r\n");
    return;
  }

  if (starts_with(cursor, "HOMEZERO"))
  {
    int32_t dir = 1;
    int32_t current_ma = ZDT_HOME_SHENGJIANG_CURRENT_MA;
    int32_t time_ms = ZDT_HOME_SHENGJIANG_TIME_MS;
    int32_t timeout_ms = ZDT_HOME_SHENGJIANG_TIMEOUT_MS;
    uint8_t status = 0U;
    bool ok;

    cursor += 8;
    (void)parse_i32(&cursor, &dir);
    (void)parse_i32(&cursor, &current_ma);
    (void)parse_i32(&cursor, &time_ms);
    (void)parse_i32(&cursor, &timeout_ms);
    trim_left(&cursor);
    if (*cursor != '\0' || dir < 0 || dir > 1 ||
        current_ma < 400 || current_ma > 3000 ||
        time_ms < 20 || time_ms > 200 ||
        timeout_ms < 1000 || timeout_ms > 30000)
    {
      txs("# ERROR: SJ HOMEZERO params rejected\r\n");
      return;
    }
    txs("# SJ HOMEZERO START mode=2 dir=");
    txi(dir);
    txs(" current=");
    txi(current_ma);
    txs(" time=");
    txi(time_ms);
    txs(" timeout=");
    txi(timeout_ms);
    txs("\r\n");
    ok = shengjiang_homezero_run((uint8_t)dir, (uint16_t)current_ma,
                                 (uint16_t)time_ms, (uint32_t)timeout_ms, &status);
    txs("# SJ HOMEZERO ");
    txs(ok ? "done" : "timeout_or_fail");
    txs(" status=");
    tx_hex8(status);
    txs(ok ? " driver_pos=0 software_pos=0.000\r\n" : " driver_pos_unknown software_pos_kept\r\n");
    return;
  }

  if (starts_with(cursor, "HCFG"))
  {
    int32_t dir;
    int32_t current_ma = ZDT_HOME_SHENGJIANG_CURRENT_MA;
    int32_t time_ms = ZDT_HOME_SHENGJIANG_TIME_MS;

    cursor += 4;
    if (!parse_i32(&cursor, &dir))
    {
      txs("# ERROR: SJ HCFG format invalid\r\n");
      return;
    }
    (void)parse_i32(&cursor, &current_ma);
    (void)parse_i32(&cursor, &time_ms);
    trim_left(&cursor);
    if (*cursor != '\0' || dir < 0 || dir > 1 ||
        current_ma < 400 || current_ma > 3000 ||
        time_ms < 20 || time_ms > 200)
    {
      txs("# ERROR: SJ HCFG params rejected\r\n");
      return;
    }
    (void)zdt_home_cfg_sensorless(SHENGJIANG_ADDR, (uint8_t)dir,
                                  ZDT_HOME_SHENGJIANG_TIMEOUT_MS,
                                  (uint16_t)current_ma,
                                  (uint16_t)time_ms);
    txs("# SJ HCFG addr=5 mode=2 dir=");
    txi(dir);
    txs(" current_ma=");
    txu((uint32_t)current_ma);
    txs(" time_ms=");
    txu((uint32_t)time_ms);
    txs(" save=0 auto=0\r\n");
    return;
  }

  if (starts_with(cursor, "HSTOP"))
  {
    cursor += 5;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: SJ format invalid\r\n");
      return;
    }
    zdt_home_stop_repeat(SHENGJIANG_ADDR);
    txs("# SJ HSTOP addr=5 home_abort=1 repeat=3\r\n");
    return;
  }

  if (starts_with(cursor, "UPZERO") || starts_with(cursor, "UP"))
  {
    bool clear_after;
    int32_t ms = (int32_t)SHENGJIANG_UPZERO_DEFAULT_MS;
    int32_t speed = (int32_t)SHENGJIANG_UPZERO_DEFAULT_SPEED;
    int32_t acc = (int32_t)SHENGJIANG_UPZERO_DEFAULT_ACC;

    clear_after = starts_with(cursor, "UPZERO");
    cursor += clear_after ? 6 : 2;
    (void)parse_i32(&cursor, &ms);
    (void)parse_i32(&cursor, &speed);
    (void)parse_i32(&cursor, &acc);
    trim_left(&cursor);
    if (*cursor != '\0' ||
        ms < 50 || ms > (int32_t)SHENGJIANG_UPZERO_MAX_MS ||
        speed < 1 || speed > SHENGJIANG_MAX_SPEED ||
        acc < 0 || acc > 255)
    {
      txs("# ERROR: SJ UP params rejected\r\n");
      return;
    }

    if (clear_after)
    {
      shengjiang_upzero_run((uint32_t)ms, (uint16_t)speed, (uint8_t)acc);
    }
    else
    {
      (void)zdt_en(SHENGJIANG_ADDR, true, false);
      delay_ms1(5U);
      (void)zdt_vel(SHENGJIANG_ADDR, 1U, (uint16_t)speed, (uint8_t)acc, false);
      delay_ms1((uint32_t)ms);
      zdt_stop_repeat(SHENGJIANG_ADDR);
    }

    txs("# SJ ");
    txs(clear_after ? "UPZERO" : "UP");
    txs(" dir=1 ms=");
    txu((uint32_t)ms);
    txs(" speed=");
    txu((uint32_t)speed);
    txs(" acc=");
    txu((uint32_t)acc);
    if (clear_after)
    {
      txs(" driver_pos=0 software_pos=0.000");
    }
    txs("\r\n");
    return;
  }

  if (starts_with(cursor, "HOME3"))
  {
    cursor += 5;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: SJ format invalid\r\n");
      return;
    }
    (void)zdt_home_trigger(SHENGJIANG_ADDR, 3U, false);
    txs("# SJ HOME3 addr=5 mode=3 limit-switch\r\n");
    return;
  }

  if (starts_with(cursor, "HOME"))
  {
    cursor += 4;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: SJ format invalid\r\n");
      return;
    }
    (void)zdt_home_trigger(SHENGJIANG_ADDR, 2U, false);
    txs("# SJ HOME addr=5 mode=2 sensorless\r\n");
    return;
  }

  if (starts_with(cursor, "CLEAR"))
  {
    cursor += 5;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: SJ format invalid\r\n");
      return;
    }
    (void)zdt_clear_pos(SHENGJIANG_ADDR);
    shengjiang_set_current_pos(0.0f);
    txs("# SJ CLEAR addr=5 driver_pos=0 software_pos=0.000\r\n");
    return;
  }

  if (starts_with(cursor, "UNSTALL"))
  {
    cursor += 7;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: SJ format invalid\r\n");
      return;
    }
    (void)zdt_clear_stall(SHENGJIANG_ADDR);
    txs("# SJ UNSTALL addr=5\r\n");
    return;
  }

  if (starts_with(cursor, "ZERO"))
  {
    cursor += 4;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: SJ format invalid\r\n");
      return;
    }
    shengjiang_set_current_pos(0.0f);
    txs("# SJ ZERO pos=0.000\r\n");
    return;
  }

  if (starts_with(cursor, "SET"))
  {
    cursor += 3;
    if (!parse_i32(&cursor, &pos))
    {
      txs("# ERROR: SJ SET format invalid\r\n");
      return;
    }
    trim_left(&cursor);
    if (*cursor != '\0' || pos < SHENGJIANG_MIN_POS || pos > SHENGJIANG_MAX_POS)
    {
      txs("# ERROR: SJ SET params rejected\r\n");
      return;
    }
    shengjiang_set_current_pos((float)pos);
    txs("# SJ SET pos=");
    tx_float3(shengjiang_current_pos());
    txs("\r\n");
    return;
  }

  if (!parse_i32(&cursor, &pos))
  {
    txs("# ERROR: SJ format invalid\r\n");
    return;
  }
  (void)parse_i32(&cursor, &speed);
  (void)parse_i32(&cursor, &acc);
  trim_left(&cursor);
  if (*cursor != '\0' ||
      pos < SHENGJIANG_MIN_POS || pos > SHENGJIANG_MAX_POS ||
      speed < 1 || speed > SHENGJIANG_MAX_SPEED ||
      acc < 0 || acc > 255)
  {
    txs("# ERROR: SJ params rejected\r\n");
    return;
  }

  wait_ms = shengjiang_control((int)pos, (uint16_t)speed, (uint8_t)acc);
  txs("# SJ target=");
  txi(pos);
  txs(" speed=");
  txi(speed);
  txs(" acc=");
  txi(acc);
  txs(" pos=");
  tx_float3(shengjiang_current_pos());
  txs(" wait_ms=");
  txu((uint32_t)wait_ms);
  txs("\r\n");
}

static void handle_pingtui(char *params)
{
  char *cursor = params;
  int32_t pos;
  int32_t speed = PINGTUI_DEFAULT_SPEED;
  int32_t acc = PINGTUI_DEFAULT_ACC;
  double wait_ms;

  trim_left(&cursor);
  if (*cursor == '\0' || *cursor == '?')
  {
    txs("# PUSH pos=");
    tx_float3(pingtui_current_pos());
    txs(" range=0..65 default_speed=");
    txu(PINGTUI_DEFAULT_SPEED);
    txs(" default_acc=");
    txu(PINGTUI_DEFAULT_ACC);
    txs(" home=HCFG/HSTAT/HOME/HOME3/HSTOP/CLEAR/UNSTALL\r\n");
    return;
  }

  if (starts_with(cursor, "STOP"))
  {
    cursor += 4;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: PUSH format invalid\r\n");
      return;
    }
    zdt_home_stop_repeat(PINGTUI_ADDR);
    txs("# PUSH STOP addr=6 home_abort=1 repeat=3\r\n");
    return;
  }

  if (starts_with(cursor, "EN"))
  {
    cursor += 2;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: PUSH format invalid\r\n");
      return;
    }
    (void)zdt_en(PINGTUI_ADDR, true, false);
    txs("# PUSH EN addr=6\r\n");
    return;
  }

  if (starts_with(cursor, "DIS"))
  {
    cursor += 3;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: PUSH format invalid\r\n");
      return;
    }
    (void)zdt_en(PINGTUI_ADDR, false, false);
    txs("# PUSH DIS addr=6\r\n");
    return;
  }

  if (starts_with(cursor, "READ"))
  {
    cursor += 4;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: PUSH format invalid\r\n");
      return;
    }
    (void)zdt_read(PINGTUI_ADDR, ZDT_S_CPOS);
    delay_ms1(5U);
    (void)zdt_read(PINGTUI_ADDR, ZDT_S_VEL);
    delay_ms1(5U);
    (void)zdt_read(PINGTUI_ADDR, ZDT_S_FLAG);
    txs("# PUSH READ addr=6\r\n");
    return;
  }

  if (starts_with(cursor, "HSTAT"))
  {
    cursor += 5;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: PUSH format invalid\r\n");
      return;
    }
    zdt_home_status_read(PINGTUI_ADDR);
    txs("# PUSH HSTAT addr=6 read=0x22,0x3B\r\n");
    return;
  }

  if (starts_with(cursor, "HCFG"))
  {
    int32_t dir;
    int32_t current_ma = ZDT_HOME_PINGTUI_CURRENT_MA;
    int32_t time_ms = ZDT_HOME_PINGTUI_TIME_MS;

    cursor += 4;
    if (!parse_i32(&cursor, &dir))
    {
      txs("# ERROR: PUSH HCFG format invalid\r\n");
      return;
    }
    (void)parse_i32(&cursor, &current_ma);
    (void)parse_i32(&cursor, &time_ms);
    trim_left(&cursor);
    if (*cursor != '\0' || dir < 0 || dir > 1 ||
        current_ma < 400 || current_ma > 3000 ||
        time_ms < 20 || time_ms > 200)
    {
      txs("# ERROR: PUSH HCFG params rejected\r\n");
      return;
    }
    (void)zdt_home_cfg_sensorless(PINGTUI_ADDR, (uint8_t)dir,
                                  ZDT_HOME_TIMEOUT_MS,
                                  (uint16_t)current_ma,
                                  (uint16_t)time_ms);
    txs("# PUSH HCFG addr=6 mode=2 dir=");
    txi(dir);
    txs(" current_ma=");
    txu((uint32_t)current_ma);
    txs(" time_ms=");
    txu((uint32_t)time_ms);
    txs(" save=0 auto=0\r\n");
    return;
  }

  if (starts_with(cursor, "HSTOP"))
  {
    cursor += 5;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: PUSH format invalid\r\n");
      return;
    }
    zdt_home_stop_repeat(PINGTUI_ADDR);
    txs("# PUSH HSTOP addr=6 home_abort=1 repeat=3\r\n");
    return;
  }

  if (starts_with(cursor, "HOME3"))
  {
    cursor += 5;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: PUSH format invalid\r\n");
      return;
    }
    (void)zdt_home_trigger(PINGTUI_ADDR, 3U, false);
    txs("# PUSH HOME3 addr=6 mode=3 limit-switch\r\n");
    return;
  }

  if (starts_with(cursor, "HOME"))
  {
    cursor += 4;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: PUSH format invalid\r\n");
      return;
    }
    (void)zdt_home_trigger(PINGTUI_ADDR, 2U, false);
    txs("# PUSH HOME addr=6 mode=2 sensorless\r\n");
    return;
  }

  if (starts_with(cursor, "CLEAR"))
  {
    cursor += 5;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: PUSH format invalid\r\n");
      return;
    }
    (void)zdt_clear_pos(PINGTUI_ADDR);
    pingtui_set_current_pos(0.0f);
    txs("# PUSH CLEAR addr=6 driver_pos=0 software_pos=0.000\r\n");
    return;
  }

  if (starts_with(cursor, "UNSTALL"))
  {
    cursor += 7;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: PUSH format invalid\r\n");
      return;
    }
    (void)zdt_clear_stall(PINGTUI_ADDR);
    txs("# PUSH UNSTALL addr=6\r\n");
    return;
  }

  if (starts_with(cursor, "ZERO"))
  {
    cursor += 4;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: PUSH format invalid\r\n");
      return;
    }
    pingtui_set_current_pos(0.0f);
    txs("# PUSH ZERO pos=0.000\r\n");
    return;
  }

  if (starts_with(cursor, "SET"))
  {
    cursor += 3;
    if (!parse_i32(&cursor, &pos))
    {
      txs("# ERROR: PUSH SET format invalid\r\n");
      return;
    }
    trim_left(&cursor);
    if (*cursor != '\0' || pos < PINGTUI_MIN_POS || pos > PINGTUI_MAX_POS)
    {
      txs("# ERROR: PUSH SET params rejected\r\n");
      return;
    }
    pingtui_set_current_pos((float)pos);
    txs("# PUSH SET pos=");
    tx_float3(pingtui_current_pos());
    txs("\r\n");
    return;
  }

  if (!parse_i32(&cursor, &pos))
  {
    txs("# ERROR: PUSH format invalid\r\n");
    return;
  }
  (void)parse_i32(&cursor, &speed);
  (void)parse_i32(&cursor, &acc);
  trim_left(&cursor);
  if (*cursor != '\0' ||
      pos < PINGTUI_MIN_POS || pos > PINGTUI_MAX_POS ||
      speed < 1 || speed > PINGTUI_MAX_SPEED ||
      acc < 0 || acc > 255)
  {
    txs("# ERROR: PUSH params rejected\r\n");
    return;
  }

  wait_ms = pingtui_control((float)pos, (uint16_t)speed, (uint8_t)acc);
  txs("# PUSH target=");
  txi(pos);
  txs(" speed=");
  txi(speed);
  txs(" acc=");
  txi(acc);
  txs(" pos=");
  tx_float3(pingtui_current_pos());
  txs(" wait_ms=");
  txu((uint32_t)wait_ms);
  txs("\r\n");
}

static bool parse_act_color(char **cursor, uint8_t *code)
{
  trim_left(cursor);
  if (**cursor == 'R' || **cursor == 'r' || **cursor == '1')
  {
    *code = 1U;
  }
  else if (**cursor == 'G' || **cursor == 'g' || **cursor == '2')
  {
    *code = 2U;
  }
  else if (**cursor == 'B' || **cursor == 'b' || **cursor == '3')
  {
    *code = 3U;
  }
  else
  {
    return false;
  }
  (*cursor)++;
  trim_left(cursor);
  return **cursor == '\0';
}

static float act_yaw_for_color(uint8_t code)
{
  if (code == 1U)
  {
    return ACT_YT_RIGHT_RING_DEG;
  }
  if (code == 2U)
  {
    return ACT_YT_CENTER_RING_DEG;
  }
  return ACT_YT_LEFT_RING_DEG;
}

static void act_pose(float ping, int32_t lift, float yaw)
{
  (void)move_all(ping, ACT_PING_SPEED, ACT_PING_ACC,
                 (float)lift, ACT_LIFT_SPEED, ACT_LIFT_ACC,
                 yaw, ACT_YT_SPEED);
}

static void act_stop_all(void)
{
  stop_all_motion();
  zdt_home_stop_repeat(PINGTUI_ADDR);
  zdt_home_stop_repeat(SHENGJIANG_ADDR);
  race_heat_set(0U);
}

static void act_safe_pose(void)
{
  race_heat_set(0U);
  stop_all_motion();
  set_zhuashou_kai();
  set_wukuaipingtai_weizhi(0x01);
  act_pose(ACT_X_SAFE, ACT_H_SAFE, ACT_YT_SAFE_DEG);
}

static void act_rack_ready(void)
{
  act_pose(ACT_X_RACK, ACT_H_RACK, ACT_YT_PLACE_RACK_DEG);
}

static void act_pick_from_platform(uint8_t platform)
{
  set_wukuaipingtai_weizhi(platform);
  act_rack_ready();
  set_zhuashou_he();
}

static void act_platform_to_yaw(uint8_t platform, float yaw)
{
  set_zhuashou_kai();
  set_wukuaipingtai_weizhi(platform);
  act_pose(ACT_X_RACK, ACT_H_RACK, ACT_YT_PLACE_RACK_DEG);
  set_zhuashou_he();
  act_pose(ACT_X_RACK, ACT_H_TRANSFER, ACT_YT_PLACE_RACK_DEG);
  act_pose(ACT_X_PLACE, ACT_H_TRANSFER, yaw);
  act_pose(ACT_X_PLACE, ACT_H_PLACE, yaw);
  set_zhuashou_kai();
  act_pose(ACT_X_PLACE, ACT_H_TRANSFER, yaw);
  act_pose(ACT_X_PLACE, ACT_H_TRANSFER, ACT_YT_PLACE_RACK_DEG);
}

static void act_finish_push_lift_zero(void)
{
  act_pose(ACT_X_SAFE, ACT_H_TRANSFER, ACT_YT_PLACE_RACK_DEG);
  act_pingtui_prezero("ACT ENDZERO");
  shengjiang_homezero_default("ACT ENDZERO");
}

static void act_round3(void)
{
  act_platform_to_yaw(1U, ACT_YT_RIGHT_RING_DEG);
  act_platform_to_yaw(2U, ACT_YT_CENTER_RING_DEG);
  act_platform_to_yaw(3U, ACT_YT_LEFT_RING_DEG);
  act_finish_push_lift_zero();
}

static void act_safe_transfer(void)
{
  act_pose(ACT_X_SAFE, ACT_H_TRANSFER, ACT_YT_PLACE_RACK_DEG);
}

static void act_pick_current_platform(void)
{
  set_zhuashou_kai();
  act_rack_ready();
  set_zhuashou_he();
  act_pose(ACT_X_RACK, ACT_H_TRANSFER, ACT_YT_PLACE_RACK_DEG);
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
  act_pose(ACT_X_RACK, ACT_H_TRANSFER, ACT_YT_PLACE_RACK_DEG);
  act_rack_ready();
  set_zhuashou_kai();
  act_safe_transfer();
}

static void act_current_platform_to_yaw(uint8_t code)
{
  act_pick_current_platform();
  act_release_to_yaw(act_yaw_for_color(code));
}

static void act_pre_yaw(uint8_t code)
{
  set_zhuashou_kai();
  act_pose(ACT_X_SAFE, ACT_H_TRANSFER, act_yaw_for_color(code));
}

static void act_fang(uint8_t code)
{
  act_release_to_yaw(act_yaw_for_color(code));
}

static void act_fang_maduo(uint8_t code)
{
  act_release_to_yaw(act_yaw_for_color(code));
}

static void act_na(uint8_t code)
{
  act_pick_from_yaw(act_yaw_for_color(code));
}

static void act_yuantai_na(uint8_t code)
{
  act_current_platform_to_yaw(code);
}

static void act_yuantai_na1(uint8_t code)
{
  act_pre_yaw(code);
}

static void act_yuantai_na2(uint8_t code)
{
  act_current_platform_to_yaw(code);
}

static void handle_act(char *params)
{
  char *cursor = params;
  uint8_t code;
  int32_t value;
  int32_t ping_scaled;
  int32_t lift;
  int32_t yaw_scaled;

  trim_left(&cursor);
  if (*cursor == '\0' || *cursor == '?')
  {
    txs("# ACT commands: STOP ZERO SAFE RESET JIXIE POSE RACK PICK ROUND3 PLATFORM HEAT OPEN CLOSE NA YTNA NA1 NA2 FANG FANGMD\r\n");
    txs("# ACT colors: R/G/B or 1/2/3; platform: 1/2/3; heat: ON/OFF/<0..1999>\r\n");
    return;
  }

  if (starts_with(cursor, "STOP"))
  {
    cursor += 4;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: ACT format invalid\r\n");
      return;
    }
    act_stop_all();
    txs("# ACT STOP\r\n");
    return;
  }

  if (starts_with(cursor, "ZERO"))
  {
    cursor += 4;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: ACT format invalid\r\n");
      return;
    }
    act_pingtui_prezero("ACT ZERO");
    shengjiang_homezero_default("ACT ZERO");
    txs("# ACT ZERO ping=0.000 sj=0.000\r\n");
    return;
  }

  if (starts_with(cursor, "SAFE") || starts_with(cursor, "RESET") || starts_with(cursor, "JIXIE"))
  {
    if (starts_with(cursor, "SAFE"))
    {
      cursor += 4;
    }
    else if (starts_with(cursor, "RESET"))
    {
      cursor += 5;
    }
    else
    {
      cursor += 5;
    }
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: ACT format invalid\r\n");
      return;
    }
    act_pingtui_prezero("ACT PREZERO");
    shengjiang_homezero_default("ACT PREZERO");
    act_safe_pose();
    txs("# ACT SAFE done\r\n");
    return;
  }

  if (starts_with(cursor, "OPEN"))
  {
    cursor += 4;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: ACT format invalid\r\n");
      return;
    }
    set_zhuashou_kai();
    txs("# ACT OPEN\r\n");
    return;
  }

  if (starts_with(cursor, "CLOSE"))
  {
    cursor += 5;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: ACT format invalid\r\n");
      return;
    }
    set_zhuashou_he();
    txs("# ACT CLOSE\r\n");
    return;
  }

  if (starts_with(cursor, "RACK"))
  {
    cursor += 4;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: ACT format invalid\r\n");
      return;
    }
    act_pingtui_prezero("ACT PREZERO");
    shengjiang_homezero_default("ACT PREZERO");
    act_rack_ready();
    txs("# ACT RACK\r\n");
    return;
  }

  if (starts_with(cursor, "ROUND3"))
  {
    cursor += 6;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: ACT ROUND3 format invalid\r\n");
      return;
    }
    act_pingtui_prezero("ACT PREZERO");
    shengjiang_homezero_default("ACT PREZERO");
    act_round3();
    txs("# ACT ROUND3 done\r\n");
    return;
  }

  if (starts_with(cursor, "PICK"))
  {
    cursor += 4;
    if (!parse_i32(&cursor, &value))
    {
      value = 1;
    }
    trim_left(&cursor);
    if (*cursor != '\0' || value < 1 || value > 3)
    {
      txs("# ERROR: ACT PICK params rejected\r\n");
      return;
    }
    act_pingtui_prezero("ACT PREZERO");
    shengjiang_homezero_default("ACT PREZERO");
    act_pick_from_platform((uint8_t)value);
    txs("# ACT PICK platform=");
    txi(value);
    txs("\r\n");
    return;
  }

  if (starts_with(cursor, "PLATFORM") || starts_with(cursor, "PT"))
  {
    if (starts_with(cursor, "PLATFORM"))
    {
      cursor += 8;
    }
    else
    {
      cursor += 2;
    }
    if (!parse_i32(&cursor, &value))
    {
      txs("# ERROR: ACT PLATFORM format invalid\r\n");
      return;
    }
    trim_left(&cursor);
    if (*cursor != '\0' || value < 1 || value > 3)
    {
      txs("# ERROR: ACT PLATFORM params rejected\r\n");
      return;
    }
    set_wukuaipingtai_weizhi((int)value);
    txs("# ACT PLATFORM ");
    txi(value);
    txs("\r\n");
    return;
  }

  if (starts_with(cursor, "HEAT"))
  {
    cursor += 4;
    trim_left(&cursor);
    if (starts_with(cursor, "ON"))
    {
      cursor += 2;
      value = ACT_HEAT_COMPARE;
    }
    else if (starts_with(cursor, "OFF"))
    {
      cursor += 3;
      value = 0;
    }
    else if (!parse_i32(&cursor, &value))
    {
      txs("# ERROR: ACT HEAT format invalid\r\n");
      return;
    }
    trim_left(&cursor);
    if (*cursor != '\0' || value < 0 || value > 1999)
    {
      txs("# ERROR: ACT HEAT params rejected\r\n");
      return;
    }
    race_heat_set((uint16_t)value);
    txs("# ACT HEAT compare=");
    txi(value);
    txs("\r\n");
    return;
  }

  if (starts_with(cursor, "POSE"))
  {
    cursor += 4;
    if (!parse_scaled(&cursor, &ping_scaled) ||
        !parse_i32(&cursor, &lift) ||
        !parse_scaled(&cursor, &yaw_scaled))
    {
      txs("# ERROR: ACT POSE format invalid\r\n");
      return;
    }
    trim_left(&cursor);
    if (*cursor != '\0' || lift < SHENGJIANG_MIN_POS || lift > SHENGJIANG_MAX_POS)
    {
      txs("# ERROR: ACT POSE params rejected\r\n");
      return;
    }
    act_pose((float)ping_scaled / (float)PID_SCALE, lift,
             (float)yaw_scaled / (float)PID_SCALE);
    txs("# ACT POSE done\r\n");
    return;
  }

  if (starts_with(cursor, "YTNA"))
  {
    cursor += 4;
    if (!parse_act_color(&cursor, &code))
    {
      txs("# ERROR: ACT YTNA format invalid\r\n");
      return;
    }
    act_pingtui_prezero("ACT PREZERO");
    shengjiang_homezero_default("ACT PREZERO");
    act_yuantai_na(code);
    txs("# ACT YTNA done\r\n");
    return;
  }

  if (starts_with(cursor, "FANGMD"))
  {
    cursor += 6;
    if (!parse_act_color(&cursor, &code))
    {
      txs("# ERROR: ACT FANGMD format invalid\r\n");
      return;
    }
    act_pingtui_prezero("ACT PREZERO");
    shengjiang_homezero_default("ACT PREZERO");
    act_fang_maduo(code);
    txs("# ACT FANGMD done\r\n");
    return;
  }

  if (starts_with(cursor, "FANG"))
  {
    cursor += 4;
    if (!parse_act_color(&cursor, &code))
    {
      txs("# ERROR: ACT FANG format invalid\r\n");
      return;
    }
    act_pingtui_prezero("ACT PREZERO");
    shengjiang_homezero_default("ACT PREZERO");
    act_fang(code);
    txs("# ACT FANG done\r\n");
    return;
  }

  if (starts_with(cursor, "NA2"))
  {
    cursor += 3;
    if (!parse_act_color(&cursor, &code))
    {
      txs("# ERROR: ACT NA2 format invalid\r\n");
      return;
    }
    act_pingtui_prezero("ACT PREZERO");
    shengjiang_homezero_default("ACT PREZERO");
    act_yuantai_na2(code);
    txs("# ACT NA2 done\r\n");
    return;
  }

  if (starts_with(cursor, "NA1"))
  {
    cursor += 3;
    if (!parse_act_color(&cursor, &code))
    {
      txs("# ERROR: ACT NA1 format invalid\r\n");
      return;
    }
    act_pingtui_prezero("ACT PREZERO");
    shengjiang_homezero_default("ACT PREZERO");
    act_yuantai_na1(code);
    txs("# ACT NA1 done\r\n");
    return;
  }

  if (starts_with(cursor, "NA"))
  {
    cursor += 2;
    if (!parse_act_color(&cursor, &code))
    {
      txs("# ERROR: ACT NA format invalid\r\n");
      return;
    }
    act_pingtui_prezero("ACT PREZERO");
    shengjiang_homezero_default("ACT PREZERO");
    act_na(code);
    txs("# ACT NA done yaw=");
    tx_float3(act_yaw_for_color(code));
    txs("\r\n");
    return;
  }

  txs("# ERROR: ACT format invalid\r\n");
}

static void handle_llm(char *params)
{
  char *cursor = params;
  int32_t speed = s_llm_drive_speed;
  float target_delta = 0.0f;

  trim_left(&cursor);
  if (starts_with(cursor, "ON"))
  {
    cursor += 2;
    trim_left(&cursor);
    if (*cursor != '\0' && !parse_i32(&cursor, &speed))
    {
      txs("# ERROR: LLM format invalid\r\n");
      return;
    }
    trim_left(&cursor);
    if (*cursor != '\0' && !parse_float(&cursor, &target_delta))
    {
      txs("# ERROR: LLM format invalid\r\n");
      return;
    }
    trim_left(&cursor);
    if (*cursor != '\0' || speed < LLM_MIN_SPEED || speed > LLM_MAX_SPEED)
    {
      txs("# ERROR: LLM params rejected\r\n");
      return;
    }

    imu_scan(fAcc, fGyro, fAngle);
    s_llm_feedback_yaw = fAngle[2];
    if (!imu_data_ready())
    {
      txs("# ERROR: IMU data incomplete; run IMU/IMURAW/IMUREAD before LLM\r\n");
      emit_llm_csv();
      return;
    }
    s_llm_drive_speed = speed;
    s_llm_target_yaw = normalize_yaw(s_llm_feedback_yaw + target_delta);
    llm_reset_state();
    s_move_active = 0U;
    s_llm_active = 1U;
    s_llm_next_sample_ms = HAL_GetTick();
    move_vel(0, s_llm_drive_speed, 0);
    txs("# LLM ATT ON\r\n");
    emit_llm_csv();
  }
  else if (starts_with(cursor, "OFF"))
  {
    cursor += 3;
    trim_left(&cursor);
    if (*cursor != '\0')
    {
      txs("# ERROR: LLM format invalid\r\n");
      return;
    }
    stop_all_motion();
    txs("# LLM OFF\r\n");
  }
  else
  {
    txs("# ERROR: LLM format invalid\r\n");
  }
}

static void handle_line(char *line)
{
  char *cmd = line;

  trim_left(&cmd);
  trim_right(cmd);
  if (*cmd == '\0')
  {
    return;
  }

  if (strcmp(cmd, "STATUS") == 0)
  {
    emit_status();
  }
  else if (strcmp(cmd, "RESET") == 0)
  {
    stop_all_motion();
    apply_default_llm_pid();
    apply_default_vel_acc();
    txs("# RESET OK\r\n");
    emit_llm_csv();
  }
  else if (strcmp(cmd, "ENC") == 0)
  {
    handle_enc(cmd + 3);
  }
  else if (starts_with(cmd, "IMUCFG"))
  {
    handle_imu_cfg(cmd + 6);
  }
  else if (starts_with(cmd, "IMUREAD"))
  {
    handle_imu_read(cmd + 7);
  }
  else if (starts_with(cmd, "IMURAW"))
  {
    handle_imu_raw(cmd + 6);
  }
  else if (starts_with(cmd, "IMU"))
  {
    handle_imu(cmd + 3);
  }
  else if (starts_with(cmd, "SET "))
  {
    handle_set(cmd + 4);
  }
  else if (starts_with(cmd, "PID "))
  {
    handle_pid(cmd + 4);
  }
  else if (starts_with(cmd, "ACC "))
  {
    handle_acc(cmd + 4);
  }
  else if (strcmp(cmd, "SERVO") == 0)
  {
    handle_servo(cmd + 5);
  }
  else if (starts_with(cmd, "SERVO "))
  {
    handle_servo(cmd + 6);
  }
  else if (starts_with(cmd, "YT "))
  {
    handle_angle_servo(cmd + 3, 'Y');
  }
  else if (starts_with(cmd, "PT "))
  {
    handle_angle_servo(cmd + 3, 'P');
  }
  else if (starts_with(cmd, "ZH "))
  {
    handle_angle_servo(cmd + 3, 'Z');
  }
  else if (strcmp(cmd, "SJ") == 0)
  {
    handle_shengjiang(cmd + 2);
  }
  else if (starts_with(cmd, "SJ "))
  {
    handle_shengjiang(cmd + 3);
  }
  else if (strcmp(cmd, "PUSH") == 0)
  {
    handle_pingtui(cmd + 4);
  }
  else if (starts_with(cmd, "PUSH "))
  {
    handle_pingtui(cmd + 5);
  }
  else if (starts_with(cmd, "ACT"))
  {
    handle_act(cmd + 3);
  }
  else if (starts_with(cmd, "LLM "))
  {
    handle_llm(cmd + 4);
  }
  else if (starts_with(cmd, "WHEEL "))
  {
    stop_all_motion();
    handle_wheel(cmd + 6);
  }
  else if (starts_with(cmd, "MOVE "))
  {
    handle_move(cmd + 5);
  }
  else if (starts_with(cmd, "PULSE1 "))
  {
    handle_pulse_one(cmd + 7);
  }
  else if (starts_with(cmd, "PULSE "))
  {
    handle_pulse(cmd + 6);
  }
  else
  {
    txs("# ERROR: unknown command\r\n");
  }
}

static void arm_rx(void)
{
  (void)HAL_UART_Receive_IT(&TUNER_UART, &s_rx_byte, 1U);
}

void motor_ack_test_init(void)
{
  CAN_FilterTypeDef filter = {0};

  s_line_ready = 0U;
  s_line_len = 0U;
  s_line2_ready = 0U;
  s_line2_len = 0U;
  s_rx_pos = 0U;
  s_rx_overflow = 0U;
  s_overflow_ready = 0U;
  s_can_rx_head = 0U;
  s_can_rx_tail = 0U;
  s_can_rx_overflow = 0U;
  s_move_stop_ms = 0U;
  s_move_active = 0U;
  apply_default_vel_acc();
  s_move_done_msg = "# MOVE done\r\n";
  s_llm_active = 0U;
  s_llm_drive_speed = LLM_DEFAULT_SPEED;
  s_llm_target_yaw = 0.0f;
  s_llm_feedback_yaw = 0.0f;
  s_llm_error = 0.0f;
  s_llm_output = 0.0f;
  s_llm_next_sample_ms = 0U;
  s_servo_pwm_started = 0U;
  s_servo_ch2 = SERVO_CH2_DEFAULT;
  s_servo_ch3 = SERVO_CH3_DEFAULT;
  s_servo_ch4 = SERVO_CH4_DEFAULT;
  shengjiang_set_current_pos(0.0f);
  imu_init();
  PIDInit();
  apply_default_llm_pid();
  llm_reset_state();

  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0U;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = 0U;
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;
  (void)HAL_CAN_ConfigFilter(&hcan1, &filter);
  (void)HAL_CAN_Start(&hcan1);
  (void)HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  zdt_set_wait_enabled(true);
  pingtui_set_current_pos(0.0f);
  (void)zdt_clear_pos(PINGTUI_ADDR);
  shengjiang_set_current_pos(0.0f);
  (void)zdt_clear_pos(SHENGJIANG_ADDR);
  arm_rx();
  txs("# BOOT servo_pwm=off explicit_command_required\r\n");
  emit_status();
}

void motor_ack_test_uart_rx_callback(UART_HandleTypeDef *huart)
{
  char c;

  if (huart != &TUNER_UART)
  {
    return;
  }

  c = (char)s_rx_byte;
  if (c == '\r')
  {
    arm_rx();
    return;
  }

  if (c == '\n')
  {
    if (s_rx_overflow != 0U)
    {
      s_rx_overflow = 0U;
      s_overflow_ready = 1U;
      s_rx_pos = 0U;
    }
    else
    {
      if (s_line_ready == 0U)
      {
        uint8_t i;

        s_line_len = s_rx_pos;
        for (i = 0U; i <= s_rx_pos && i < CMD_MAX_LEN; i++)
        {
          s_line[i] = s_rx_line[i];
        }
        s_line[s_rx_pos] = '\0';
        s_line_ready = 1U;
      }
      else if (s_line2_ready == 0U)
      {
        uint8_t i;

        s_line2_len = s_rx_pos;
        for (i = 0U; i <= s_rx_pos && i < CMD_MAX_LEN; i++)
        {
          s_line2[i] = s_rx_line[i];
        }
        s_line2[s_rx_pos] = '\0';
        s_line2_ready = 1U;
      }
      else
      {
        s_overflow_ready = 1U;
      }
      s_rx_pos = 0U;
    }
    arm_rx();
    return;
  }

  if (s_rx_overflow == 0U)
  {
    if (s_rx_pos < (CMD_MAX_LEN - 1U))
    {
      s_rx_line[s_rx_pos++] = c;
    }
    else
    {
      s_rx_overflow = 1U;
    }
  }

  arm_rx();
}

void motor_ack_test_poll(void)
{
  char line[CMD_MAX_LEN];
  uint8_t len = 0U;
  uint8_t ready;
  uint8_t overflow;
  uint8_t i;
  uint32_t now = HAL_GetTick();

  __disable_irq();
  ready = s_line_ready;
  overflow = s_overflow_ready;
  if (overflow != 0U)
  {
    s_overflow_ready = 0U;
  }
  if (ready != 0U)
  {
    len = s_line_len;
    for (i = 0U; i < len && i < CMD_MAX_LEN - 1U; i++)
    {
      line[i] = s_line[i];
    }
    line[i] = '\0';
    s_line_ready = 0U;
    s_line_len = 0U;
    if (s_line2_ready != 0U)
    {
      uint8_t j;

      s_line_len = s_line2_len;
      for (j = 0U; j < s_line2_len && j < CMD_MAX_LEN - 1U; j++)
      {
        s_line[j] = s_line2[j];
      }
      s_line[j] = '\0';
      s_line_ready = 1U;
      s_line2_ready = 0U;
      s_line2_len = 0U;
    }
  }
  __enable_irq();

  if (overflow != 0U)
  {
    txs("# ERROR: command too long\r\n");
  }
  if (s_move_active != 0U && (int32_t)(now - s_move_stop_ms) >= 0)
  {
    move_vel(0, 0, 0);
    s_move_active = 0U;
    txs(s_move_done_msg);
  }
  if (ready != 0U)
  {
    handle_line(line);
  }
  if (s_llm_active != 0U)
  {
    llm_poll(now);
  }
  else
  {
    emit_can_events();
  }
}

void motor_ack_test_can_rx_callback(CAN_HandleTypeDef *hcan)
{
  motor_can_frame_t frame;
  uint8_t next;

  if (hcan != &hcan1)
  {
    return;
  }

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &frame.header, frame.data) != HAL_OK)
    {
      return;
    }
    next = (uint8_t)((s_can_rx_head + 1U) % RX_RING_SIZE);
    if (next == s_can_rx_tail)
    {
      s_can_rx_overflow++;
      continue;
    }
    s_can_rx_ring[s_can_rx_head] = frame;
    s_can_rx_head = next;
  }
}
