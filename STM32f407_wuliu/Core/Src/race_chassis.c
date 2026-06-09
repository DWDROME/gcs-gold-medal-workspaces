#include "race_chassis.h"
#include "hal_zdt.h"
#include "host_proto.h"
#include "imu.h"
#include "race_io.h"
#include "tim.h"
#include <math.h>
#include <stdlib.h>

#define ABSF(x) ((x) >= 0.0f ? (x) : -(x))
#define SERVO_PWM_MIN 500U
#define SERVO_PWM_MAX 2500U
#define YUANTAI_ANGLE_OFFSET 7.9f
#define YUANTAI_DEFAULT_DEG 0.0f
#define YUANTAI_PWM_STEP 10U
#define YUANTAI_STEP_DELAY_MS 20U
#define WUKUAI_PWM_STEP 20U
#define WUKUAI_STEP_DELAY_MS 10U
#define WUKUAI_POS1_DEG 0.0f
#define WUKUAI_POS2_DEG 135.0f
#define WUKUAI_POS3_DEG 270.0f
#define ZHUASHOU_CLOSE_DEG 25.0f
#define ZHUASHOU_OPEN_DEG 50.0f
#define ZHUASHOU_WAIT_MS 600.0f
#define SHENGJIANG_ADDR 5U
#define PINGTUI_ADDR 6U
#define SHENGJIANG_MAX_POS 60
#define PINGTUI_MIN_POS 0.0f
#define PINGTUI_MAX_POS 65.0f
#define ZDT_STOP_REPEAT 3U
#define ZDT_STOP_DELAY_MS 5U
#define ZDT_HOME_VEL 120U
#define ZDT_HOME_TIMEOUT_MS 10000U
#define ZDT_HOME_SHENGJIANG_TIMEOUT_MS 3000U
#define ZDT_HOME_SENSORLESS_VEL 300U
#define ZDT_HOME_SHENGJIANG_CURRENT_MA 750U
#define ZDT_HOME_SHENGJIANG_TIME_MS 30U
#define ZDT_HOME_POLL_DELAY_MS 25U
#define ZDT_HOME_SW_HIT_ABS_VEL_RPM 5
#define ZDT_HOME_SW_HIT_PHASE_MA 700U
#define ZDT_HOME_SW_HIT_TIME_MS 80U
#define ZDT_HOME_STATUS_REPLY 0x3BU
#define ZDT_HOME_STATUS_ENCODER_READY 0x01U
#define ZDT_HOME_STATUS_TABLE_READY 0x02U
#define ZDT_HOME_STATUS_GOING 0x04U
#define ZDT_HOME_STATUS_FAIL 0x08U
#define RX_RING_SIZE 16U
#define SHENGJIANG_HOMEZERO_LOG 1U

typedef struct
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
} motor_can_frame_t;

PID mypid = {0};
float feedbackValue = 0.0f;
float feedbackValue_pre = 0.0f;
float targetValue = 0.0f;
float fAcc[3] = {0};
float fGyro[3] = {0};
float fAngle[3] = {0};
int vx = 0;
int vy = 0;
int omiga = 0;
int flag = 0;
uint8_t dianji_move_flag = 0U;
uint8_t PID_control_flag = 0U;
uint8_t circle = 0U;
float same_time = 0.0f;

static int car_move_speed = 50;
static int a1 = 70;
static int a = 230;
static const float yuantai_reset_ms = 300.0f;
static const float yuantai_move_ms = 300.0f;
static const float wukuai_wait_ms = 150.0f;
static float current_pos = 0.0f;
static float current_pos1 = 0.0f;
static float yuantai_angle = YUANTAI_DEFAULT_DEG;
static float wukuai_angle = WUKUAI_POS1_DEG;
static uint32_t delay_time = 0U;
static uint32_t real_time = 0U;
static volatile uint8_t s_tim4_tick = 0U;
static volatile uint8_t s_can_rx_head = 0U;
static volatile uint8_t s_can_rx_tail = 0U;
static volatile uint32_t s_can_rx_overflow = 0U;
static motor_can_frame_t s_can_rx_ring[RX_RING_SIZE];
static uint8_t s_servo_pwm_started = 0U;
static int32_t s_home_vel_rpm = 0;
static uint16_t s_home_phase_ma = 0U;
static bool s_home_vel_valid = false;
static bool s_home_phase_valid = false;

#if SHENGJIANG_HOMEZERO_LOG
static void dbg_txs(const char *s)
{
  uint16_t n = 0U;

  while (s[n] != 0)
  {
    n++;
  }

  (void)HAL_UART_Transmit(&DBG_UART, (uint8_t *)s, n, 100U);
}

static void dbg_txu(uint32_t v)
{
  char b[12];
  int i = 10;

  b[11] = 0;
  if (v == 0U)
  {
    b[10] = '0';
    dbg_txs(&b[10]);
    return;
  }

  while (v != 0U && i >= 0)
  {
    b[i] = (char)('0' + (v % 10U));
    v /= 10U;
    i--;
  }

  dbg_txs(&b[i + 1]);
}

static void dbg_txi(int32_t v)
{
  if (v < 0)
  {
    dbg_txs("-");
    dbg_txu((uint32_t)(-v));
    return;
  }

  dbg_txu((uint32_t)v);
}

static void dbg_txh4(uint8_t v)
{
  const char hex[] = "0123456789ABCDEF";
  char b[3];

  b[0] = hex[(v >> 4) & 0x0FU];
  b[1] = hex[v & 0x0FU];
  b[2] = 0;
  dbg_txs(b);
}

static void dbg_txh8(uint32_t v)
{
  uint8_t i;

  for (i = 0U; i < 4U; i++)
  {
    dbg_txh4((uint8_t)(v >> (24U - 8U * i)));
  }
}

static void dbg_can_frame(uint32_t elapsed_ms, const motor_can_frame_t *frame, uint32_t id)
{
  uint8_t i;
  uint16_t phase_ma;
  int32_t vel_rpm;

  dbg_txs("# SJ CANRX t=");
  dbg_txu(elapsed_ms);
  dbg_txs(" id=0x");
  dbg_txh8(id);
  dbg_txs(" dlc=");
  dbg_txu(frame->header.DLC);
  dbg_txs(" data=");
  for (i = 0U; i < frame->header.DLC && i < 8U; i++)
  {
    if (i != 0U)
    {
      dbg_txs(" ");
    }
    dbg_txh4(frame->data[i]);
  }
  dbg_txs("\r\n");

  if (frame->header.DLC >= 4U && frame->data[0] == 0x27U)
  {
    phase_ma = ((uint16_t)frame->data[1] << 8) | (uint16_t)frame->data[2];
    dbg_txs("# SJ CPHA t=");
    dbg_txu(elapsed_ms);
    dbg_txs(" phase_ma=");
    dbg_txu(phase_ma);
    dbg_txs("\r\n");
  }
  else if (frame->header.DLC >= 5U && frame->data[0] == 0x35U)
  {
    vel_rpm = ((int32_t)frame->data[2] << 8) | (int32_t)frame->data[3];
    if (frame->data[1] != 0U)
    {
      vel_rpm = -vel_rpm;
    }
    dbg_txs("# SJ VEL t=");
    dbg_txu(elapsed_ms);
    dbg_txs(" rpm=");
    dbg_txi(vel_rpm);
    dbg_txs("\r\n");
  }
}

static void dbg_home_status(uint32_t elapsed_ms, uint8_t status)
{
  dbg_txs("# SJ HSTAT t=");
  dbg_txu(elapsed_ms);
  dbg_txs(" status=0x");
  dbg_txh4(status);
  dbg_txs(" enc=");
  dbg_txu((status & ZDT_HOME_STATUS_ENCODER_READY) ? 1U : 0U);
  dbg_txs(" table=");
  dbg_txu((status & ZDT_HOME_STATUS_TABLE_READY) ? 1U : 0U);
  dbg_txs(" going=");
  dbg_txu((status & ZDT_HOME_STATUS_GOING) ? 1U : 0U);
  dbg_txs(" fail=");
  dbg_txu((status & ZDT_HOME_STATUS_FAIL) ? 1U : 0U);
  dbg_txs("\r\n");
}
#else
#define dbg_txs(s) ((void)0)
#define dbg_txu(v) ((void)0)
#define dbg_txi(v) ((void)0)
#define dbg_txh4(v) ((void)0)
#define dbg_txh8(v) ((void)0)
#define dbg_can_frame(elapsed_ms, frame, id) ((void)0)
#define dbg_home_status(elapsed_ms, status) ((void)0)
#endif

static uint16_t servo_pulse_270(float angle)
{
  return (uint16_t)((angle / 270.0f) * 2000.0f + 500.0f);
}

static uint16_t servo_clamp_pwm_i32(int32_t value)
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

static uint16_t yuantai_pwm_from_angle(float target)
{
  int32_t pwm = (int32_t)(((target - YUANTAI_ANGLE_OFFSET) / 360.0f) * 2000.0f + 500.0f);
  return servo_clamp_pwm_i32(pwm);
}

static uint16_t wukuai_pwm_from_angle(float target)
{
  int32_t pwm = (int32_t)((target / 270.0f) * 2000.0f + 500.0f);
  return servo_clamp_pwm_i32(pwm);
}

static void servo_pwm_start_if_needed(void)
{
  if (s_servo_pwm_started != 0U)
  {
    return;
  }

  (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  s_servo_pwm_started = 1U;
}

static void set_servo_pwm_slow(uint32_t channel, uint16_t current_pwm, uint16_t target_pwm, uint16_t step, uint32_t delay_ms)
{
  servo_pwm_start_if_needed();

  while (current_pwm != target_pwm)
  {
    if (current_pwm < target_pwm)
    {
      uint16_t next_pwm = current_pwm + step;
      current_pwm = (next_pwm > target_pwm) ? target_pwm : next_pwm;
    }
    else
    {
      uint16_t next_pwm = current_pwm - step;
      current_pwm = (next_pwm < target_pwm) ? target_pwm : next_pwm;
    }
    __HAL_TIM_SET_COMPARE(&htim2, channel, current_pwm);
    delay_ms1(delay_ms);
  }
}

static void set_yuantai_pwm_slow(uint16_t target_pwm)
{
  set_servo_pwm_slow(TIM_CHANNEL_4, yuantai_pwm_from_angle(yuantai_angle), target_pwm, YUANTAI_PWM_STEP, YUANTAI_STEP_DELAY_MS);
}

static void set_wukuai_pwm_slow(uint16_t target_pwm)
{
  set_servo_pwm_slow(TIM_CHANNEL_3, wukuai_pwm_from_angle(wukuai_angle), target_pwm, WUKUAI_PWM_STEP, WUKUAI_STEP_DELAY_MS);
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
  (void)HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void race_chassis_can_rx_callback(CAN_HandleTypeDef *hcan)
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

static void clear_can_rx_ring(void)
{
  __disable_irq();
  s_can_rx_tail = s_can_rx_head;
  __enable_irq();
}

static int32_t abs_i32(int32_t v)
{
  return (v < 0) ? -v : v;
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

static bool pop_home_status(uint8_t addr, uint32_t elapsed_ms, uint8_t *status)
{
  motor_can_frame_t frame;
  uint32_t id;

  while (pop_can_frame(&frame))
  {
    id = (frame.header.IDE == CAN_ID_EXT) ? frame.header.ExtId : frame.header.StdId;
    if (frame.header.IDE == CAN_ID_EXT &&
        (uint8_t)(id >> 8) == addr)
    {
      dbg_can_frame(elapsed_ms, &frame, id);
      if (frame.header.DLC >= 4U && frame.data[0] == 0x27U)
      {
        s_home_phase_ma = ((uint16_t)frame.data[1] << 8) | (uint16_t)frame.data[2];
        s_home_phase_valid = true;
      }
      else if (frame.header.DLC >= 5U && frame.data[0] == 0x35U)
      {
        s_home_vel_rpm = ((int32_t)frame.data[2] << 8) | (int32_t)frame.data[3];
        if (frame.data[1] != 0U)
        {
          s_home_vel_rpm = -s_home_vel_rpm;
        }
        s_home_vel_valid = true;
      }
      if (frame.header.DLC >= 3U &&
          frame.data[0] == ZDT_HOME_STATUS_REPLY)
      {
        *status = frame.data[1];
        return true;
      }
    }
  }
  return false;
}

static bool wait_home_idle(uint8_t addr, uint32_t timeout_ms, uint8_t *last_status)
{
  uint32_t start = HAL_GetTick();
  uint32_t now;
  uint32_t elapsed;
  uint32_t last_no_status_log = 0U;
  uint32_t sw_hit_start = 0U;
  uint8_t status = ZDT_HOME_STATUS_GOING;

  *last_status = status;
  while ((uint32_t)(HAL_GetTick() - start) < timeout_ms)
  {
    (void)zdt_read(addr, ZDT_S_VEL);
    delay_ms1(2U);
    (void)zdt_read(addr, ZDT_S_CPHA);
    delay_ms1(2U);
    (void)zdt_read(addr, ZDT_S_ORG);
    delay_ms1(ZDT_HOME_POLL_DELAY_MS);
    now = HAL_GetTick();
    elapsed = (uint32_t)(now - start);
    while (pop_home_status(addr, elapsed, &status))
    {
      now = HAL_GetTick();
      elapsed = (uint32_t)(now - start);
      *last_status = status;
      dbg_home_status(elapsed, status);
      if ((status & ZDT_HOME_STATUS_GOING) == 0U)
      {
        return true;
      }
    }
    now = HAL_GetTick();
    elapsed = (uint32_t)(now - start);
    if (s_home_vel_valid && s_home_phase_valid &&
        abs_i32(s_home_vel_rpm) <= ZDT_HOME_SW_HIT_ABS_VEL_RPM &&
        s_home_phase_ma >= ZDT_HOME_SW_HIT_PHASE_MA)
    {
      if (sw_hit_start == 0U)
      {
        sw_hit_start = elapsed;
      }
      if ((elapsed - sw_hit_start) >= ZDT_HOME_SW_HIT_TIME_MS)
      {
        *last_status = ZDT_HOME_STATUS_ENCODER_READY | ZDT_HOME_STATUS_TABLE_READY;
        dbg_txs("# SJ HOMEZERO SW_HIT t=");
        dbg_txu(elapsed);
        dbg_txs(" vel=");
        dbg_txi(s_home_vel_rpm);
        dbg_txs(" phase_ma=");
        dbg_txu(s_home_phase_ma);
        dbg_txs("\r\n");
        return true;
      }
    }
    else
    {
      sw_hit_start = 0U;
    }
    if ((elapsed - last_no_status_log) >= 500U)
    {
      last_no_status_log = elapsed;
      dbg_txs("# SJ HSTAT wait t=");
      dbg_txu(elapsed);
      dbg_txs(" no_status\r\n");
    }
    delay_ms1(ZDT_HOME_POLL_DELAY_MS);
  }

  return false;
}

static zdt_ret_t zdt_home_cfg_sensorless(uint8_t addr, uint8_t dir, uint16_t current_ma, uint16_t time_ms)
{
  return zdt_home_params_write(addr, false, 2U, dir, ZDT_HOME_VEL, ZDT_HOME_SHENGJIANG_TIMEOUT_MS,
                               ZDT_HOME_SENSORLESS_VEL, current_ma, time_ms, false);
}

static bool shengjiang_homezero_run(uint8_t *status)
{
  bool idle;
  bool ok;
  zdt_ret_t ret;

  dbg_txs("# SJ HOMEZERO START mode=2 dir=1 home_vel=");
  dbg_txu(ZDT_HOME_VEL);
  dbg_txs(" hit_vel=");
  dbg_txu(ZDT_HOME_SENSORLESS_VEL);
  dbg_txs(" current=");
  dbg_txu(ZDT_HOME_SHENGJIANG_CURRENT_MA);
  dbg_txs(" time=");
  dbg_txu(ZDT_HOME_SHENGJIANG_TIME_MS);
  dbg_txs(" timeout=");
  dbg_txu(ZDT_HOME_SHENGJIANG_TIMEOUT_MS);
  dbg_txs("\r\n");

  clear_can_rx_ring();
  s_home_vel_rpm = 0;
  s_home_phase_ma = 0U;
  s_home_vel_valid = false;
  s_home_phase_valid = false;
  dbg_txs("# SJ HOMEZERO rx_flush=1 overflow=");
  dbg_txu(s_can_rx_overflow);
  dbg_txs("\r\n");
  (void)zdt_clear_stall(SHENGJIANG_ADDR);
  delay_ms1(20U);
  (void)zdt_en(SHENGJIANG_ADDR, true, false);
  delay_ms1(50U);
  ret = zdt_home_cfg_sensorless(SHENGJIANG_ADDR, 1U,
                                ZDT_HOME_SHENGJIANG_CURRENT_MA,
                                ZDT_HOME_SHENGJIANG_TIME_MS);
  dbg_txs("# SJ HOMEZERO cfg_ret=");
  dbg_txi((int32_t)ret);
  dbg_txs("\r\n");
  delay_ms1(20U);
  ret = zdt_home_trigger(SHENGJIANG_ADDR, 2U, false);
  dbg_txs("# SJ HOMEZERO trigger_ret=");
  dbg_txi((int32_t)ret);
  dbg_txs("\r\n");
  idle = wait_home_idle(SHENGJIANG_ADDR, ZDT_HOME_SHENGJIANG_TIMEOUT_MS, status);
  ok = idle && ((*status & ZDT_HOME_STATUS_FAIL) == 0U);
  dbg_txs("# SJ HOMEZERO ");
  dbg_txs(ok ? "DONE" : (idle ? "FAIL" : "TIMEOUT"));
  dbg_txs(" status=0x");
  dbg_txh4(*status);
  dbg_txs(" overflow=");
  dbg_txu(s_can_rx_overflow);
  dbg_txs(" stop=1\r\n");
  zdt_home_stop_repeat(SHENGJIANG_ADDR);
  if (!ok)
  {
    return false;
  }

  delay_ms1(60U);
  (void)zdt_clear_pos(SHENGJIANG_ADDR);
  delay_ms1(20U);
  current_pos1 = 0.0f;
  dbg_txs("# SJ HOMEZERO CLEAR driver_pos=0 software_pos=0.000\r\n");
  return true;
}

void race_chassis_init(void)
{
  can_filter_start();
  s_servo_pwm_started = 0U;
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  current_pos = 0.0f;
  current_pos1 = 0.0f;
  yuantai_angle = YUANTAI_DEFAULT_DEG;
  wukuai_angle = WUKUAI_POS1_DEG;
  s_can_rx_head = 0U;
  s_can_rx_tail = 0U;
  s_can_rx_overflow = 0U;
  (void)zdt_clear_pos(PINGTUI_ADDR);
  delay_ms1(5U);
  (void)zdt_clear_pos(SHENGJIANG_ADDR);
  delay_ms1(5U);
  race_heat_set(0U);

  dianji_move_flag = 1U;
  imu_init();
  PIDInit();
  race_actuator_reset_all();
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

void PID_Calc(PID *pid, float reference, float feedback)
{
  float dout;
  float pout;

  pid->lastError = pid->error;

  if ((reference - feedback) < -180.0f)
  {
    pid->error = reference - feedback + 360.0f;
  }
  else if ((reference - feedback) > 180.0f)
  {
    pid->error = reference - feedback - 360.0f;
  }
  else
  {
    pid->error = reference - feedback;
  }

  dout = (pid->error - pid->lastError) * pid->kd;
  pout = pid->error * pid->kp;
  pid->integral += pid->error * pid->ki;

  if (pid->integral > pid->maxIntegral)
  {
    pid->integral = pid->maxIntegral;
  }
  else if (pid->integral < -pid->maxIntegral)
  {
    pid->integral = -pid->maxIntegral;
  }

  pid->output = pout + dout + pid->integral;
  if (pid->output > pid->maxOutput)
  {
    pid->output = pid->maxOutput;
  }
  else if (pid->output < -pid->maxOutput)
  {
    pid->output = -pid->maxOutput;
  }
}

void PID_DIL(int v_x, int v_y, int w, uint32_t time, float target)
{
  vx = v_x;
  vy = v_y;
  omiga = w;
  delay_time = time;
  real_time = 0U;
  targetValue = target;
}

void race_chassis_tim4_callback(void)
{
  s_tim4_tick = 1U;
}

static void PID_tim4_run(void)
{
  if ((real_time <= delay_time) && (dianji_move_flag != 0U))
  {
    imu_scan(fAcc, fGyro, fAngle);
    feedbackValue = fAngle[2];
    PID_Calc(&mypid, targetValue, feedbackValue);
    if (feedbackValue == feedbackValue_pre)
    {
      same_time++;
    }
    feedbackValue_pre = fAngle[2];
    car_move(vx, vy, (int)mypid.output);
    flag = 0;
  }
  else if (PID_control_flag == 0U)
  {
    car_move(0, 0, 0);
    flag = 1;
  }

  real_time += 50U;
  circle++;
}

uint8_t PID_move(int v_x, int v_y, int w, uint32_t time, float target, int pid_choose)
{
  flag = 0;
  change_A(230);

  if (pid_choose == 0)
  {
    mypid.kp = 2.0f;
    mypid.ki = 0.0f;
    mypid.kd = 0.8f;
  }
  else if (pid_choose == 1)
  {
    mypid.kp = 3.3f;
    mypid.ki = 0.0f;
    mypid.kd = 1.8f;
    mypid.maxOutput = 30.0f;
  }
  else if (pid_choose == 2)
  {
    mypid.kp = 5.0f;
    mypid.ki = 0.0f;
    mypid.kd = 5.0f;
  }
  else if (pid_choose == 3)
  {
    mypid.kp = 2.0f;
    mypid.ki = 0.05f;
    mypid.kd = 0.4f;
  }
  else if (pid_choose == 4)
  {
    mypid.kp = 2.0f;
    mypid.ki = 0.07f;
    mypid.kd = 0.4f;
  }

  if (target > 180.0f)
  {
    target -= 360.0f;
  }
  else if (target < -180.0f)
  {
    target += 360.0f;
  }

  PID_DIL(v_x, v_y, w, time, target);
  s_tim4_tick = 0U;
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
  (void)HAL_TIM_Base_Start_IT(&htim4);

  while (flag == 0)
  {
    if (s_tim4_tick != 0U)
    {
      __disable_irq();
      s_tim4_tick = 0U;
      __enable_irq();
      PID_tim4_run();
    }
  }

  flag = 0;
  mypid.maxOutput = 230.0f;
  (void)HAL_TIM_Base_Stop_IT(&htim4);
  delay_ms1(100U);
  return 1U;
}

double shengjiang_control(int target_pos, uint16_t speed, uint8_t accel)
{
  int move_distance;
  uint8_t dir;
  uint32_t pulses;
  double time;

  if (target_pos > SHENGJIANG_MAX_POS)
  {
    target_pos = SHENGJIANG_MAX_POS;
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
    (void)zdt_en(SHENGJIANG_ADDR, true, false);
    delay_ms1(5U);
    (void)zdt_pos(SHENGJIANG_ADDR, dir, speed, accel, pulses, false, false);
  }

  time = shengjiang_move_delay((float)pulses, accel, speed);
  current_pos1 = (float)target_pos;
  return time;
}

float shengjiang_current_pos(void)
{
  return current_pos1;
}

void shengjiang_set_current_pos(float pos)
{
  if (pos < 0.0f)
  {
    pos = 0.0f;
  }
  if (pos > (float)SHENGJIANG_MAX_POS)
  {
    pos = (float)SHENGJIANG_MAX_POS;
  }
  current_pos1 = pos;
}

bool race_shengjiang_homezero(void)
{
  uint8_t status = 0U;

  return shengjiang_homezero_run(&status);
}

double pingtui_control(float target_pos, uint16_t speed, uint8_t accel)
{
  float move_distance;
  uint8_t dir;
  uint32_t pulses;
  double time;

  if (target_pos > PINGTUI_MAX_POS)
  {
    target_pos = PINGTUI_MAX_POS;
  }
  if (target_pos < PINGTUI_MIN_POS)
  {
    target_pos = PINGTUI_MIN_POS;
  }

  move_distance = target_pos - current_pos;
  dir = (move_distance > 0.0f) ? 0U : 1U;
  pulses = (uint32_t)(ABSF(move_distance) * 25.465f);

  if (move_distance != 0.0f)
  {
    (void)zdt_en(PINGTUI_ADDR, true, false);
    delay_ms1(5U);
    (void)zdt_pos(PINGTUI_ADDR, dir, speed, accel, pulses, false, false);
  }

  time = shengjiang_move_delay((float)pulses, accel, speed);
  current_pos = target_pos;
  return time;
}

float pingtui_current_pos(void)
{
  return current_pos;
}

void pingtui_set_current_pos(float pos)
{
  if (pos < PINGTUI_MIN_POS)
  {
    pos = PINGTUI_MIN_POS;
  }
  if (pos > PINGTUI_MAX_POS)
  {
    pos = PINGTUI_MAX_POS;
  }
  current_pos = pos;
}

void race_pingtui_clear_zero(void)
{
  pingtui_set_current_pos(0.0f);
  (void)zdt_clear_pos(PINGTUI_ADDR);
  delay_ms1(5U);
}

void race_actuator_stop_all(void)
{
  (void)zdt_stop_all();
  zdt_home_stop_repeat(PINGTUI_ADDR);
  zdt_home_stop_repeat(SHENGJIANG_ADDR);
  race_heat_set(0U);
}

void race_actuator_reset_all(void)
{
  set_zhuashou_kai();
  set_wukuaipingtai_weizhi(0x01);
  set_yuantai_Angle(YUANTAI_DEFAULT_DEG, yuantai_reset_ms);
  (void)pingtui_control(0.0f, 1000U, 200U);
  race_pingtui_clear_zero();
  (void)race_shengjiang_homezero();
}

void pingtui_reset(void)
{
  (void)pingtui_control(0.0f, 1000U, 200U);
  race_pingtui_clear_zero();
}

void set_yuantai_Angle(float target, float time)
{
  uint16_t target_pwm = yuantai_pwm_from_angle(target);

  set_yuantai_pwm_slow(target_pwm);
  yuantai_angle = target;
  delay_ms1((uint32_t)time);
}

void set_zhuashou_Angle(float target_abs, float speed)
{
  servo_pwm_start_if_needed();
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, servo_pulse_270(target_abs - 1.0f));
  delay_ms1((uint32_t)speed);
}

void set_wukuaipingtai_Angle(float target_abs, float speed)
{
  uint16_t target_pwm = wukuai_pwm_from_angle(target_abs);

  set_wukuai_pwm_slow(target_pwm);
  wukuai_angle = target_abs;
  delay_ms1((uint32_t)speed);
}

void set_zhuashou_he(void)
{
  set_zhuashou_Angle(ZHUASHOU_CLOSE_DEG, ZHUASHOU_WAIT_MS);
}

void set_zhuashou_kai(void)
{
  set_zhuashou_Angle(ZHUASHOU_OPEN_DEG, ZHUASHOU_WAIT_MS);
}

void set_wukuaipingtai_weizhi(int pos)
{
  if (pos == 0x01)
  {
    set_wukuaipingtai_Angle(WUKUAI_POS1_DEG, wukuai_wait_ms);
  }
  else if (pos == 0x02)
  {
    set_wukuaipingtai_Angle(WUKUAI_POS2_DEG, wukuai_wait_ms);
  }
  else if (pos == 0x03)
  {
    set_wukuaipingtai_Angle(WUKUAI_POS3_DEG, wukuai_wait_ms);
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
  set_yuantai_Angle(target, yuantai_move_ms);
  return (int)yuantai_move_ms;
}
