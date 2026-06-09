#include "chassis_calib.h"
#include "hal_zdt.h"
#include "usart.h"
#include "tim.h"
#include "host_proto.h"   /* 取 DBG_UART 角色定义(串口角色单一切换点) */

static uint8_t st = 0U;
static uint32_t t1 = 0U;
static int x1 = 105;
static int y1 = 103;
static uint16_t v1 = 30U;
static uint16_t v2 = 20U;
static uint8_t a1 = 70U;
static uint8_t a2 = 40U;
static uint16_t s_servo2 = 520U;
static uint16_t s_servo3 = 500U;
static uint16_t s_servo4 = 1500U;
static uint32_t rx_id[5];
static uint8_t rx_dlc[5];
static uint8_t rx_dat[5][8];
static uint8_t rx_new[5];
static int rx_vel[5];
static int32_t rx_pos[5];

static void txh(uint8_t v);
static void tx32h(uint32_t v);
static void txi(int32_t v);
static void dec1(uint8_t i);
static void servo_apply(void);
static void servo_dump(void);
static void servo_set2(uint16_t v);
static void servo_set3(uint16_t v);
static void servo_set4(uint16_t v);
static void servo_step2(int d);
static void servo_step3(int d);
static void servo_step4(int d);

static void txs(const char *s)
{
  uint16_t n = 0U;

  while (s[n] != 0)
  {
    n++;
  }

  (void)HAL_UART_Transmit(&DBG_UART, (uint8_t *)s, n, 100U);
}

static void txu(uint32_t v)
{
  char b[12];
  int i = 10;

  b[11] = 0;
  if (v == 0U)
  {
    b[10] = '0';
    txs(&b[10]);
    return;
  }

  while (v != 0U && i >= 0)
  {
    b[i] = (char)('0' + (v % 10U));
    v /= 10U;
    i--;
  }

  txs(&b[i + 1]);
}

static void txi(int32_t v)
{
  if (v < 0)
  {
    txs("-");
    txu((uint32_t)(-v));
    return;
  }

  txu((uint32_t)v);
}

static void say(const char *a, uint32_t v, const char *b)
{
  txs(a);
  txu(v);
  txs(b);
}

static void txh(uint8_t v)
{
  char b[3];

  b[0] = (char)((v >> 4) < 10U ? ('0' + (v >> 4)) : ('A' + (v >> 4) - 10U));
  b[1] = (char)((v & 0x0FU) < 10U ? ('0' + (v & 0x0FU)) : ('A' + (v & 0x0FU) - 10U));
  b[2] = 0;
  txs(b);
}

static void tx32h(uint32_t v)
{
  txh((uint8_t)(v >> 24));
  txh((uint8_t)(v >> 16));
  txh((uint8_t)(v >> 8));
  txh((uint8_t)(v >> 0));
}

static void servo_apply(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, s_servo2);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, s_servo3);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, s_servo4);
}

static void servo_dump(void)
{
  txs("SERVO CH2=");
  txu(s_servo2);
  txs(" CH3=");
  txu(s_servo3);
  txs(" CH4=");
  txu(s_servo4);
  txs("\r\n");
}

static void servo_set2(uint16_t v)
{
  if (v < 500U)
  {
    v = 500U;
  }
  if (v > 2500U)
  {
    v = 2500U;
  }

  s_servo2 = v;
  servo_apply();
  txs("ZH PWM=");
  txu(s_servo2);
  txs("\r\n");
}

static void servo_set3(uint16_t v)
{
  if (v < 500U)
  {
    v = 500U;
  }
  if (v > 2500U)
  {
    v = 2500U;
  }

  s_servo3 = v;
  servo_apply();
  txs("PT PWM=");
  txu(s_servo3);
  txs("\r\n");
}

static void servo_set4(uint16_t v)
{
  if (v < 500U)
  {
    v = 500U;
  }
  if (v > 2500U)
  {
    v = 2500U;
  }

  s_servo4 = v;
  servo_apply();
  txs("YT PWM=");
  txu(s_servo4);
  txs("\r\n");
}

static void servo_step2(int d)
{
  servo_set2((uint16_t)((int)s_servo2 + d));
}

static void servo_step3(int d)
{
  servo_set3((uint16_t)((int)s_servo3 + d));
}

static void servo_step4(int d)
{
  servo_set4((uint16_t)((int)s_servo4 + d));
}

static void err(zdt_ret_t r)
{
  uint8_t i;

  txs("CMD ERR ");
  if (r == ZDT_ERR_ARG) txs("ARG");
  else if (r == ZDT_ERR_CAN) txs("CAN");
  else if (r == ZDT_ERR_MAIL) txs("MAIL");
  else txs("UNK");
  txs(" HE=");
  txu((uint32_t)HAL_CAN_GetError(&hcan1));
  txs(" HS=");
  txu((uint32_t)HAL_CAN_GetState(&hcan1));
  txs(" ID=");
  txu(zdt_last_id());
  txs(" DLC=");
  txu(zdt_last_dlc());
  txs(" BOX=");
  txu(zdt_last_box());
  txs(" TSR=0x");
  tx32h(zdt_last_tsr());
  txs(" ESR=0x");
  tx32h(zdt_last_esr());
  txs(" MSR=0x");
  tx32h(zdt_last_msr());
  txs(" D=");
  for (i = 0U; i < zdt_last_dlc() && i < 8U; i++)
  {
    txh(zdt_last_dat(i));
    if ((i + 1U) < zdt_last_dlc() && (i + 1U) < 8U)
    {
      txs(" ");
    }
  }
  txs("\r\n");
}

static void dump1(uint8_t i)
{
  uint8_t j;

  txs("RX ");
  txu(i);
  txs(" ID=");
  txu(rx_id[i]);
  txs(" DLC=");
  txu(rx_dlc[i]);
  txs(" D=");
  for (j = 0U; j < rx_dlc[i] && j < 8U; j++)
  {
    txh(rx_dat[i][j]);
    if ((j + 1U) < rx_dlc[i] && (j + 1U) < 8U)
    {
      txs(" ");
    }
  }
  txs("\r\n");
  dec1(i);
}

static void dec1(uint8_t i)
{
  int32_t p;
  int v;

  if (rx_dlc[i] >= 4U && rx_dat[i][0] == 0x35U)
  {
    v = ((int)rx_dat[i][2] << 8) | (int)rx_dat[i][3];
    if (rx_dat[i][1] != 0U)
    {
      v = -v;
    }
    rx_vel[i] = v;
    txs("VEL ");
    txu(i);
    txs(" = ");
    txi(v);
    txs(" RPM\r\n");
    return;
  }

  if (rx_dlc[i] >= 6U && rx_dat[i][0] == 0x36U)
  {
    p = ((int32_t)rx_dat[i][2] << 24) |
        ((int32_t)rx_dat[i][3] << 16) |
        ((int32_t)rx_dat[i][4] << 8) |
        ((int32_t)rx_dat[i][5] << 0);
    if (rx_dat[i][1] != 0U)
    {
      p = -p;
    }
    rx_pos[i] = p;
    txs("POS ");
    txu(i);
    txs(" = ");
    txi(p);
    txs(" pul\r\n");
    return;
  }

  if (rx_dlc[i] >= 3U && rx_dat[i][0] == 0x00U && rx_dat[i][1] == 0xEEU)
  {
    txs("DRV ERR ");
    txu(i);
    txs("\r\n");
  }
}

static zdt_ret_t stop4(void)
{
  return zdt_stop_all();
}

static zdt_ret_t one(uint8_t id, uint8_t dir, uint16_t vel, uint8_t acc)
{
  zdt_ret_t r;

  r = zdt_vel(id, dir, vel, acc, false);
  if (r != ZDT_OK)
  {
    return r;
  }

  st = 1U;
  t1 = HAL_GetTick() + 600U;
  return ZDT_OK;
}

static zdt_ret_t vel4(int s1, int s2, int s3, int s4, uint8_t acc)
{
  zdt_ret_t r;

  r = zdt_vel(1U, s1 >= 0 ? 1U : 0U, (uint16_t)(s1 >= 0 ? s1 : -s1), acc, false);
  if (r != ZDT_OK) return r;
  r = zdt_vel(2U, s2 >= 0 ? 0U : 1U, (uint16_t)(s2 >= 0 ? s2 : -s2), acc, false);
  if (r != ZDT_OK) return r;
  r = zdt_vel(3U, s3 >= 0 ? 0U : 1U, (uint16_t)(s3 >= 0 ? s3 : -s3), acc, false);
  if (r != ZDT_OK) return r;
  r = zdt_vel(4U, s4 >= 0 ? 1U : 0U, (uint16_t)(s4 >= 0 ? s4 : -s4), acc, false);
  if (r != ZDT_OK) return r;

  st = 1U;
  t1 = HAL_GetTick() + 1000U;
  return ZDT_OK;
}

static zdt_ret_t pos_x(int mm)
{
  uint32_t pul = (uint32_t)((mm >= 0 ? mm : -mm) * x1);
  zdt_ret_t r;

  r = zdt_pos(2U, mm >= 0 ? 0U : 1U, v2, a1, pul, false, true);
  if (r != ZDT_OK) return r;
  r = zdt_pos(1U, mm >= 0 ? 1U : 0U, v2, a1, pul, false, true);
  if (r != ZDT_OK) return r;
  r = zdt_pos(3U, mm >= 0 ? 0U : 1U, v2, a1, pul, false, true);
  if (r != ZDT_OK) return r;
  r = zdt_pos(4U, mm >= 0 ? 1U : 0U, v2, a1, pul, false, true);
  if (r != ZDT_OK) return r;
  r = zdt_sync(0U);
  if (r != ZDT_OK) return r;

  st = 2U;
  t1 = HAL_GetTick() + 2500U;
  return ZDT_OK;
}

static zdt_ret_t pos_y(int mm)
{
  uint32_t pul = (uint32_t)((mm >= 0 ? mm : -mm) * y1);
  zdt_ret_t r;

  r = zdt_pos(2U, mm >= 0 ? 1U : 0U, v2, a1, pul, false, true);
  if (r != ZDT_OK) return r;
  r = zdt_pos(1U, mm >= 0 ? 1U : 0U, v2, a1, pul, false, true);
  if (r != ZDT_OK) return r;
  r = zdt_pos(3U, mm >= 0 ? 1U : 0U, v2, a1, pul, false, true);
  if (r != ZDT_OK) return r;
  r = zdt_pos(4U, mm >= 0 ? 1U : 0U, v2, a1, pul, false, true);
  if (r != ZDT_OK) return r;
  r = zdt_sync(0U);
  if (r != ZDT_OK) return r;

  st = 2U;
  t1 = HAL_GetTick() + 2500U;
  return ZDT_OK;
}

void calib_init(void)
{
  uint8_t i;
  uint8_t j;

  st = 0U;
  t1 = 0U;
  servo_apply();
  for (i = 0U; i < 5U; i++)
  {
    rx_id[i] = 0U;
    rx_dlc[i] = 0U;
    rx_new[i] = 0U;
    rx_vel[i] = 0;
    rx_pos[i] = 0;
    for (j = 0U; j < 8U; j++)
    {
      rx_dat[i][j] = 0U;
    }
  }
}

void calib_poll(void)
{
  if (st == 0U)
  {
    return;
  }

  if ((int32_t)(HAL_GetTick() - t1) < 0)
  {
    return;
  }

  (void)stop4();
  if (st == 1U)
  {
    txs("STOP RUN\r\n");
  }
  else
  {
    txs("STOP POS\r\n");
  }
  st = 0U;
}

uint8_t calib_handle(uint8_t c)
{
  zdt_ret_t r = ZDT_ERR_ARG;

  switch (c)
  {
    case '0':
      r = stop4();
      if (r == ZDT_OK) txs("STOP ALL\r\n");
      break;
    case '1':
      r = one(1U, 1U, v1, a2);
      if (r == ZDT_OK) txs("RUN 1\r\n");
      break;
    case '2':
      r = one(2U, 0U, v1, a2);
      if (r == ZDT_OK) txs("RUN 2\r\n");
      break;
    case '3':
      r = one(3U, 0U, v1, a2);
      if (r == ZDT_OK) txs("RUN 3\r\n");
      break;
    case '4':
      r = one(4U, 1U, v1, a2);
      if (r == ZDT_OK) txs("RUN 4\r\n");
      break;
    case 'V':
      r = vel4((int)v1, (int)v1, (int)v1, (int)v1, a2);
      if (r == ZDT_OK) txs("RUN V\r\n");
      break;
    case 'W':
      r = vel4((int)v1, (int)v1, (int)v1, (int)v1, a2);
      if (r == ZDT_OK) txs("RUN W\r\n");
      break;
    case 'X':
      r = pos_x(100);
      if (r == ZDT_OK) say("POS X ", 100U, "\r\n");
      break;
    case 'x':
      r = pos_x(-100);
      if (r == ZDT_OK) say("POS X ", 100U, " NEG\r\n");
      break;
    case 'Y':
      r = pos_y(100);
      if (r == ZDT_OK) say("POS Y ", 100U, "\r\n");
      break;
    case 'y':
      r = pos_y(-100);
      if (r == ZDT_OK) say("POS Y ", 100U, " NEG\r\n");
      break;
    case 'P':
      rx_new[1] = 0U;
      rx_new[2] = 0U;
      rx_new[3] = 0U;
      rx_new[4] = 0U;
      r = zdt_read(1U, ZDT_S_CPOS);
      if (r == ZDT_OK) r = zdt_read(2U, ZDT_S_CPOS);
      if (r == ZDT_OK) r = zdt_read(3U, ZDT_S_CPOS);
      if (r == ZDT_OK) r = zdt_read(4U, ZDT_S_CPOS);
      if (r == ZDT_OK) txs("READ CPOS\r\n");
      break;
    case 'Q':
      rx_new[1] = 0U;
      rx_new[2] = 0U;
      rx_new[3] = 0U;
      rx_new[4] = 0U;
      r = zdt_read(1U, ZDT_S_VEL);
      if (r == ZDT_OK) r = zdt_read(2U, ZDT_S_VEL);
      if (r == ZDT_OK) r = zdt_read(3U, ZDT_S_VEL);
      if (r == ZDT_OK) r = zdt_read(4U, ZDT_S_VEL);
      if (r == ZDT_OK) txs("READ VEL\r\n");
      break;
    case 'C':
      err(ZDT_OK);
      return 1U;
    case 'S':
    case 's':
      servo_dump();
      return 1U;
    case 'T':
      servo_step4(-5);
      return 1U;
    case 't':
      servo_step4(5);
      return 1U;
    case 'M':
    case 'm':
      s_servo4 = 1500U;
      servo_apply();
      txs("YT MID\r\n");
      servo_dump();
      return 1U;
    case 'G':
      servo_step2(-20);
      return 1U;
    case 'g':
      servo_step2(20);
      return 1U;
    case 'H':
    case 'h':
      s_servo2 = 520U;
      servo_apply();
      txs("ZH MID\r\n");
      servo_dump();
      return 1U;
    case 'J':
    case 'j':
      servo_set3(1500U);
      return 1U;
    case 'L':
    case 'l':
      servo_set4(838U);
      return 1U;
    case 'N':
    case 'n':
      servo_set4(1500U);
      return 1U;
    case 'O':
    case 'o':
      servo_set4(2200U);
      return 1U;
    default:
      return 0U;
  }

  if (r != ZDT_OK)
  {
    err(r);
  }

  return 1U;
}

void calib_can(uint32_t id, uint32_t dlc, uint8_t *data, uint8_t ext)
{
  uint8_t i;
  uint8_t j;

  if (ext == 0U || data == 0)
  {
    return;
  }

  i = (uint8_t)((id >> 8) & 0xFFU);
  if (i == 0U || i > 4U)
  {
    return;
  }

  rx_id[i] = id;
  rx_dlc[i] = (uint8_t)dlc;
  for (j = 0U; j < dlc && j < 8U; j++)
  {
    rx_dat[i][j] = data[j];
  }
  rx_new[i] = 1U;
  dump1(i);
}
