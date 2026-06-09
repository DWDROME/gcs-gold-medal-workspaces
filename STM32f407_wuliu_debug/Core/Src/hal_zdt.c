#include "hal_zdt.h"

static uint32_t s_id = 0U;
static uint32_t s_dlc = 0U;
static uint32_t s_box = 0U;
static uint32_t s_tsr = 0U;
static uint32_t s_esr = 0U;
static uint32_t s_msr = 0U;
static uint8_t s_dat[8] = {0};
static bool s_wait_enabled = true;

static void zdt_snap(uint32_t id, uint32_t dlc, uint32_t box, uint8_t *data)
{
  uint8_t i;

  s_id = id;
  s_dlc = dlc;
  s_box = box;
  s_tsr = hcan1.Instance->TSR;
  s_esr = hcan1.Instance->ESR;
  s_msr = hcan1.Instance->MSR;

  for (i = 0U; i < 8U; i++)
  {
    s_dat[i] = data[i];
  }
}

static zdt_ret_t zdt_wait(uint32_t mbox)
{
  uint32_t t0 = HAL_GetTick();

  while (HAL_CAN_IsTxMessagePending(&hcan1, mbox) != 0U)
  {
    if ((HAL_GetTick() - t0) > 50U)
    {
      zdt_snap(s_id, s_dlc, mbox, s_dat);
      return ZDT_ERR_MAIL;
    }
  }

  zdt_snap(s_id, s_dlc, mbox, s_dat);
  return ZDT_OK;
}

void zdt_set_wait_enabled(bool enabled)
{
  s_wait_enabled = enabled;
}

bool zdt_wait_enabled(void)
{
  return s_wait_enabled;
}

zdt_ret_t zdt_tx(uint8_t *cmd, uint8_t len)
{
  CAN_TxHeaderTypeDef tx = {0};
  uint8_t data[8] = {0};
  uint8_t i = 0U;
  uint8_t j;
  uint8_t k;
  uint8_t l;
  uint8_t pack = 0U;
  uint32_t mbox = 0U;
  zdt_ret_t ret;

  if (cmd == 0 || len < 3U)
  {
    return ZDT_ERR_ARG;
  }

  j = (uint8_t)(len - 2U);

  while (i < j)
  {
    k = (uint8_t)(j - i);

    tx.StdId = 0U;
    tx.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)pack;
    tx.IDE = CAN_ID_EXT;
    tx.RTR = CAN_RTR_DATA;
    tx.TransmitGlobalTime = DISABLE;

    data[0] = cmd[1];
    for (l = 1U; l < 8U; l++)
    {
      data[l] = 0U;
    }

    if (k < 8U)
    {
      for (l = 0U; l < k; l++, i++)
      {
        data[l + 1U] = cmd[i + 2U];
      }
      tx.DLC = k + 1U;
    }
    else
    {
      for (l = 0U; l < 7U; l++, i++)
      {
        data[l + 1U] = cmd[i + 2U];
      }
      tx.DLC = 8U;
    }

    if (HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mbox) != HAL_OK)
    {
      zdt_snap(tx.ExtId, tx.DLC, mbox, data);
      return ZDT_ERR_CAN;
    }

    zdt_snap(tx.ExtId, tx.DLC, mbox, data);
    if (s_wait_enabled)
    {
      ret = zdt_wait(mbox);
      if (ret != ZDT_OK)
      {
        return ret;
      }
    }

    pack++;
  }

  return ZDT_OK;
}

zdt_ret_t zdt_en(uint8_t addr, bool on, bool sync)
{
  uint8_t cmd[6];

  cmd[0] = addr;
  cmd[1] = 0xF3;
  cmd[2] = 0xAB;
  cmd[3] = on ? 1U : 0U;
  cmd[4] = sync ? 1U : 0U;
  cmd[5] = 0x6B;
  return zdt_tx(cmd, 6U);
}

zdt_ret_t zdt_vel(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool sync)
{
  uint8_t cmd[8];

  cmd[0] = addr;
  cmd[1] = 0xF6;
  cmd[2] = dir;
  cmd[3] = (uint8_t)(vel >> 8);
  cmd[4] = (uint8_t)(vel >> 0);
  cmd[5] = acc;
  cmd[6] = sync ? 1U : 0U;
  cmd[7] = 0x6B;
  return zdt_tx(cmd, 8U);
}

zdt_ret_t zdt_pos(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool abs, bool sync)
{
  uint8_t cmd[13];

  cmd[0] = addr;
  cmd[1] = 0xFD;
  cmd[2] = dir;
  cmd[3] = (uint8_t)(vel >> 8);
  cmd[4] = (uint8_t)(vel >> 0);
  cmd[5] = acc;
  cmd[6] = (uint8_t)(clk >> 24);
  cmd[7] = (uint8_t)(clk >> 16);
  cmd[8] = (uint8_t)(clk >> 8);
  cmd[9] = (uint8_t)(clk >> 0);
  cmd[10] = abs ? 1U : 0U;
  cmd[11] = sync ? 1U : 0U;
  cmd[12] = 0x6B;
  return zdt_tx(cmd, 13U);
}

zdt_ret_t zdt_stop(uint8_t addr, bool sync)
{
  uint8_t cmd[5];

  cmd[0] = addr;
  cmd[1] = 0xFE;
  cmd[2] = 0x98;
  cmd[3] = sync ? 1U : 0U;
  cmd[4] = 0x6B;
  return zdt_tx(cmd, 5U);
}

zdt_ret_t zdt_sync(uint8_t addr)
{
  uint8_t cmd[4];

  cmd[0] = addr;
  cmd[1] = 0xFF;
  cmd[2] = 0x66;
  cmd[3] = 0x6B;
  return zdt_tx(cmd, 4U);
}

zdt_ret_t zdt_read(uint8_t addr, zdt_sys_t s)
{
  uint8_t cmd[5];
  uint8_t i = 0U;

  cmd[i++] = addr;

  switch (s)
  {
    case ZDT_S_VER: cmd[i++] = 0x1F; break;
    case ZDT_S_RL: cmd[i++] = 0x20; break;
    case ZDT_S_PID: cmd[i++] = 0x21; break;
    case ZDT_S_VBUS: cmd[i++] = 0x24; break;
    case ZDT_S_CPHA: cmd[i++] = 0x27; break;
    case ZDT_S_ENCL: cmd[i++] = 0x31; break;
    case ZDT_S_TPOS: cmd[i++] = 0x33; break;
    case ZDT_S_VEL: cmd[i++] = 0x35; break;
    case ZDT_S_CPOS: cmd[i++] = 0x36; break;
    case ZDT_S_PERR: cmd[i++] = 0x37; break;
    case ZDT_S_FLAG: cmd[i++] = 0x3A; break;
    case ZDT_S_ORG: cmd[i++] = 0x3B; break;
    case ZDT_S_CONF:
      cmd[i++] = 0x42;
      cmd[i++] = 0x6C;
      break;
    case ZDT_S_STATE:
      cmd[i++] = 0x43;
      cmd[i++] = 0x7A;
      break;
    default:
      return ZDT_ERR_ARG;
  }

  cmd[i++] = 0x6B;
  return zdt_tx(cmd, i);
}

zdt_ret_t zdt_stop_all(void)
{
  zdt_ret_t r;

  r = zdt_stop(1U, false);
  if (r != ZDT_OK) return r;
  r = zdt_stop(2U, false);
  if (r != ZDT_OK) return r;
  r = zdt_stop(3U, false);
  if (r != ZDT_OK) return r;
  r = zdt_stop(4U, false);
  if (r != ZDT_OK) return r;
  return ZDT_OK;
}

uint32_t zdt_last_id(void)
{
  return s_id;
}

uint32_t zdt_last_dlc(void)
{
  return s_dlc;
}

uint32_t zdt_last_box(void)
{
  return s_box;
}

uint32_t zdt_last_tsr(void)
{
  return s_tsr;
}

uint32_t zdt_last_esr(void)
{
  return s_esr;
}

uint32_t zdt_last_msr(void)
{
  return s_msr;
}

uint8_t zdt_last_dat(uint8_t i)
{
  if (i >= 8U)
  {
    return 0U;
  }

  return s_dat[i];
}
