#ifndef HAL_ZDT_H
#define HAL_ZDT_H

#include "can.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  ZDT_OK = 0,
  ZDT_ERR_ARG = -1,
  ZDT_ERR_CAN = -2,
  ZDT_ERR_MAIL = -3
} zdt_ret_t;

typedef enum
{
  ZDT_S_VER = 0,
  ZDT_S_RL = 1,
  ZDT_S_PID = 2,
  ZDT_S_VBUS = 3,
  ZDT_S_CPHA = 5,
  ZDT_S_ENCL = 7,
  ZDT_S_TPOS = 8,
  ZDT_S_VEL = 9,
  ZDT_S_CPOS = 10,
  ZDT_S_PERR = 11,
  ZDT_S_FLAG = 13,
  ZDT_S_CONF = 14,
  ZDT_S_STATE = 15,
  ZDT_S_ORG = 16
} zdt_sys_t;

void zdt_set_wait_enabled(bool enabled);
bool zdt_wait_enabled(void);
zdt_ret_t zdt_tx(uint8_t *cmd, uint8_t len);
zdt_ret_t zdt_en(uint8_t addr, bool on, bool sync);
zdt_ret_t zdt_vel(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool sync);
zdt_ret_t zdt_pos(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool abs, bool sync);
zdt_ret_t zdt_stop(uint8_t addr, bool sync);
zdt_ret_t zdt_sync(uint8_t addr);
zdt_ret_t zdt_read(uint8_t addr, zdt_sys_t s);
zdt_ret_t zdt_stop_all(void);
uint32_t zdt_last_id(void);
uint32_t zdt_last_dlc(void);
uint32_t zdt_last_box(void);
uint32_t zdt_last_tsr(void);
uint32_t zdt_last_esr(void);
uint32_t zdt_last_msr(void);
uint8_t zdt_last_dat(uint8_t i);

#endif
