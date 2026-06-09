#ifndef RACE_IO_H
#define RACE_IO_H

#include "main.h"
#include <stdint.h>

extern uint8_t Serial_RxPacket[9];

void delay_ms1(uint32_t ms);
void LED_Toggle0(void);
void LED_Toggle1(void);
void LED_Toggle2(void);
uint8_t swith_out(void);

void Serial_SendByte(uint8_t byte);
uint8_t Serial_GetRxFlag(void);
void Serial_CopyRxPacket(void);

void HMISends(char *buf);
void HMISendb(uint8_t byte);

void race_wait_host_ready(void);
void race_wait_switch(void);
void race_io_poll(void);

#endif
