#ifndef CHASSIS_CALIB_H
#define CHASSIS_CALIB_H

#include <stdint.h>

void calib_init(void);
void calib_poll(void);
uint8_t calib_handle(uint8_t c);
void calib_can(uint32_t id, uint32_t dlc, uint8_t *data, uint8_t ext);

#endif
