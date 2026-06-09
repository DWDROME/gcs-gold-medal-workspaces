#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

void imu_init(void);
void imu_scan(float *fAcc, float *fGyro, float *fAngle);
uint32_t imu_rx_count(void);
uint32_t imu_update_count(void);
uint32_t imu_angle_update_count(void);
void imu_uart2_rx_cplt(void);
void Uart2Send(unsigned char *p_data, unsigned int uiSize);
void Usart2Init(unsigned int uiBaud);

#endif
