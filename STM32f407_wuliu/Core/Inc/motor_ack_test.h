#ifndef MOTOR_ACK_TEST_H
#define MOTOR_ACK_TEST_H

#include "can.h"
#include "usart.h"

void motor_ack_test_init(void);
void motor_ack_test_poll(void);
void motor_ack_test_uart_rx_callback(UART_HandleTypeDef *huart);
void motor_ack_test_can_rx_callback(CAN_HandleTypeDef *hcan);

#endif /* MOTOR_ACK_TEST_H */
