#include "race_io.h"
#include "host_proto.h"
#include "usart.h"

uint8_t Serial_RxPacket[9];

void delay_ms1(uint32_t ms)
{
  HAL_Delay(ms);
}

void LED_Toggle0(void)
{
  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
}

void LED_Toggle1(void)
{
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

void LED_Toggle2(void)
{
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

uint8_t swith_out(void)
{
  return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET) ? 1U : 0U;
}

void Serial_CopyRxPacket(void)
{
  host_rx_copy(Serial_RxPacket);
}

uint8_t Serial_GetRxFlag(void)
{
  if (host_get_rx_flag() == 0U)
  {
    return 0U;
  }

  Serial_CopyRxPacket();
  return 1U;
}

void Serial_SendByte(uint8_t byte)
{
  host_send_byte(byte);
}

void HMISends(char *buf)
{
  uint16_t len = 0U;

  if (buf == 0)
  {
    return;
  }

  while (buf[len] != '\0')
  {
    len++;
  }

  if (len != 0U)
  {
    (void)HAL_UART_Transmit(&huart6, (uint8_t *)buf, len, 200U);
  }
}

void HMISendb(uint8_t byte)
{
  uint8_t i;

  for (i = 0U; i < 3U; i++)
  {
    (void)HAL_UART_Transmit(&huart6, &byte, 1U, 100U);
  }
}

void race_wait_host_ready(void)
{
  while (1)
  {
    LED_Toggle1();
    delay_ms1(200U);
    if (Serial_GetRxFlag() == 1U && host_is_handshake() == 1U)
    {
      break;
    }
  }
}

void race_wait_switch(void)
{
  while (swith_out() == 1U)
  {
    LED_Toggle1();
    delay_ms1(50U);
  }
}

void race_io_poll(void)
{
}
