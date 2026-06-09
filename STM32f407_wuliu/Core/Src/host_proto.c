#include "host_proto.h"

#define HOST_RX_DEBUG 0U

/* 接收数据包(9字节ASCII载荷)、收包状态机、标志位。
   volatile: 这几个由接收中断(host_isr_rx_cplt 经 RxCplt 回调)写、主循环读,
   跨中断/主循环共享, 必须 volatile 防编译器把读优化掉。
   s_rx_byte 是 HAL 单字节落点, 仅在中断上下文内读写, 不跨上下文, 故不加。 */
static volatile uint8_t s_rx_packet[HOST_RX_LEN];
static volatile uint8_t s_rx_flag;
static volatile uint8_t s_rx_state;   /* 0:等帧头 1:收载荷 2:等帧尾 */
static volatile uint8_t s_rx_pos;
static uint8_t s_rx_byte;             /* HAL 单字节接收缓冲(仅中断内用) */

#if HOST_RX_DEBUG
static void host_dbg_rx_byte(uint8_t b)
{
  static const char hex[] = "0123456789ABCDEF";
  char line[] = "# HRX 00\r\n";

  line[6] = hex[(b >> 4) & 0x0FU];
  line[7] = hex[b & 0x0FU];
  (void)HAL_UART_Transmit(&DBG_UART, (uint8_t *)line, (uint16_t)(sizeof(line) - 1U), 10U);
}
#endif

/* 收包状态机 —— 逐状态对齐 yyb_stm32/SYSTEM/Serial.c::USART1_IRQHandler。
   注意状态 2: 收到非 0xFE 时停在状态 2 继续等 0xFE, 不重置不回退。
   这是旧固件原始行为(帧尾恒为 0xFE, 实测可用), 有意保持一致, 不加额外重同步。 */
static void host_on_rx_byte(uint8_t b)
{
  if (s_rx_state == 0U)             /* 等帧头 0xFF */
  {
    if (b == 0xFFU)
    {
      s_rx_state = 1U;
      s_rx_pos = 0U;
    }
  }
  else if (s_rx_state == 1U)        /* 收 9 字节载荷 */
  {
    s_rx_packet[s_rx_pos] = b;
    s_rx_pos++;
    if (s_rx_pos >= HOST_RX_LEN)
    {
      s_rx_state = 2U;
    }
  }
  else if (s_rx_state == 2U)        /* 等帧尾 0xFE */
  {
    if (b == 0xFEU)                 /* 仅帧尾置位标志并归零状态(对齐旧逻辑) */
    {
      s_rx_state = 0U;
      s_rx_flag = 1U;
    }
  }
  else
  {
    s_rx_state = 0U;
  }
}

void host_init(void)
{
  uint8_t i;

  s_rx_flag = 0U;
  s_rx_state = 0U;
  s_rx_pos = 0U;
  for (i = 0U; i < HOST_RX_LEN; i++)
  {
    s_rx_packet[i] = 0U;
  }
  /* 武装单字节中断接收; 之后每收 1 字节触发一次 RxCplt 回调 */
  (void)HAL_UART_Receive_IT(&HOST_UART, &s_rx_byte, 1U);
}

void host_isr_rx_cplt(void)
{
  /* 中断上下文: 处理刚收到的字节, 再立即重新武装下一字节接收 */
#if HOST_RX_DEBUG
  host_dbg_rx_byte(s_rx_byte);
#endif
  host_on_rx_byte(s_rx_byte);
  (void)HAL_UART_Receive_IT(&HOST_UART, &s_rx_byte, 1U);
}

uint8_t host_get_rx_flag(void)
{
  if (s_rx_flag == 1U)
  {
    s_rx_flag = 0U;            /* 读后即清, 一帧只被消费一次 */
    return 1U;
  }
  return 0U;
}

uint8_t host_rx_at(uint8_t i)
{
  if (i >= HOST_RX_LEN)
  {
    return 0U;
  }
  return s_rx_packet[i];
}

void host_rx_copy(uint8_t *out)
{
  uint8_t i;

  if (out == 0)
  {
    return;
  }
  for (i = 0U; i < HOST_RX_LEN; i++)
  {
    out[i] = s_rx_packet[i];
  }
}

void host_send_byte(uint8_t b)
{
  /* 阻塞发送一字节; 不补换行(对端 readline 靠 0.1s 超时取回, 见 .h 取舍说明) */
  (void)HAL_UART_Transmit(&HOST_UART, &b, 1U, 100U);
}

void host_send_cmd(char c)
{
  host_send_byte((uint8_t)c);
}

void host_send_string(const char *s)
{
  uint16_t n = 0U;

  if (s == 0)
  {
    return;
  }
  while (s[n] != '\0')
  {
    n++;
  }
  if (n > 0U)
  {
    (void)HAL_UART_Transmit(&HOST_UART, (uint8_t *)s, n, 200U);
  }
}

uint8_t host_is_handshake(void)
{
  /* 上位机启动时发 "987654321" 作握手; 这里要求整包精确匹配 */
  return host_packet_eq("987654321");
}

/* 当前包是否与给定 9 字节串逐字节相等。用于识别约定好的整包信号
   (如握手 987654321、检测失败 000000004)。入参 s 需为 9 字符。 */
uint8_t host_packet_eq(const char *s)
{
  uint8_t i;

  if (s == 0)
  {
    return 0U;
  }

  for (i = 0U; i < HOST_RX_LEN; i++)
  {
    if ((uint8_t)s[i] != s_rx_packet[i])
    {
      return 0U;
    }
  }

  return 1U;
}
