#ifndef HOST_PROTO_H
#define HOST_PROTO_H

#include "usart.h"
#include <stdint.h>

/* ============================================================
 *  上位机(树莓派)串口协议 —— 移植自 yyb_stm32/SYSTEM/Serial.c
 *
 *  对端是树莓派上跑的 cv2_python/zongchengxu20250724chusai.py。
 *
 *  帧格式(PC -> STM32): 0xFF + 9字节ASCII载荷 + 0xFE, 共 11 字节
 *    例: 握手 "987654321"、检测失败 "000000004"、
 *        坐标结果 "{x:04}{y:04}{color}"。载荷不足 9 字节时 PC 端补 0x00。
 *  方向(STM32 -> PC)  : 单字节文本命令, 不补换行。
 *    取舍: 上位机用 pyserial readline() 读, 它靠 timeout=0.1s 超时取回,
 *          所以单字节即可, 无需 '\n'。这是旧固件已验证的行为, 不要擅自加换行。
 *
 *  用法契约:
 *    1) 上电调一次 host_init() —— 复位状态机并武装 HOST_UART 接收中断。
 *    2) 在 HAL_UART_RxCpltCallback 里, 命中 HOST_UART 时调 host_isr_rx_cplt()。
 *    3) 主循环轮询 host_get_rx_flag(); 为真时用 host_rx_at()/host_rx_copy() 取包。
 *    4) 要触发 PC 任务时, 主循环里调 host_send_byte()/host_send_cmd()。
 *
 *  串口角色配置(单一切换点):
 *    - 旧工程/当前对齐基线: 上位机协议口 = USART1(PA9/PA10)
 *    - 本地调试口(单字符 calib + 日志) = UART5(PC12/PD2)
 *    - 若以后接线改了, 把下面 HOST_UART / DBG_UART 两行对调即可, 其余代码不动
 *  HOST_UART / DBG_UART 直接引用 usart.c 里的全局句柄。
 * ============================================================ */
#define HOST_UART   huart1   /* 上位机帧协议口: USART1 (PA9/PA10) */
#define DBG_UART    huart5   /* 调试口: 单字符 calib + 可读日志 UART5 (PC12/PD2) */

#define HOST_RX_LEN 9U       /* 帧载荷固定 9 字节 */

/* 复位状态机/标志, 并在 HOST_UART 上武装单字节中断接收。上电调一次。 */
void host_init(void);

/* 收到一个字节后的处理: 推进收包状态机并重新武装接收中断。
   边界: 只在中断回调(HAL_UART_RxCpltCallback)里调, 不要在主循环调。 */
void host_isr_rx_cplt(void);

/* 读并清: 收到完整一帧返回 1, 否则 0(读后自动清零, 语义同旧 Serial_GetRxFlag)。
   主循环轮询用。 */
uint8_t host_get_rx_flag(void);

/* 取载荷第 i 字节(0..8); i 越界返回 0。 */
uint8_t host_rx_at(uint8_t i);

/* 拷出整包 9 字节到 out。边界: 调用方需保证 out 至少 9 字节。 */
void host_rx_copy(uint8_t *out);

/* STM32 -> PC: 发单字节命令。不补换行(见文件头取舍说明)。
   边界: 阻塞发送, 只在主循环调, 不要在 ISR 里调。 */
void host_send_byte(uint8_t b);
void host_send_cmd(char c);

/* STM32 -> PC: 发字符串到 '\0'(不补换行)。同样仅主循环调用。 */
void host_send_string(const char *s);

/* 当前包是否为握手帧 987654321(整包精确匹配)。 */
uint8_t host_is_handshake(void);

/* 当前包是否与给定 9 字符串逐字节相等; 用于识别约定整包信号(握手/失败码等)。 */
uint8_t host_packet_eq(const char *s);

#endif /* HOST_PROTO_H */
