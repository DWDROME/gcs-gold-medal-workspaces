#include "imu.h"
#include "usart.h"
#include "wit_c_sdk.h"

#define ACC_UPDATE   0x01
#define GYRO_UPDATE  0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE   0x08
#define READ_UPDATE  0x80

float realbaud = 0.0f;

static volatile char s_cDataUpdate = 0;
static volatile uint32_t s_rx_count = 0U;
static volatile uint32_t s_update_count = 0U;
static volatile uint32_t s_angle_update_count = 0U;
static uint8_t s_uart2_rx = 0U;

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

void imu_init(void)
{
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(SensorUartSend);
  WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
  Usart2Init(115200U);
}

void Usart2Init(unsigned int uiBaud)
{
  huart2.Init.BaudRate = uiBaud;
  (void)HAL_UART_Init(&huart2);
  (void)HAL_UART_Receive_IT(&huart2, &s_uart2_rx, 1U);
  realbaud = (float)uiBaud;
}

void imu_uart2_rx_cplt(void)
{
  s_rx_count++;
  WitSerialDataIn(s_uart2_rx);
  (void)HAL_UART_Receive_IT(&huart2, &s_uart2_rx, 1U);
}

uint32_t imu_rx_count(void)
{
  return s_rx_count;
}

uint32_t imu_update_count(void)
{
  return s_update_count;
}

uint32_t imu_angle_update_count(void)
{
  return s_angle_update_count;
}

void Uart2Send(unsigned char *p_data, unsigned int uiSize)
{
  (void)HAL_UART_Transmit(&huart2, p_data, (uint16_t)uiSize, 100U);
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
  Uart2Send(p_data, uiSize);
}

static void Delayms(uint16_t ucMs)
{
  HAL_Delay(ucMs);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
  uint32_t i;

  s_update_count++;
  for (i = 0U; i < uiRegNum; i++)
  {
    switch (uiReg)
    {
      case AZ:
        s_cDataUpdate |= ACC_UPDATE;
        break;
      case GZ:
        s_cDataUpdate |= GYRO_UPDATE;
        break;
      case HZ:
        s_cDataUpdate |= MAG_UPDATE;
        break;
      case Yaw:
        s_cDataUpdate |= ANGLE_UPDATE;
        s_angle_update_count++;
        break;
      default:
        s_cDataUpdate |= READ_UPDATE;
        break;
    }
    uiReg++;
  }
}

void imu_scan(float *fAcc, float *fGyro, float *fAngle)
{
  int i;
  char update = s_cDataUpdate;

  if (update == 0)
  {
    return;
  }

  for (i = 0; i < 3; i++)
  {
    fAcc[i] = (float)sReg[AX + i] / 32768.0f * 16.0f;
    fGyro[i] = (float)sReg[GX + i] / 32768.0f * 2000.0f;
    fAngle[i] = (float)sReg[Roll + i] / 32768.0f * 180.0f;
  }

  s_cDataUpdate = 0;
}
