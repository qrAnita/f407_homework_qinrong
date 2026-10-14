#include "bsp_usart.hpp"
#include "usart.h"
void USART_Init(void)
{
  USART1_Init();
  USART6_Init();
}

void USART6_Init()
{
  __HAL_UART_CLEAR_IDLEFLAG(&huart6);
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
  if (huart6.hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
  }
  // TODO: 配置 USART6 的 DMA 接收、发送及空闲中断，并启动接收。
};

void USART1_Init()
{
   __HAL_UART_CLEAR_IDLEFLAG(&huart1);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  // 关闭 DMA 半传输中断，只在接收完成/总线空闲时产生回调
  if (huart1.hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
  // TODO: 配置 USART1 的 DMA 接收、发送及空闲中断，并启动接收。
};

void USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, enum USART_Mode mode)
{
  if (huart == NULL || huart->Instance == NULL || pData == NULL || Size == 0)
  {
    return;
  }
  // 发送器未就绪（上一次发送未完成）时不启动新的发送
  if (huart->gState != HAL_UART_STATE_READY)
  {
    return;
  }

  switch (mode)
  {
  case USART_MODE_BLOCK:
    HAL_UART_Transmit(huart, pData, Size, HAL_MAX_DELAY);
    break;
  case USART_MODE_DMA:
    HAL_UART_Transmit_DMA(huart, pData, Size);
    break;
  case USART_MODE_IT:
    HAL_UART_Transmit_IT(huart, pData, Size);
    break;
  default:
    break;
  }
  // TODO: 检查参数，并按 mode 选择阻塞、DMA 或中断发送。
 
}

void USART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  if (huart == NULL || huart->Instance == NULL || pData == NULL || Size == 0)
  {
    return;
  }

  // DMA + 空闲中断接收，接收完成或总线空闲时触发 HAL_UARTEx_RxEventCallback
  HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size);
  if (huart->hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  // TODO: 选择接收方式并启动 HAL UART 接收。
}
