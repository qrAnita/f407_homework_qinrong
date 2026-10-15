//
// Created by cosmosmount on 2025/9/2.
//

#include "bsp_pwm.hpp"
TIM_HandleTypeDef htim10 = {0};
static uint32_t PWM_GetTimerClock(TIM_TypeDef *TIMx)
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();

  if ((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM9) ||
      (TIMx == TIM10) || (TIMx == TIM11))
  {
    return ((RCC->CFGR & RCC_CFGR_PPRE2) == RCC_CFGR_PPRE2_DIV1) ? pclk2 : pclk2 * 2;
  }
  return ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1) ? pclk1 : pclk1 * 2;
}
void PWM_Init(void)
{
    PWM_Start(&htim10, TIM_CHANNEL_1);
    // TODO: 根据实际使用的定时器和通道完成 PWM 初始化。
}

void PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    if (htim == NULL || htim->Instance == NULL)
  {
    return;
  }

  if (HAL_TIM_PWM_Start(htim, Channel) != HAL_OK)
  {
    // 启动失败处理（可在此添加日志打印）
    return;
  }
    // TODO: 调用 HAL 启动指定定时器通道的 PWM，并检查返回状态。
}

void PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    if (htim == NULL || htim->Instance == NULL)
  {
    return;
  }

  HAL_TIM_PWM_Stop(htim, Channel);
    // TODO: 调用 HAL 停止指定定时器通道的 PWM。
}

void PWM_SetPeriod(TIM_HandleTypeDef *htim, float period_s)
{
  if (htim == NULL || htim->Instance == NULL || period_s <= 0.0f)
  {
    return;
  }

  uint32_t tim_clk = PWM_GetTimerClock(htim->Instance);
  uint32_t psc = htim->Instance->PSC;

  // ARR = 定时器时钟 / (PSC + 1) × 周期(秒) - 1
  uint32_t arr = (uint32_t)((float)tim_clk / (float)(psc + 1) * period_s) - 1;
  if (arr > 0xFFFF) // F4 通用定时器为 16 位
  {
    arr = 0xFFFF;
  }

  __HAL_TIM_SET_AUTORELOAD(htim, arr);
  htim->Instance->EGR = TIM_EGR_UG; // 产生更新事件，让新周期立即生效
    // TODO: 根据定时器时钟和预分频值，将秒转换为 ARR 并更新周期。
}

void PWM_SetDutyRatio(TIM_HandleTypeDef *htim, float dutyratio, uint32_t channel)
{
  if (htim == NULL || htim->Instance == NULL)
  {
    return;
  }

  // 占空比限幅到 [0, 1]
  if (dutyratio < 0.0f)
  {
    dutyratio = 0.0f;
  }
  if (dutyratio > 1.0f)
  {
    dutyratio = 1.0f;
  }

  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
  uint32_t ccr = (uint32_t)(dutyratio * (float)(arr + 1));
  if (ccr > arr + 1)
  {
    ccr = arr + 1;
  }

  __HAL_TIM_SET_COMPARE(htim, channel, ccr);
    // TODO: 将 [0, 1] 占空比换算为比较值，写入指定通道。
}
