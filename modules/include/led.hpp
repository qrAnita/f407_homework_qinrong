#ifndef LED_HPP
#define LED_HPP

#include "gpio.h"

/* 引脚来源：Core/Src/gpio.c（PH10/11/12 = RGB 灯）
 * 若点亮后颜色对不上，交换下面的引脚定义即可 */
#define LED_R_PORT GPIOH
#define LED_R_PIN  GPIO_PIN_12
#define LED_G_PORT GPIOH
#define LED_G_PIN  GPIO_PIN_11
#define LED_B_PORT GPIOH
#define LED_B_PIN  GPIO_PIN_10

#define LED_ON  GPIO_PIN_SET    // 若灯逻辑反了（SET变灭），交换 ON/OFF 这两个宏
#define LED_OFF GPIO_PIN_RESET

namespace Led
{
    void Init();      // 全部熄灭
    void R_On();  void R_Off();  void R_Toggle();
    void G_On();  void G_Off();  void G_Toggle();
    void B_On();  void B_Off();  void B_Toggle();
}

#endif