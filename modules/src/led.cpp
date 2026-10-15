#include "led.hpp"

void Led::Init()
{
    HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, LED_OFF);
    HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, LED_OFF);
    HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, LED_OFF);
}

void Led::R_On()    { HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, LED_ON); }
void Led::R_Off()   { HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, LED_OFF); }
void Led::R_Toggle(){ HAL_GPIO_TogglePin(LED_R_PORT, LED_R_PIN); }
// G、B 同理复制