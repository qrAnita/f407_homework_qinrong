#include "led.hpp"

void Led::Init()     { R_Off(); G_Off(); B_Off(); }

void Led::R_On()     { HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, LED_ON); }
void Led::R_Off()    { HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, LED_OFF); }
void Led::R_Toggle() { HAL_GPIO_TogglePin(LED_R_PORT, LED_R_PIN); }

void Led::G_On()     { HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, LED_ON); }
void Led::G_Off()    { HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, LED_OFF); }
void Led::G_Toggle() { HAL_GPIO_TogglePin(LED_G_PORT, LED_G_PIN); }

void Led::B_On()     { HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, LED_ON); }
void Led::B_Off()    { HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, LED_OFF); }
void Led::B_Toggle() { HAL_GPIO_TogglePin(LED_B_PORT, LED_B_PIN); }