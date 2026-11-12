#ifndef BSP_LED_H_
#define BSP_LED_H_

#include "main.h"
#include "app_config.h"

//低电平点亮,高电平熄灭
#define LED_ON    GPIO_PIN_RESET
#define LED_OFF   GPIO_PIN_SET

//对应app_config设置
#define LED1   0
#define LED2   1

void BSP_LED_Set(uint8_t led, GPIO_PinState state);
void BSP_LED_Toggle(uint8_t led);

#endif
