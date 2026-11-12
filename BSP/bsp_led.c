#include "bsp_led.h"

typedef struct
{
	GPIO_TypeDef * port;
    uint16_t       pin;
}LED_map;

static const LED_map led_map[2] = 
{
	{LED1_PORT, LED1_PIN},
    {LED2_PORT, LED2_PIN},
};

//设置指定LED状态
void BSP_LED_Set(uint8_t led, GPIO_PinState state)
{
    if(led < 2)
    HAL_GPIO_WritePin(led_map[led].port, led_map[led].pin, state);
}

//翻转指定LED
void BSP_LED_Toggle(uint8_t led)
{
    if(led < 2)
    HAL_GPIO_TogglePin(led_map[led].port, led_map[led].pin);
}
