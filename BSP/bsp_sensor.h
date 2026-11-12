#ifndef BSP_SENSOR_H_
#define BSP_SENSOR_H_

#include "main.h"
#include <stdint.h>

#define AHT20_ADDR       (0x38 << 1) //7位地址0x38,左移1位得到写地址

int8_t BSP_AHT20_Init(void);
int8_t BSP_AHT20_Read(int16_t * temp, int16_t * humi);

#endif
