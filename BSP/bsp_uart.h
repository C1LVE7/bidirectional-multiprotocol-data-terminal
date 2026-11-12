#ifndef BSP_UART_H_
#define BSP_UART_H_

#include "main.h"

void BSP_UART_Init(void);
void bsp_log(const char * fmt, ...);

//RS485方向:拉高=发送,拉低=接收
void RS485_SetTx(void);
void RS485_SetRx(void);

#endif
