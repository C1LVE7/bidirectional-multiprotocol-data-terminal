/*
串口日志 USART1
bsp_log()是采用非半主机(vsnprintf模式),防止无调试器时卡死
使用printf为半主机模式,会向PC发送应答,无应答会卡死
*/
#include "bsp_uart.h"
#include "usart.h"
#include "app_config.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdio.h>
#include <stdarg.h>

static SemaphoreHandle_t log_mutex;

//RS485发送态:方向脚拉高
void RS485_SetTx(void)
{
	HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET);
}

//RS485接收态:方向脚拉低
void RS485_SetRx(void)
{
	HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
}

//初始化:创建保护日志的互斥量
void BSP_UART_Init(void)
{
	log_mutex = xSemaphoreCreateMutex();
	RS485_SetRx();
}

//printf方式的串口输出USART1
void bsp_log(const char * fmt, ...) //...=可变参数
{
	//如果不放在静态区,任务使用时创建会导致每个使用的任务增加256B栈空间
	static char buf[256];
	va_list ap; //一个"取值游标"
	int n;
	
	//互斥量过程:格式化+发送
	if(log_mutex != NULL) //判断是否创建成功
	xSemaphoreTake(log_mutex,portMAX_DELAY);
	
	//初始化:从fmt之后的第一个参数开始
	va_start(ap, fmt);
	//把fmt和参数"格式化"进 buf
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
	//收尾(释放游标)
    va_end(ap);
	
	if(n > 0)
	{
		//防止越界,越界部分不读取
		if((unsigned int)n >= sizeof(buf))
		//n=最多写多少字节(含结尾'\0',所以实际最多n−1个字符)
		n = (int)sizeof(buf)-1;
		
		HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 100);
	}
	
	if(log_mutex != NULL)
	xSemaphoreGive(log_mutex);
}
