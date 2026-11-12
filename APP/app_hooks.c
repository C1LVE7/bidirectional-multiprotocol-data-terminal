#include "FreeRTOS.h"   
#include "task.h"       

//栈溢出钩子
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    taskDISABLE_INTERRUPTS();
	
    while(1);
}

//内存申请失败钩子
void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
	
    while(1);
}
