#ifndef APP_TASKS_H_
#define APP_TASKS_H_

#include "main.h"
#include "mqtt_client.h"
#include <stdint.h>

//应用初始化:创建互斥量,初始化外设驱动,创建FreeRTOS任务
int8_t App_Init(void);

//获取遥测数据快照
int8_t App_GetTelemetry(gateway_telemetry_t * out);

#endif
