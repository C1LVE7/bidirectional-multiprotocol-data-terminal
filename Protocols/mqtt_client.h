#ifndef MQTT_CLIENT_H_
#define MQTT_CLIENT_H_

#include "app_config.h"
#include <stdint.h>

//网关遥测数据:采集任务写,MQTT任务读
//互斥量g_tele_mutex在app_tasks.c内,保护本结构
typedef struct
{
	int16_t  temp;          //温度,单位0.1摄氏度
	int16_t  humi;          //湿度,单位0.1%RH
	uint8_t  sensor_valid;  //1=本次AHT20读取成功,0=失败(值保留上一轮)
} gateway_telemetry_t;

//下行命令回调:display_mode 0=0.0待机 1=温度 2=湿度 3=自动轮显
typedef void (*mqtt_cmd_cb_t)(uint8_t display_mode);

//初始化coreMQTT上下文(不联网)
void MQTT_Client_Init(void);

//连TCP+连Broker+订阅命令主题;0成功,负数失败
int8_t MQTT_Client_Connect(void);

//周期收包;0正常,负数表示需重连
int8_t MQTT_Client_Loop(void);

//发布遥测到MQTT_TOPIC_TELEMETRY;0成功
int8_t MQTT_Client_PublishTelemetry(const gateway_telemetry_t * t);

//发布上线/离线状态到MQTT_TOPIC_STATUS;0成功
int8_t MQTT_Client_PublishStatus(uint8_t online);

//发布命令回执到MQTT_TOPIC_RESULT;0成功
int8_t MQTT_Client_PublishResult(uint8_t display_mode, uint8_t ok);

//断开MQTT与TCP连接
void MQTT_Client_Disconnect(void);

//查询是否已连接,1=已连接
uint8_t MQTT_Client_IsConnected(void);

//注册下行命令回调
void MQTT_Client_SetCmdCallback(mqtt_cmd_cb_t cb);

#endif
