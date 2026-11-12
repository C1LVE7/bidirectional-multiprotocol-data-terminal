#ifndef BSP_WIFI_H_
#define BSP_WIFI_H_

#include "main.h"

//初始化:硬复位模块->开接收中断->AT同步->关回显->设station
int8_t BSP_WIFI_Init(void);

//连WiFi+连Broker+进透传,host只能传IP字符串
int8_t BSP_WIFI_TcpConnect(const char * host, uint16_t port);

//透传发送:返回实际发送字节数,<0失败
int32_t BSP_WIFI_Send(const uint8_t * buf, uint16_t len);

//透传接收:返回收到字节数,0=静默超时,<0链路未建立
int32_t BSP_WIFI_Recv(uint8_t * buf, uint16_t len);

//供USART3中断调用:把1字节写进接收环形缓冲
void BSP_WIFI_RxPush(uint8_t b);

//退出透传并关闭TCP
void BSP_WIFI_Close(void);

//查询透传链路是否已建立,1=已建立
uint8_t BSP_WIFI_IsLinked(void);

#endif
