#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

//HAL库的总入口头文件,间接带进各种宏,类型和函数声明
#include "stm32f4xx_hal.h"

/*
bsp_led对应宏
LED1为运行心跳灯
LED2为网络状态灯
*/
#define LED1_PORT   GPIOA
#define LED1_PIN    GPIO_PIN_6
#define LED2_PORT   GPIOA
#define LED2_PIN    GPIO_PIN_7

/*
RS485收发器方向脚
MAX3485的DE与/RE短接后共用PA1:高=发送,低=接收
*/
#define RS485_DE_PORT   GPIOA
#define RS485_DE_PIN    GPIO_PIN_1

/*
ESP8266 WiFi透传配置
RST由PB8控制,上电先给模块复位脉冲,把模块拽回确定状态
它的作用不是"退出残留透传":AT+CIPMODE不保存到Flash
复位后模块必然回到普通传输模式,不存在"仍停在透传态"这回事
仅支持2.4GHZ网络频段 网络连接 AT+CWJAP
*/
#define WIFI_RST_PORT   GPIOB
#define WIFI_RST_PIN    GPIO_PIN_8
#define WIFI_SSID       "12T"		//WIFI名称
#define WIFI_PASSWORD   "11111111"	//WIFI密码

/*
MQTT配置
broker只能填IP,透传方案走AT+CIPSTART,不经域名解析
EMQX公共服务器的IP是会变的,连不上了先在电脑上nslookup broker.emqx.io查当前IP再替换
*/
#define MQTT_BROKER_HOST        "35.172.255.228"
#define MQTT_BROKER_PORT        1883
#define MQTT_CLIENT_ID          "stm32f407_thrms_001" //必须唯一,同ID连接会被broker踢掉前一个
#define MQTT_USERNAME           ""      //留空=匿名连接
#define MQTT_PASSWORD           ""
#define MQTT_KEEPALIVE_S        30      //保活周期,单位秒

//主题,须与MQTTX里的订阅主题一致
#define MQTT_TOPIC_TELEMETRY    "thrms/001/telemetry"
#define MQTT_TOPIC_COMMAND      "thrms/001/command"
#define MQTT_TOPIC_STATUS       "thrms/001/status"
#define MQTT_TOPIC_RESULT       "thrms/001/result"


//任务周期
#define SENSOR_POLL_MS   2000U	//AHT20采集周期2s
#define TELEMETRY_PUBLISH_MS   5000U	//遥测上报周期5s

/*
六位数显表
中盛科技6位数码管,出厂:从机模式/地址1/9600-8-N-1/应答延迟30ms
F0-00是显示数据缓存区,FC06写入即显示,也是本从机唯一响应的寄存器
*/
#define SEG_SLAVE_ADDR   1U          //从机地址,出厂默认1
#define SEG_REG_RAW0     0x0000U     //F0-00:显示原始数据寄存器
#define SEG_POLL_MS      1500U       //写显示数据节拍,须小于从机超时(面板须设≥2s,否则报E.Con1)
#define SEG_ROTATE_MS    3000U       //自动轮显时温/湿度切换周期,须为SEG_POLL_MS整数倍

#endif
