/*
 * core_mqtt_config.h
 * ---------------------------------------------------------------------------
 * coreMQTT 库的自定义配置文件。
 *
 * coreMQTT 默认会通过 core_mqtt_config_defaults.h 引入本文件（除非定义了
 * MQTT_DO_NOT_USE_CUSTOM_CONFIG）。这里把 coreMQTT 内部的日志宏 LogError /
 * LogWarn / LogInfo / LogDebug / LogTrace 重定向到本工程的串口日志函数
 * bsp_log()，这样 MQTT 协议栈的运行状态就能在串口调试助手上直接看到。
 *
 * 注意：coreMQTT 调用这些日志宏时使用「双重括号」写法，例如：
 *     LogError( ("connect failed, status=%d", status) );
 * 因此下面的宏写成 bsp_log message 即可正确展开为一次函数调用。
 */
#ifndef CORE_MQTT_CONFIG_H_
#define CORE_MQTT_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* bsp_log 由 BSP/bsp_uart.c 提供，是一个 printf 风格的串口日志函数。 */
extern void bsp_log( const char * fmt, ... );

/* 日志等级前缀 + 内容 + 换行，统一通过串口输出。
 * 只保留 Error/Warn/Info 三个等级即可满足调试需求。
 *
 * LogDebug / LogTrace 保持为空宏，原因有二：
 *   1. coreMQTT 内部的 LogTrace 会用到 strerror(errno)，而该源文件未包含
 *      <errno.h>，开启会导致编译报错；
 *   2. Debug/Trace 等级日志非常频繁，会淹没 115200 串口，影响调试。 */
#define LogError( message )    do { bsp_log( "[MQTT-E] " ); bsp_log message; bsp_log( "\r\n" ); } while( 0 )
#define LogWarn( message )     do { bsp_log( "[MQTT-W] " ); bsp_log message; bsp_log( "\r\n" ); } while( 0 )
#define LogInfo( message )     do { bsp_log( "[MQTT-I] " ); bsp_log message; bsp_log( "\r\n" ); } while( 0 )
#define LogDebug( message )
#define LogTrace( message )

/* ============================ 保活/断线检测参数 ========================== */
/* ESP8266 透传模式下，TCP 断线无法被底层直接感知，只能靠 coreMQTT 定期发
 * PINGREQ 探测（PINGRESP 超时判断线）。本网关上行遥测为 QoS0（Broker 无 ACK），
 * 正常运行时也长期收不到数据，因此断线检测由「RX 空闲超时」触发 PINGREQ：
 *
 *   断线检测时间 ≈ PACKET_RX_TIMEOUT_MS + MQTT_PINGRESP_TIMEOUT_MS
 *
 * 默认值 30000ms + 5000ms = 35s。这里把 RX 空闲超时缩短到 10s，使断线后约
 * 15s 内即可自动重连（PINGREQ 仅 2 字节，流量开销可忽略）。 */
#define PACKET_RX_TIMEOUT_MS    ( 10000U )   /* RX 空闲 10s 即发 PINGREQ 探测 */

#ifdef __cplusplus
}
#endif

#endif /* CORE_MQTT_CONFIG_H_ */
