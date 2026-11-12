#include "app_tasks.h"
#include "app_config.h"
#include "bsp_led.h"
#include "bsp_uart.h"
#include "bsp_sensor.h"
#include "bsp_wifi.h"
#include "mqtt_client.h"
#include "modbus_rtu_master.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

//显示模式:取值与云端display命令一致
#define DISP_MODE_OFF    0U      //0.0待机
#define DISP_MODE_TEMP   1U      //固定显示温度
#define DISP_MODE_HUMI   2U      //固定显示湿度
#define DISP_MODE_AUTO   3U      //自动轮显,温湿每SEG_ROTATE_MS切换

//共享遥测数据
static gateway_telemetry_t g_telemetry;
//保护遥测数据的互斥量
static SemaphoreHandle_t g_tele_mutex;
//当前显示模式:MQTT任务(命令回调)写,显示任务每轮读
static volatile uint8_t g_disp_mode;
//回执队列:显示任务入队->MQTT任务出队发布
static QueueHandle_t g_result_queue;

//函数声明
static void vTaskLed(void *pvParameters);
static void vTaskSensor(void *pvParameters);
static void vTaskMqtt(void *pvParameters);
static void vTaskDisplay(void *pvParameters);

//回执项:显示任务写完从机入队,MQTT任务统一发布到result主题
typedef struct
{
	uint8_t mode;   //目标显示模式0/1/2/3
	uint8_t ok;     //写从机成功=1
} disp_result_t;

//MQTT下行display命令回调(协议层已校验值为0/1/2/3):只写模式变量,
//实际写从机由显示任务串行执行(Modbus的USART2由显示任务独占)
static void on_display_cmd(uint8_t display_mode)
{
	g_disp_mode = display_mode;
	bsp_log("CMD display=%u\r\n", (unsigned)display_mode);
}

/*
应用初始化:创建互斥量,初始化外设驱动,创建 FreeRTOS 任务
0为成功,-1为失败
*/
int8_t App_Init(void)
{
    //创建互斥量与回执队列:内核堆不足会返回NULL
    g_tele_mutex = xSemaphoreCreateMutex();
    g_result_queue = xQueueCreate(4, sizeof(disp_result_t));
    if(g_tele_mutex == NULL || g_result_queue == NULL)
    {
        bsp_log("FATAL: IPC create failed (heap low)\r\n");
        return -1;
    }

	//初始化ESP8266:失败不阻断,后续TcpConnect里有硬复位自愈
    if(BSP_WIFI_Init() != 0)
    bsp_log("WiFi init failed, check ESP8266 wiring\r\n");
    //初始化AHT20
    if(BSP_AHT20_Init() != 0)
    bsp_log("AHT20 init failed, check I2C wiring\r\n");

    //初始化coreMQTT上下文
    MQTT_Client_Init();
	//注册下行display命令回调:收到{"display":n}时更新显示模式
    MQTT_Client_SetCmdCallback(on_display_cmd);

    //创建LED任务
    if(xTaskCreate(vTaskLed, "led", 128, NULL, 1, NULL) != pdPASS)
    bsp_log("FATAL: task 'led' create failed\r\n");
	//创建采集任务
    if(xTaskCreate(vTaskSensor, "sensor", 256, NULL, 2, NULL) != pdPASS)
    bsp_log("FATAL: task 'sensor' create failed\r\n");
	//创建显示任务
    if(xTaskCreate(vTaskDisplay, "disp", 384, NULL, 3, NULL) != pdPASS)
    bsp_log("FATAL: task 'disp' create failed\r\n");
	//创建MQTT任务
    if(xTaskCreate(vTaskMqtt, "mqtt", 768, NULL, 4, NULL) != pdPASS)
    bsp_log("FATAL: task 'mqtt' create failed\r\n");

    return 0;
}

/*
获取遥测数据快照:互斥量保护下整体拷贝,调用者拿到一致副本
out不能为NULL;成功返回0,失败返回-1
*/
int8_t App_GetTelemetry(gateway_telemetry_t * out)
{
	if(out == NULL) return -1;

	xSemaphoreTake(g_tele_mutex, portMAX_DELAY);
	*out = g_telemetry;
	xSemaphoreGive(g_tele_mutex);

	return 0;
}

//心跳灯
static void vTaskLed(void *pvParameters)
{
    while(1)
    {
        BSP_LED_Toggle(LED1);
		//pdMS_TO_TICKS将节拍变为真正的ms防止系统节拍不是1000hz
        vTaskDelay(pdMS_TO_TICKS(500));	//vTaskDelay记的是节拍数
    }
}

//采集任务:AHT20读温湿度,写入共享数据
static void vTaskSensor(void *pvParameters)
{
    int16_t t = 0, h = 0; //局部变量放最前,保留上轮值

    while(1)
    {
        if(BSP_AHT20_Read(&t, &h) == 0)
        {
            xSemaphoreTake(g_tele_mutex, portMAX_DELAY);
            g_telemetry.temp = t;
            g_telemetry.humi = h;
            g_telemetry.sensor_valid = 1;
            xSemaphoreGive(g_tele_mutex);
        }
        else
        {
            xSemaphoreTake(g_tele_mutex, portMAX_DELAY);
            g_telemetry.sensor_valid = 0; //只清标志,temp/humi保留上一轮值
            xSemaphoreGive(g_tele_mutex);
        }
		
        vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_MS));
    }
}

//MQTT任务:建连维护+周期上报遥测,连接失败按指数退避重试,收包失败即断开重连
static void vTaskMqtt(void *pvParameters)
{
	//上次发布时刻,0=连上后立即发一次
	uint32_t last_publish = 0;
	//连接失败重试间隔,指数退避2->4->8->16->30s封顶
	uint32_t retry_ms = 2000;
	gateway_telemetry_t snap;
	disp_result_t r;

	while(1)
	{
		//未连接:灭灯后连TCP+CONNECT+SUBSCRIBE
		if(!MQTT_Client_IsConnected())
		{
			BSP_LED_Set(LED2, LED_OFF);

			if(MQTT_Client_Connect() == 0)
			{
				//Connect内部不发上线通知,正常重连又不触发LWT
				MQTT_Client_PublishStatus(1);
				last_publish = 0;
				//连上即重置退避,下次失败从2s重新起算
				retry_ms = 2000;
			}
			else
			{
				//指数退避:公共Broker上快速重连会被判滥用,回CONNACK 0x8A封禁
				vTaskDelay(pdMS_TO_TICKS(retry_ms));
				if(retry_ms < 30000) retry_ms *= 2;
				if(retry_ms > 30000) retry_ms = 30000;
				continue;
			}
		}

		//已连接:收包(含下行命令回调),失败即断开重连
		if(MQTT_Client_Loop() != 0)
		{
			MQTT_Client_Disconnect();
			BSP_LED_Set(LED2, LED_OFF);
			bsp_log("MQTT disconnected, reconnect...\r\n");
			continue;
		}

		//排空显示任务入队的命令回执,逐条发布到result主题
		//断线期间的回执留在队列里,重连后随本循环补发
		while(xQueueReceive(g_result_queue, &r, 0) == pdTRUE)
		{
			if(MQTT_Client_PublishResult(r.mode, r.ok) == 0) bsp_log("Result published\r\n");
			else bsp_log("Result publish failed\r\n");
		}

		//到上报周期:取快照发布遥测
		if((HAL_GetTick() - last_publish) >= TELEMETRY_PUBLISH_MS)
		{
			last_publish = HAL_GetTick();
			if(App_GetTelemetry(&snap) == 0)
			{
				if(MQTT_Client_PublishTelemetry(&snap) == 0) bsp_log("Telemetry published\r\n");
				else bsp_log("Telemetry publish failed\r\n");
			}
		}

		vTaskDelay(pdMS_TO_TICKS(10)); //让出CPU,控制收包频率
	}
}

//显示任务:把温湿度写到六位数显表F0-00,支持待机/固定温度/固定湿度/自动轮显
static void vTaskDisplay(void *pvParameters)
{
	//本函数永不返回,故这些局部变量跨轮次保留在栈上,不能写static
	uint8_t mode;                          //本轮读到的模式
	uint8_t new_cmd;                       //本轮是否为新命令
	uint8_t last_mode = DISP_MODE_OFF;     //上一轮模式
	uint8_t write_ok;                      //写从机是否成功
	uint8_t phase = 0U;                    //0=显温度 1=显湿度
	uint32_t phase_t = 0U;                 //当前phase的起始tick
	int16_t last_temp = 0, last_humi = 0;  //传感器失效时沿用旧值
	gateway_telemetry_t t;
	int16_t v;
	const char * tag;

	while(1)
	{
		mode = g_disp_mode;             //单字节原子读
		new_cmd = (mode != last_mode);  //与上轮不同=新命令
		last_mode = mode;

		//取快照:读成功且数据有效才更新沿用值,否则保留旧值使显示不跳变
		if(App_GetTelemetry(&t) == 0 && t.sensor_valid != 0U)
		{
			last_temp = t.temp;
			last_humi = t.humi;
		}

		//相位决策:AUTO每SEG_ROTATE_MS翻转一次;1/2固定相位;OFF不动
		if(mode == DISP_MODE_AUTO)
		{
			if((HAL_GetTick() - phase_t) >= SEG_ROTATE_MS)
			{
				phase ^= 1U;
				phase_t = HAL_GetTick();
			}
		}
		else if(mode == DISP_MODE_TEMP) phase = 0U;
		else if(mode == DISP_MODE_HUMI) phase = 1U;

		//取本轮要写的值:OFF写0(屏显0.0),否则按相位取温/湿度
		if(mode == DISP_MODE_OFF)
		{
			v = 0;
			tag = "OFF";
		}
		else if(phase == 0U)
		{
			v = last_temp;
			tag = "TEMP";
		}
		else
		{
			v = last_humi;
			tag = "HUMI";
		}

		//写F0-00:写成功即从机在线的判据
		write_ok = (MB_WriteSingleRegister(SEG_SLAVE_ADDR, SEG_REG_RAW0, (uint16_t)v) == 0) ? 1U : 0U;

		if(new_cmd)
		{
			//新命令执行完入队回执,由MQTT任务发布到result主题
			disp_result_t r;
			r.mode = mode;
			r.ok = write_ok;
			if(xQueueSend(g_result_queue, &r, 0) != pdTRUE) bsp_log("SEG result queue full\r\n");

			bsp_log("SEG cmd mode=%u %s %d -> %s\r\n", (unsigned)mode, tag, (int)v, write_ok ? "OK" : "FAIL");
		}
		else bsp_log("SEG POLL %s %d -> %s\r\n", tag, (int)v, write_ok ? "OK" : "FAIL");

		vTaskDelay(pdMS_TO_TICKS(SEG_POLL_MS));
	}
}
