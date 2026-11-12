/*
ESP8266:WiFi透传驱动(USART3 + AT指令)

1.模块跑出厂AT固件,通过USART3收发ASCII指令
2.连接流程:AT同步->ATE0关回显->AT+CWMODE=1设station->
AT+CWJAP连WiFi->AT+CIPSTART连Broker->AT+CIPMODE=1+AT+CIPSEND进透传
3.透传态下串口成为"裸字节管道",不再解析任何AT应答
4.发送走阻塞;接收走RXNE中断+环形缓冲——USART无硬件FIFO,轮询会因来不及读DR而ORE丢字节

函数开启顺序:
1.Init :复位->清启动banner->开RXNE中断->AT探活x3->ATE0关回显->CWMODE=1设客户端
2.Tcp  :ensure_joined(未联网则CWJAP,失败硬复位自愈一次)->CIPMUX=0单连接->
        CIPSTART建TCP(等"CONNECT\r\n"并翻案DISCONNECT)->CIPMODE=1开透传模式->
        CIPSEND进透传(等">")->延时20ms再flush->g_tcp_open=1标志链路已通
3.收发关闭:Send(需g_tcp_open)/Recv(静默30ms判帧尾);Close=+++(前后各1.1s)
        ->AT探针失败即硬复位自愈,成功则CIPMODE=0->CIPCLOSE

工程所用AT指令集
+++  (不是AT指令)
作用:退出透传;不能带\r\n,前后各需1.1s静默
返回:成功OK,否则ERROR或无应答
AT
作用:探活,确认串口通且模块在命令模式
返回:OK
ATE0
作用:关回显,避免指令回显混进应答干扰匹配
返回:OK
AT+CWMODE=1
1 = Station(客户端):模块去连别人的路由器,由路由器DHCP分配IP
2 = SoftAP(软热点):模块自己当热点,供手机/电脑连接
3 = SoftAP+Station:同时做热点和客户端
作用:设WiFi模式为Station(客户端)
返回:OK
AT+CIFSR
作用:查本机IP;只读本地信息,不产生网络流量
返回:已联网:+CIFSR:STAIP,"192.168.x.x" -> +CIFSR:STAMAC,"xx:..." -> OK
     未联网:+CIFSR:STAIP,"0.0.0.0" -> +CIFSR:STAMAC -> OK
AT+CWJAP="ssid","pwd"
作用:连路由器;含DHCP,全流程最慢
返回:WIFI DISCONNECT -> WIFI CONNECTED -> WIFI GOT IP -> OK
     前三行是模块主动推的提示信息,OK才是正式响应
     失败 +CWJAP_CUR:<n> + FAIL (1超时 2密码错 3找不到AP 4连接失败)
AT+CIPMUX=0
作用:设单连接模式;透传模式的硬前提
返回:OK
AT+CIPSTART="TCP","host",port
作用:建立TCP连接
返回:CONNECT -> OK (CONNECT是提示信息,OK才是正式响应)
     失败回ERROR
AT+CIPMODE=1
作用:开透传模式
返回:OK
AT+CIPSEND
作用:进透传(仅CIPMODE=1下);此后串口变裸字节管道
返回:先回OK,再回> (OK在前,\r\n>在后;">"后面没有空格,实测三轮一致)
AT+CIPCLOSE
作用:关TCP连接
返回:CLOSED -> OK (CLOSED是提示信息)
AT+CIPMODE=0
作用:退透传配置,防下次CIPSEND状态残留
返回:OK
通用1:收到ERROR不等于指令执行失败,它只表示两种意思:
    1.模块不认识这条指令(拼错,或固件不支持)
    2.当前状态不允许执行(如还没连上WiFi就发AT+CIPSTART)
通用2:CWJAP这类指令失败回的是FAIL(形如 +CWJAP_CUR:2 + FAIL),不是ERROR;
      判成败要认FAIL,不能只搜ERROR (1超时 2密码错 3找不到AP 4连接失败)
通用3:等应答不要一律写"OK":各指令要匹配的关键字不同,代码就是按下面这么等的:
      AT+CWJAP     等 WIFI GOT IP  (共4行,OK在最后一行,故取更早出现的它)
      AT+CIPSTART  等 CONNECT      (共2行,OK在最后一行,故取更早出现的它)
      AT+CIPSEND   等 >            (进透传的提示符;应答为"先OK,再\r\n>",">"后无空格)
*/
#include "bsp_wifi.h"
#include "bsp_uart.h"   
#include "app_config.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"      
#include <stdio.h>
#include <string.h>

//USART3无硬件FIFO,轮询接收会丢字节;改中断收:每字节到达即写入环形缓冲
//512字节(实际可存511):覆盖coreMQTT收包缓冲(1024)以下的常见报文
#define WIFI_RX_BUF_SIZE    512

//透传链路是否已建立:1=已进入透传模式
static volatile uint8_t g_tcp_open;
static volatile uint8_t  s_rx_buf[WIFI_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0;   //写索引,只在中断里改
static volatile uint16_t s_rx_tail = 0;   //读索引,只在任务里改

//供USART3中断调用:把1字节写进环形缓冲,满则丢弃
void BSP_WIFI_RxPush(uint8_t b)
{
	//防止后面判断不通过导致s_rx_head不好改回来
	uint16_t next = (uint16_t)(s_rx_head + 1U);
	//判断是否越界
	if(next >= WIFI_RX_BUF_SIZE) next = 0;
	//留一个字节,防止head=tail导致的判空
	if(next != s_rx_tail)
	{
		s_rx_buf[s_rx_head] = b;
		s_rx_head = next;
	}
}

//任务侧:非阻塞读1字节,返回1=读到,0=无数据
static int wifi_rx_byte(uint8_t * b)
{
	//head=tail判空
	if(s_rx_head == s_rx_tail) return 0;
	//写入b数据
	*b = s_rx_buf[s_rx_tail];
	//移到下一位
	s_rx_tail = (uint16_t)(s_rx_tail + 1U);
	//判断是否越界
	if(s_rx_tail >= WIFI_RX_BUF_SIZE) s_rx_tail = 0;
	//读到数据返回1
	return 1;
}

//清空接收缓冲:发AT指令前调用,丢掉残留
static void wifi_flush_rx(void)
{
	uint8_t b;
	while(wifi_rx_byte(&b));
}

//阻塞发送
static HAL_StatusTypeDef wifi_uart_send(const uint8_t * data, uint16_t len)
{
	return HAL_UART_Transmit(&huart3, data, len, 2000);
}

static void wifi_send_str(const char * s)
{
	wifi_uart_send((const uint8_t *)s, (uint16_t)strlen(s));
}

//发一条AT指令,自动补\r\n
static void wifi_send_cmd(const char * cmd)
{
	wifi_send_str(cmd);
	wifi_send_str("\r\n");
}

//调度器已启动时用vTaskDelay让出CPU,否则用HAL_Delay
static void wifi_delay_ms(uint32_t ms)
{
	if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) vTaskDelay(pdMS_TO_TICKS(ms));
	else HAL_Delay(ms);
}

//发送指令后等待应答:把收到的字节追加进out;返回0=命中expect,-1=收到ERROR/FAIL,-2=超时
static int8_t wifi_wait_expect(char * out, uint16_t out_cap,
                               const char * expect, uint32_t timeout_ms)
{
	uint32_t t = HAL_GetTick();
	uint16_t n = (uint16_t)strlen(out);

	//上一次调用可能已把应答攒在out里:主循环只在"有新字节"时才匹配,故入口先对已有内容查一遍,否则漏检
	if(strstr(out, "ERROR") != NULL || strstr(out, "FAIL") != NULL) return -1;
	if(strstr(out, expect) != NULL) return 0;

	//主循环:有字节就追加进out;匹配只在"有新字节"时触发
	while((HAL_GetTick() - t) < timeout_ms)
	{
		uint8_t b;
		if(wifi_rx_byte(&b))
		{
			/*
			缓冲将满:丢掉前一半,保留后一半,给新字节腾空间
			注意:为什么不用memcpy是因为搬运空间有重叠,memcpy不会判断
			而memmove会自动判断如何搬运
			*/
			if(n >= out_cap - 1)
			{
				memmove(out, out + n/2, n - n/2 + 1);
				n = (uint16_t)(n - n/2);
			}

			out[n++] = (char)b;
			out[n] = '\0';
			
			//先认错再认对:同时含ERROR和OK时应判失败
			if(strstr(out, "ERROR") != NULL || strstr(out, "FAIL") != NULL) return -1;
			if(strstr(out, expect) != NULL) return 0;
		}
		else wifi_delay_ms(5);	//没数据,等待5ms
	}
	return -2;	//超时
}

//发一条AT指令并等待期望应答
static int8_t wifi_at(const char * cmd, const char * expect, uint32_t timeout_ms)
{
	char buf[128] = {0};

	wifi_flush_rx();	//清残留
	wifi_send_cmd(cmd);	//发指令(自动补\r\n)
	return wifi_wait_expect(buf, sizeof(buf), expect, timeout_ms); //等应答
}

//查询是否已连上WiFi(已取到IP):1=已联网,0=未联网
static uint8_t wifi_has_ip(void)
{
	char buf[128] = {0};
	int8_t r;

	wifi_flush_rx();
	wifi_send_cmd("AT+CIFSR");	//AT+CIFSR=查询本机IP地址
	r = wifi_wait_expect(buf, sizeof(buf), "OK", 3000);
	
	//未联网时CIFSR回"0.0.0.0",出现即视为未联网
	if(r == 0 && strstr(buf, "0.0.0.0") == NULL) return 1;
	
	return 0;
}

//连WiFi:设station模式+AT+CWJAP 返回0成功,-1设模式失败,-2连不上
static int8_t wifi_join_ap(void)
{
	char cmd[128];
	char buf[128] = {0};
	int8_t r;

	//先确保是station模式:幂等操作,防止模块上电后模式不对
	if(wifi_at("AT+CWMODE=1", "OK", 1000) != 0) return -1;

	//拼指令:SSID和密码从app_config.h的宏填进去
	snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASSWORD);

	/*
	连WiFi含DHCP,慢,给足15s;等"WIFI GOT IP"
	实测:CWJAP完整应答为 WIFI DISCONNECT/WIFI CONNECTED/WIFI GOT IP/OK,
	前三行是提示信息,OK才是正式响应;WIFI GOT IP拿到IP即返回,出现更早,故取它
	*/
	wifi_flush_rx();
	wifi_send_cmd(cmd);
	r = wifi_wait_expect(buf, sizeof(buf), "WIFI GOT IP", 15000);

	//没命中,SSID或密码有问题
	if(r != 0) return -2;

	return 0;
}

//确保已连WiFi:已联网直接返回,否则重新join
static int8_t wifi_ensure_joined(void)
{
	if(wifi_has_ip()) return 0;
	return wifi_join_ap();
}

/*
初始化:硬复位模块->开接收中断->AT同步->关回显->设station
banner为上电/复位时的启动输出(模块"主动吐出来"不是对你指令的应答的那些字节)
返回设置:0->没问题 -1->未连上ESP8266 -2->关闭回显失败 -3->设置Station模式失败
*/
int8_t BSP_WIFI_Init(void)
{
	int i;
	g_tcp_open = 0;

	//硬复位,把模块拽回确定状态
	HAL_GPIO_WritePin(WIFI_RST_PORT, WIFI_RST_PIN, GPIO_PIN_RESET);	//拉低=复位
	wifi_delay_ms(200);	//手册下限100us,取200ms留足余量
	HAL_GPIO_WritePin(WIFI_RST_PORT, WIFI_RST_PIN, GPIO_PIN_SET);	//拉高=运行
	wifi_delay_ms(500);	//等模块固件启动到AT就绪
	wifi_flush_rx();	//丢弃启动banner

	//开串口3接收中断,字节一到就进ISR写环形缓冲
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
	HAL_NVIC_SetPriority(USART3_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);

	//AT同步探活:模块刚复位,串口可能有banner残留,重试3次再判死
	for(i = 0; i < 3; i++) if(wifi_at("AT", "OK", 1000) == 0) break;
	//循环走满=3次全无应答
	if(i == 3)
	{
		bsp_log("WiFi: no response from ESP8266, check UART wiring\r\n");
		return -1;
	}

	//关回显,避免指令回显混进应答干扰匹配
	if(wifi_at("ATE0", "OK", 1000) != 0) return -2;

	//设Station模式(客户端)
	if(wifi_at("AT+CWMODE=1", "OK", 1000) != 0) return -3;
	
	return 0;
}

/*
连WiFi+连Broker+进透传,host只能传IP字符串
返回设置:0->成功 -1->未联网,且硬复位自愈也没救回来 -2->AT+CIPMUX=0失败
-3->AT+CIPSTART失败 -4&-5->进入透传模式失败
host:IP或域名 port:端口号
*/
int8_t BSP_WIFI_TcpConnect(const char * host, uint16_t port)
{
	char cmd[128];
	char buf[128] = {0};
	int8_t r;

	g_tcp_open = 0;

	/*
	确保已连WiFi
	连不上可能是模块异常(死机/串口卡死):此时AT被吞,CIFSR/join全超时,每次重连都卡在同一死点
	借Init的RST脉冲把模块拽回确定状态,再试一轮;只重试一次,避免无退避地反复复位
	*/
	if(wifi_ensure_joined() != 0)
	{
		bsp_log("WiFi: not joined, recovering (hw reset) ...\r\n");
		if(BSP_WIFI_Init() != 0 || wifi_ensure_joined() != 0) return -1;
	}

	//单连接模式,透传的硬前提
	if(wifi_at("AT+CIPMUX=0", "OK", 1000) != 0) return -2;
	
	//建TCP连接,公网Broker握手慢,给10s
	snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u", host, (unsigned)port);
	wifi_flush_rx();	//清残答,防上一轮应答混入
	wifi_send_cmd(cmd);
	//必加\r\n:只写"CONNECT"的话,应答里先出现WIFI DISCONNECT这类含该子串的提示,
	//读到第7个字符就命中,连接失败会被误判成成功
	r = wifi_wait_expect(buf, sizeof(buf), "CONNECT\r\n", 10000);
	
	//DISCONNECT\r\n自第4字符起恰好是CONNECT\r\n,会被误命中,必须翻案
	if(r == 0) if(strstr(buf, "DISCONNECT") != NULL) r = -3;
	if(r != 0)
	{
		bsp_log("WiFi: TCP connect to %s:%u failed\r\n", host, (unsigned)port);
		return -3;
	}

	//进透传模式,CIPMODE=1后CIPSEND,出现>即进入纯字节流状态
	if(wifi_at("AT+CIPMODE=1", "OK", 1000) != 0)
	{
		//CIPMODE没生效,TCP可能已建立,关掉避免残留占用
		wifi_at("AT+CIPCLOSE", "OK", 2000);
		return -4;
	}
	if(wifi_at("AT+CIPSEND", ">", 2000) != 0)
	{
		//CIPMODE=1已生效但没进透传:先恢复CIPMODE=0再关连接,防下次状态残留
		wifi_at("AT+CIPMODE=0", "OK", 1000);
		wifi_at("AT+CIPCLOSE", "OK", 2000);
		return -5;
	}

	/*
	先延时再清(顺序不能反):模块回了>才真正切到纯字节流,
	切换期冒出的杂散字节排在CONNACK前会让coreMQTT长度错位,反复重连;
	20ms@115200约230字节,不会丢掉真正的透传数据
	*/
	wifi_delay_ms(20);
	wifi_flush_rx();

	g_tcp_open = 1;
	bsp_log("WiFi: TCP transparent mode ready\r\n");
	
	return 0;
}

//透传发送:返回实际发送字节数,<0失败
int32_t BSP_WIFI_Send(const uint8_t * buf, uint16_t len)
{
	if(!g_tcp_open) return -1;	//链路还没建立
	//HAL_OK=len字节全部交给串口;超时/错误/忙一律判失败
	if(wifi_uart_send(buf, len) != HAL_OK) return -2;
	return (int32_t)len;
}

//透传接收,返回收到字节数,0=本次无数据,<0=链路未建立
int32_t BSP_WIFI_Recv(uint8_t * buf, uint16_t len)
{
	uint16_t i = 0;
	uint32_t t;

	if(!g_tcp_open) return -1;

	/*
	不能写成"无数据就return 0":coreMQTT收包是先读1字节报文类型
	再逐字节读"剩余长度",中途读到0会被它判成"剩余长度非法"
	(MQTT_REMAINING_LENGTH_INVALID)->CONNACK解析失败->反复重连;
	115200下字节是逐个到的,首字节可能还没进USART3就被读成空
	*/
	t = HAL_GetTick();
	while(i < len)
	{
		if(wifi_rx_byte(&buf[i]))
		{
			i++;
			t = HAL_GetTick();	//每收到1字节刷新静默计时
		}
		else if((HAL_GetTick() - t) >= 30U) break;	//静默30ms:视为本包已收完
		else wifi_delay_ms(1);	//让出CPU,等USART3中断写入
	}
	return i;
}

//查询透传链路是否已建立,1=已建立
uint8_t BSP_WIFI_IsLinked(void)
{
	return g_tcp_open;
}

//退出透传并关闭TCP
void BSP_WIFI_Close(void)
{
	g_tcp_open = 0;

	//"+++"不是AT指令:不能带\r\n,必须单独一包,前后各留1.1s静默
	wifi_delay_ms(1100);
	wifi_send_str("+++");
	wifi_delay_ms(1100);
	wifi_flush_rx();

	/*
	探针确认是否真回到命令模式:"+++"时序敏感,可能失败;
	一旦失败模块仍停在透传态,之后所有AT都会当数据发给Broker(无应答),
	has_ip/join/connect全超时,连接永久建不回来,只能断电;
	故探针失败就给一次硬复位(走Init),运行期自愈
	*/
	if(wifi_at("AT", "OK", 1000) != 0)
	{
		bsp_log("WiFi: exit passthrough failed, resetting module\r\n");
		(void)BSP_WIFI_Init();
		return;	//复位后TCP必已断开,不用再发CIPCLOSE
	}

	//先退透传配置,再关连接
	if(wifi_at("AT+CIPMODE=0", "OK", 1000) != 0) bsp_log("WiFi: CIPMODE=0 failed\r\n");
	if(wifi_at("AT+CIPCLOSE", "OK", 2000) != 0) bsp_log("WiFi: CIPCLOSE failed\r\n");
	bsp_log("WiFi: TCP closed\r\n");
}
