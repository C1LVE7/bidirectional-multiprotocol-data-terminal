/*
mqtt_client.c——MQTT客户端封装(基于coreMQTT)

架构:
coreMQTT只负责MQTT协议编解码,真正的网络收发交给本文件下面的
transport_send/transport_recv,它们再往下调BSP_WIFI_Send/Recv
底层是ESP8266透传(串口),对coreMQTT来说就是"能收发字节的管道"
时间基准用HAL_GetTick()

四个方向:
上行 = 发布遥测 -> MQTT_TOPIC_TELEMETRY
下行 = 订阅命令 -> MQTT_TOPIC_COMMAND 收到后经回调交给APP层
回执 = 命令结果 -> MQTT_TOPIC_RESULT
遗嘱 = 掉线广播 -> MQTT_TOPIC_STATUS 发 offline(连接时随CONNECT带上)

MQTT 顺序
Init      :MQTT_Init交齐传输接口/时间基准/事件回调/收包缓冲
Connect   :TcpConnect->重跑Init->conn_info+will_info(遗嘱LWT)->Connect等CONNACK->
           Subscribe->等SUBACK(g_subscribed,2s,每10ms让出CPU)->LED2亮;任一步失败逐级收尾
Loop      :ProcessLoop;三态(Success/NeedMoreBytes/NoDataAvailable)=正常,其余判需重连
Publish   :拼JSON->填pub(属性长=0)->MQTT_Publish
Disconnect:先补发retained "offline"(正常断开不触发LWT)->MQTT_Disconnect->BSP_WIFI_Close
*/
#include "mqtt_client.h"
#include "bsp_wifi.h"
#include "bsp_uart.h"
#include "bsp_led.h"
#include "core_mqtt.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

//coreMQTT的全部运行状态
static MQTTContext_t mqtt_ctx;
//coreMQTT只记下这个指针,从串口收到的MQTT字节流攒在这里再解析
static uint8_t mqtt_netbuf[1024];
//指明mqtt_netbuf在哪,以及多大
static MQTTFixedBuffer_t mqtt_fixbuf = {mqtt_netbuf, sizeof(mqtt_netbuf)};
//下行命令回调:由APP层注册,收到命令时反过来调用它
static mqtt_cmd_cb_t g_cmd_cb;
//订阅确认标志:收到SUBACK且返回码为成功时才置1
static volatile uint8_t g_subscribed;

/*
透传发送:签名对齐coreMQTT的TransportSend_t
ctx即为transport的pNetworkContext值为NULL
*/
static int32_t transport_send(NetworkContext_t * ctx, const void * buf, size_t len)
{
	//BSP层长度是uint16_t,超了只能截断
	if(len > 0xFFFFU) len = 0xFFFFU;
	return BSP_WIFI_Send((const uint8_t *)buf, (uint16_t)len);
}

//透传接收:签名对齐coreMQTT的TransportRecv_t
static int32_t transport_recv(NetworkContext_t * ctx, void * buf, size_t len)
{
	if(len > 0xFFFFU) len = 0xFFFFU;
	return BSP_WIFI_Recv((uint8_t *)buf, (uint16_t)len);
}

//coreMQTT时间基准,返回毫秒
static uint32_t mqtt_get_time_ms(void)
{
	return HAL_GetTick();
}

/*
coreMQTT的传输接口:把"怎么发""怎么收"两个函数地址打包交给库
四个成员按声明顺序对位赋值,顺序写错编译不报错,但运行会跑飞:
recv=收字节  send=发字节
writev=一次发多段数据(可选;不用必须显式写NULL,否则是随机值)
pNetworkContext=留给你自己的空地,库每次调用原样传回,它自己从不读;单链路填NULL
本结构在MQTT_Init时交给库,库内部会整份拷贝保存
*/
static TransportInterface_t transport =
{
	transport_recv,	//recv
	transport_send,	//send
	NULL,			//writev
	NULL			//pNetworkContext
};

//判断是否为JSON,忽略的空白字符,其余返回0
static int json_ws(char c)
{
	if(c == ' ' || c == '\t' || c == '\r' || c == '\n') return 1;
	return 0;
}

/*
在JSON负载里定位"display"字段并取出它的值
成功返回0~3,找不到或形式不合法返回-1
接受:键值对顺序任意,键值前后允许JSON空白,允许无关字段
拒绝:伪字段/重复display/值非单字符0~3/值后跟乱字符/首逗号/多逗号/尾逗号/缺逗号/右括号后有非空白内容/嵌套对象或数组
*/
static int8_t json_find_display(const char * payload, size_t len)
{
	size_t i = 0;
	size_t ks;
	int is_display;
	int8_t v = -1;

	//跳过前导空白,负载必须以{开头
	while(i < len && json_ws(payload[i])) i++;
	if(i >= len || payload[i] != '{') return -1;
	i++;

	//跳到第一个成员前;空对象{}里没有display
	while(i < len && json_ws(payload[i])) i++;
	if(i >= len) return -1;

	while(1)
	{
		//此处必须是键的开始引号(首逗号在这里被拦下)
		if(payload[i] != '"') return -1;

		//读键:先记起点,再扫到闭合引号
		ks = ++i;
		while(i < len && payload[i] != '"') i++;
		if(i >= len) return -1;
		//此刻i正停在键的闭合引号上,i-ks才是真正的键长
		is_display = ((i - ks) == 7) && (strncmp(payload + ks, "display", 7) == 0);
		i++;

		//键后必须紧跟冒号
		while(i < len && json_ws(payload[i])) i++;
		if(i >= len || payload[i] != ':') return -1;
		i++;
		while(i < len && json_ws(payload[i])) i++;
		if(i >= len) return -1;

		//消费本键值对的值:display只接受单字符0~3,其余整体跳过
		if(is_display)
		{
			if(v >= 0) return -1; //重复display字段
			if(payload[i] < '0' || payload[i] > '3') return -1;
			v = (int8_t)(payload[i] - '0');
			i++;
		}
		else if(payload[i] == '"')
		{
			i++;
			while(i < len && payload[i] != '"') i++;
			if(i >= len) return -1;
			i++;
		}
		else while(i < len && payload[i] != ',' && payload[i] != '}') i++;

		//统一收口:本键值对之后必须紧跟逗号或右括号
		while(i < len && json_ws(payload[i])) i++;
		if(i >= len) return -1;
		if(payload[i] == '}')
		{
			i++;
			break;
		}
		if(payload[i] != ',') return -1;
		i++;

		//逗号后必须还有下一个键(拒尾逗号)
		while(i < len && json_ws(payload[i])) i++;
		if(i >= len) return -1;
		if(payload[i] == '}') return -1;
	}

	//右括号之后只允许空白(拒尾随内容)
	while(i < len && json_ws(payload[i])) i++;
	if(i != len) return -1;

	return v;
}

//解析下行命令,成功时回调应用层
static void parse_command(const char * payload, size_t len)
{
	int8_t display_mode = json_find_display(payload, len);

	if(display_mode >= 0 && g_cmd_cb != NULL) g_cmd_cb((uint8_t)display_mode);
}

/*
coreMQTT事件回调:CONNACK/SUBACK/PUBLISH都从这里进来
收到SUBACK且返回码成功时才置订阅标志

ctx 库上下文,本工程单链路用不上
pkt 报文头:type=报文类型(0x20=CONNACK 0x90=SUBACK 0x30=PUBLISH)
deser 已解好的内容:packetIdentifier=报文ID
pPublishInfo=PUBLISH时才有值(主题/载荷),其他报文为NULL
deserializationResult=解包结果 pReasonCode=原因码
rc 结果原因码,本工程未用
send_props 库提供的"回发属性"缓冲区,本工程未用
get_props 本包携带的属性,本工程未用
*/
static bool mqtt_event_cb(MQTTContext_t * ctx, MQTTPacketInfo_t * pkt, MQTTDeserializedInfo_t * deser,
                          MQTTSuccessFailReasonCode_t * rc, MQTTPropBuilder_t * send_props, MQTTPropBuilder_t * get_props)
{
	//订阅确认:返回码0x00~0x02才是成功;0x80以上是拒绝
	if(pkt->type == MQTT_PACKET_TYPE_SUBACK)
	{
		uint8_t * codes = NULL;
		size_t num = 0;

		if(MQTT_GetSubAckStatusCodes(pkt, &codes, &num) == MQTTSuccess && num == 1
		   && codes[0] <= MQTTSubAckSuccessQos2) g_subscribed = 1;
		else bsp_log("MQTT: subscription rejected\r\n");
	}
	//下行命令:PUBLISH固定头低4位带DUP/QoS/RETAIN标志,判类型必须用掩码
	else if((pkt->type & 0xF0U) == MQTT_PACKET_TYPE_PUBLISH)
	{
		MQTTPublishInfo_t * pub = deser->pPublishInfo;

		//只处理命令主题的消息
		if(pub != NULL && pub->topicNameLength == strlen(MQTT_TOPIC_COMMAND) &&
		strncmp(pub->pTopicName, MQTT_TOPIC_COMMAND, pub->topicNameLength) == 0)
		parse_command((const char *)pub->pPayload, pub->payloadLength);
	}

	return true;
}

//初始化MQTT上下文:把传输接口,时间基准,回调,收包缓冲一次性交给库
void MQTT_Client_Init(void)
{
	MQTTStatus_t st;

	st = MQTT_Init(&mqtt_ctx, &transport, mqtt_get_time_ms, mqtt_event_cb, &mqtt_fixbuf);
	if(st != MQTTSuccess) bsp_log("FATAL: MQTT_Init failed, st=%d\r\n", (int)st);
}

/*
建立TCP连接并完成MQTT CONNECT(含遗嘱LWT) 订阅下行命令主题
0=成功  -2=TCP失败  -3=CONNECT失败  -4=SUBSCRIBE发送失败  -5=未收到SUBACK
*/
int8_t MQTT_Client_Connect(void)
{
	MQTTConnectInfo_t conn_info = { 0 };
	MQTTPublishInfo_t will_info = { MQTTQoS0 };
	MQTTSubscribeInfo_t sub_info = { MQTTQoS0 };
	MQTTStatus_t st;
	bool session_present = false;
	uint32_t t;
	g_subscribed = 0;

	//TCP:透传方案直连IP,不经域名解析
	if(BSP_WIFI_TcpConnect(MQTT_BROKER_HOST, MQTT_BROKER_PORT) != 0)
	{
		bsp_log("MQTT: TCP connect failed\r\n");
		return -2;
	}

	//重连时上下文可能残留上一轮状态,重新初始化(库内部会memset整个上下文)
	MQTT_Init(&mqtt_ctx, &transport, mqtt_get_time_ms, mqtt_event_cb, &mqtt_fixbuf);

	/*
	CONNECT参数:
	cleanSession true=丢弃旧会话,每轮从零开始,不补发离线消息
	keepAliveSeconds 心跳周期(秒);无数据超时后库自动发PINGREQ
	pClientIdentifier 客户端ID(指针);库不拷贝内容只记地址,须长期有效
	clientIdentifierLength ID长度;须自己算,不含结尾'\0'
	pUserName/userNameLength 账号;空串时传NULL+长度0=匿名连接
	pPassword/passwordLength 密码;同上,空串则NULL+长度0
	注:指针与长度必须成对,长度算错broker直接拒连
	*/
	conn_info.cleanSession = true;
	conn_info.keepAliveSeconds = MQTT_KEEPALIVE_S;
	conn_info.pClientIdentifier = MQTT_CLIENT_ID;
	conn_info.clientIdentifierLength = strlen(MQTT_CLIENT_ID);
	conn_info.pUserName = (MQTT_USERNAME[0] != '\0') ? MQTT_USERNAME : NULL;
	conn_info.userNameLength = (MQTT_USERNAME[0] != '\0') ? strlen(MQTT_USERNAME) : 0;
	conn_info.pPassword = (MQTT_PASSWORD[0] != '\0') ? MQTT_PASSWORD : NULL;
	conn_info.passwordLength = (MQTT_PASSWORD[0] != '\0') ? strlen(MQTT_PASSWORD) : 0;

	/*
	遗嘱(LWT)参数:连接时交给broker保管,本机异常掉线由broker代为发布
	qos=MQTTQoS0 至多一次;掉线通知丢一两条无妨
	retain=true 保留消息;让offline覆盖上次的online,新订阅者不会读到过期状态
	dup=false 非重传标志;首发必须false
	pTopicName/topicNameLength 主题(指针+长度);遗嘱主题不允许带通配符
	pPayload/payloadLength 载荷;这里发字符串"offline"
	propertyLength=0 属性块长度为0=无属性;MQTT5.0每个报文的载荷都带这个长度字段
	*/
	will_info.qos = MQTTQoS0;
	will_info.retain = true;
	will_info.dup = false;
	will_info.pTopicName = MQTT_TOPIC_STATUS;
	will_info.topicNameLength = strlen(MQTT_TOPIC_STATUS);
	will_info.pPayload = "offline";
	will_info.payloadLength = strlen("offline");
	will_info.propertyLength = 0;
	
	//订阅下行命令主题:订阅只是发出请求,必须等到SUBACK才算真的订阅成功
	sub_info.qos = MQTTQoS0;
	sub_info.pTopicFilter = MQTT_TOPIC_COMMAND;
	sub_info.topicFilterLength = strlen(MQTT_TOPIC_COMMAND);

	/*
	发起连接,最多等CONNACK 5s
	参数信息 上下文 连接参数 遗嘱 超时 会话标志输出 属性 遗嘱属性
	*/
	if(MQTT_Connect(&mqtt_ctx, &conn_info, &will_info, 5000, &session_present, NULL, NULL) != MQTTSuccess)
	{
		bsp_log("MQTT: CONNECT failed\r\n");
		BSP_WIFI_Close();
		return -3;
	}

	//参数 上下文 订阅信息数组首地址 订阅项个数1 包ID 属性
	if(MQTT_Subscribe(&mqtt_ctx, &sub_info, 1, MQTT_GetPacketId(&mqtt_ctx), NULL) != MQTTSuccess)
	{
		bsp_log("MQTT: SUBSCRIBE failed\r\n");
		MQTT_Disconnect(&mqtt_ctx, NULL, NULL);
		BSP_WIFI_Close();
		return -4;
	}

	/*
	等SUBACK,最多2s
	每10ms让出CPU:本函数跑在vTaskMqtt里,忙等2s会把低优先级任务饿死
	MQTTNoDataAvailable/MQTTNeedMoreBytes都表示"报文还没收全",属正常,继续等
	*/
	t = HAL_GetTick();
	while(!g_subscribed && (HAL_GetTick() - t) < 2000U)
	{
		st = MQTT_ProcessLoop(&mqtt_ctx);
		if(st != MQTTSuccess && st != MQTTNeedMoreBytes && st != MQTTNoDataAvailable) break;
		vTaskDelay(pdMS_TO_TICKS(10));
	}

	if(!g_subscribed)
	{
		bsp_log("MQTT: no SUBACK\r\n");
		MQTT_Disconnect(&mqtt_ctx, NULL, NULL);
		BSP_WIFI_Close();
		return -5;
	}
	
	//网络灯:连上常亮
	BSP_LED_Set(LED2, LED_ON);

	return 0;
}

/*
周期收包:0=正常,负数=需要重连
MQTT_ProcessLoop返回:
MQTTSuccess 这轮处理完了
MQTTNeedMoreBytes 报文没收全
MQTTNoDataAvailable 暂时没数据
*/
int8_t MQTT_Client_Loop(void)
{
	MQTTStatus_t st = MQTT_ProcessLoop(&mqtt_ctx);

	if(st == MQTTSuccess || st == MQTTNeedMoreBytes || st == MQTTNoDataAvailable) return 0;
	return -1;
}

/*
查询是否已连接,1=已连接
mqtt_ctx.connectStatus的值
MQTTNotConnected 未连接
MQTTConnected 已连接
MQTTDisconnectPending 需断开(传输层已出错,等你去断)
*/
uint8_t MQTT_Client_IsConnected(void)
{
	return (mqtt_ctx.connectStatus == MQTTConnected) ? 1 : 0;
}

//注册下行命令回调
void MQTT_Client_SetCmdCallback(mqtt_cmd_cb_t cb)
{
	g_cmd_cb = cb;
}

//显示模式 -> 字符串(0=off 1=temp 2=humi 3=auto)
static const char * disp_str(uint8_t mode)
{
	if(mode == 1U) return "temp";
	if(mode == 2U) return "humi";
	if(mode == 3U) return "auto";
	return "off";
}

/*
发布命令回执:{"display":"off|temp|humi|auto","result":"ok|fail"}
0=成功 -1=拼串失败 -2=发布失败
*/
int8_t MQTT_Client_PublishResult(uint8_t display_mode, uint8_t ok)
{
	char payload[48];
	MQTTPublishInfo_t pub = { MQTTQoS0 };
	int n;

	n = snprintf(payload, sizeof(payload), "{\"display\":\"%s\",\"result\":\"%s\"}",
	             disp_str(display_mode), (ok != 0) ? "ok" : "fail");
	if(n < 0 || (size_t)n >= sizeof(payload)) return -1;

	pub.qos = MQTTQoS0;
	pub.retain = false;
	pub.dup = false;
	pub.pTopicName = MQTT_TOPIC_RESULT;
	pub.topicNameLength = strlen(MQTT_TOPIC_RESULT);
	pub.pPayload = payload;
	pub.payloadLength = (size_t)n;
	pub.propertyLength = 0;

	if(MQTT_Publish(&mqtt_ctx, &pub, MQTT_GetPacketId(&mqtt_ctx), NULL) != MQTTSuccess) return -2;
	return 0;
}

/*
发布遥测JSON到MQTT_TOPIC_TELEMETRY
温湿度按0.1分辨率拆整数位与小数位;负温先取绝对值,符号单独拼前缀
(直接t/10,t%10遇负温会把-0.5℃打成0.-5)
t不能为NULL;0=成功 -1=拼串失败 -2=发布失败
*/
int8_t MQTT_Client_PublishTelemetry(const gateway_telemetry_t * t)
{
	char payload[64];
	MQTTPublishInfo_t pub = { MQTTQoS0 };
	const char * sign = "";
	int it, ft; //温度整数位/小数位
	int ih, fh; //湿度整数位/小数位
	int n;

	if(t == NULL) return -1;

	//负温取绝对值再拆位,符号单独交给%s拼前缀
	if(t->temp < 0)
	{
		sign = "-";
		it = (-t->temp) / 10;
		ft = (-t->temp) % 10;
	}
	else
	{
		it = t->temp / 10;
		ft = t->temp % 10;
	}

	//湿度恒非负,直接拆
	ih = t->humi / 10;
	fh = t->humi % 10;

	n = snprintf(payload, sizeof(payload), "{\"temp\":%s%d.%d,\"humi\":%d.%d,\"sensor_valid\":%d}",
	             sign, it, ft, ih, fh, t->sensor_valid ? 1 : 0);
	if(n < 0 || (size_t)n >= sizeof(payload)) return -1;

	pub.qos = MQTTQoS0;
	pub.retain = false;
	pub.dup = false;
	pub.pTopicName = MQTT_TOPIC_TELEMETRY;
	pub.topicNameLength = strlen(MQTT_TOPIC_TELEMETRY);
	pub.pPayload = payload;
	pub.payloadLength = (size_t)n;
	pub.propertyLength = 0;

	if(MQTT_Publish(&mqtt_ctx, &pub, MQTT_GetPacketId(&mqtt_ctx), NULL) != MQTTSuccess) return -2;
	return 0;
}


/*
发布上线/离线状态到MQTT_TOPIC_STATUS
retain=true:broker保留最后一条,晚订阅的客户端一连上就能看到当前状态
online非0发online,0发offline;0=成功 -1=发布失败
*/
int8_t MQTT_Client_PublishStatus(uint8_t online)
{
	const char * payload = (online != 0) ? "online" : "offline";
	MQTTPublishInfo_t pub = { MQTTQoS0 };

	pub.qos = MQTTQoS0;
	pub.retain = true;
	pub.dup = false;
	pub.pTopicName = MQTT_TOPIC_STATUS;
	pub.topicNameLength = strlen(MQTT_TOPIC_STATUS);
	pub.pPayload = payload;
	pub.payloadLength = strlen(payload);
	pub.propertyLength = 0;

	//发布接口
	if(MQTT_Publish(&mqtt_ctx, &pub, MQTT_GetPacketId(&mqtt_ctx), NULL) != MQTTSuccess) return -1;
	return 0;
}

/*
断开MQTT与TCP连接
正常断开不触发遗嘱(LWT),故先主动补发retained offline,再发DISCONNECT;
否则状态主题会残留上一次的online
BSP_WIFI_Close含"+++退透传(前后各1.1s)+CIPMODE=0+CIPCLOSE",约2.2~3.4s耗时,
故只在链路确实建立时才走关闭流程,避免空跑一遍拖慢重连节拍
*/
void MQTT_Client_Disconnect(void)
{
	if(mqtt_ctx.connectStatus == MQTTConnected)
	{
		MQTT_Client_PublishStatus(0);
		MQTT_Disconnect(&mqtt_ctx, NULL, NULL);
	}

	if(BSP_WIFI_IsLinked()) BSP_WIFI_Close();
}
