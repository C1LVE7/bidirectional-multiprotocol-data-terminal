/*
Modbus RTU主站协议(USART2+RS485)
用于轮询/控制RS485总线上的从机设备,当前场景:六位数显表

帧格式(RTU):[从机地址][功能码][数据...][CRC16低][CRC16高]

公共接口统一返回:0成功,负数失败
-1 无应答/接收超时 -2 响应地址或功能码不符 -3 CRC校验失败
-4 响应字节数与协议不符(短帧/长帧) -5 响应长度超过接收缓冲
-6 参数非法(输出指针为NULL) -7 从机地址为广播地址0
-8 数量非法(为0或超过MB_MAX_REGS_PER_READ)

CRC校验方向往哪,就按最靠近方向那位判断是否异或多项式
即往右移则用最右边bit是否为1判断,然后位移再异或
LSB-first往右 MSB-first往左

往右移多项式需要反射,即原多项式0x8005变为0xA001
展开二项式翻转即可,但CRCx的x+1项不用算进去
*/
#include "modbus_rtu_master.h"
#include "usart.h"
#include "bsp_uart.h"

//Modbus CRC16:多项式0xA001,初值0xFFFF,方向:LSB-first
static uint16_t mb_crc16(const uint8_t * buf, uint16_t len)
{
	uint16_t crc = 0xFFFF;
	uint16_t i;
	uint8_t j;
	
	for(i = 0; i < len; i++)
	{
		crc ^= buf[i];
		for(j = 0; j < 8; j++)
		{
			if(crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
			else crc >>= 1;
		}
	}
	return crc;
}

//清空接收缓冲:把USART2里残留的旧字节读掉,读到1ms超时为止
static void mb_flush_rx(void)
{
	uint8_t ch;
	while(HAL_UART_Receive(&huart2, &ch, 1, 1) == HAL_OK);
}

//读1字节:读到返回1,每轮等10ms,累计超过timeout_ms仍未读到则返回0
static uint8_t mb_read_byte_timeout(uint8_t * ch, uint32_t timeout_ms)
{
	uint32_t t = HAL_GetTick();

	while(HAL_UART_Receive(&huart2, ch, 1, 10) != HAL_OK) if((HAL_GetTick() - t) > timeout_ms) return 0;

	return 1;
}

//发送一帧:切发送方向->发数据->等末字节移出->切回接收
static void mb_send_frame(const uint8_t * frame, uint16_t len)
{
	uint32_t t;

	RS485_SetTx();
	//切换方向后先等4ms:让收发器稳定,同时满足Modbus帧间隔>=3.5字符静默
	HAL_Delay(4);

	//发送失败(串口异常)立即切回接收,避免下面等TC标志时陷入死循环
	if(HAL_UART_Transmit(&huart2, frame, len, 100) != HAL_OK)
	{
		RS485_SetRx();
		return;
	}

	/*
	HAL_UART_Transmit返回只说明最后1字节已写进发送寄存器,它可能还在移位中
	必须等TC(发送完成)再拉低DE,否则帧尾CRC高字节被截断,从机一律不应答
	*/
	t = HAL_GetTick();
	
	//带超时保护,防串口异常卡死
	while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET) if((HAL_GetTick() - t) > 10) break;
	
	RS485_SetRx();
}

/*
接收一帧响应:按功能码推断帧长后逐字节收齐,成功返回0并由len回传实际帧长
buf_size=buf容量(防越界),timeout_ms=等待首字节(从机应答)的时间

从机应答帧格式
一.FC06正常应答=原样回显请求帧(共8字节)
格式:[地址][0x06][寄存器地址2B][写入值2B][CRC低][CRC高]
实例:[01][06][00 00][00 EB][C9][85]
说明:内容与主站请求帧逐字节相同,故本工程要核对rx[2-5]==frame[2-5]
不核对则"错误回显"会被当成写成功(静默假成功)

二.FC03正常应答(变长,长度由主站请求的寄存器个数决定)
格式:[地址][0x03][字节数][数据 2N B][CRC低][CRC高]
长度:整帧 = 5 + 字节数 = 5 + 2×寄存器个数(N)
实例:[01][03][02][00 EB][xx][xx]
说明:字节数字段本身占1字节,其值=数据区字节数=2×N,必须是偶数
规范上限250(0xFA),超过即判异常

三.异常应答(固定5字节)
格式:[地址][功能码|0x80][异常码][CRC低][CRC高]
实例:[01][86][02][xx][xx]
异常码:0x01功能码不支持 0x02寄存器地址非法 0x03数据值非法 0x04从机故障
说明:FC06异常时功能码 = 0x06|0x80 = 0x86;本工程在"功能码比对"处
(rx[1] != MB_FC_WRITE_SINGLE_REGISTER)即拦成 -2,不会走到回显核对
*/
static int8_t mb_recv_frame(uint8_t * buf, uint16_t buf_size, uint16_t * len, uint32_t timeout_ms)
{
	uint16_t idx = 0;
	uint16_t expected;
	uint8_t fc;

	//入口边界:下面要先写buf[0]/buf[1]再推算帧长,不拦会越界写2字节
	if(buf == NULL || len == NULL || buf_size < 2U) return -6;

	//读从机地址:从机无应答时卡在这里直到timeout_ms
	if(!mb_read_byte_timeout(&buf[idx++], timeout_ms)) return -1;
	//读功能码
	if(!mb_read_byte_timeout(&buf[idx++], 100)) return -2;
	fc = buf[1];
	
	//异常响应:地址+功能码(带0x80)+异常码+CRC=5字节
	if(fc & 0x80) expected = 5;
	else
	{
		switch(fc)
		{
			case MB_FC_READ_HOLDING_REGISTERS:
				//FC03要先读到"字节数"才能定整帧长度
				if(!mb_read_byte_timeout(&buf[idx++], 100)) return -3;
				//Modbus规范:读寄存器响应的字节数最大250(0xFA),超过即异常,防越界
				if(buf[2] > 250U) return -4;
				expected = 5 + buf[2];
				break;

			case MB_FC_WRITE_SINGLE_REGISTER:
				expected = 8; //FC06从机原样回显请求帧,长度固定
				break;

			default: return -4;	//不支持的功能码
		}
	}

	//帧长最终校验:推断出的长度不能超过缓冲区容量
	if(expected > buf_size) return -5;

	//补齐剩余字节
	while(idx < expected)
	{
		if(!mb_read_byte_timeout(&buf[idx], 100)) return -5;
		idx++;
	}
	
	*len = idx;
	
	return 0;
}

//校验帧尾CRC:buf=整帧(含末尾2字节CRC),len=整帧长;通过返回1,失败返回0
static uint8_t mb_check_crc(const uint8_t * buf, uint16_t len)
{
	uint16_t crc_calc;
	uint16_t crc_recv;

	//len<2时下面len-2在uint16_t下会下溢成65534,按超长长度算CRC直接越界读
	if(buf == NULL || len < 2U) return 0;

	//CRC只覆盖"地址+功能码+数据",不含它自己
	crc_calc = mb_crc16(buf, len - 2);
	//Modbus帧尾CRC是低字节在前
	crc_recv = (uint16_t)((uint16_t)buf[len - 1] << 8) | buf[len - 2];

	return (crc_calc == crc_recv) ? 1 : 0;
}

/*
读保持寄存器(FC03):从slave的start_addr起连续读num_regs个,结果填入out
0成功,负数为失败码(见文件头)
参数:从机地址 起始寄存器地址 连续读几个寄存器 结果写到哪
*/
int8_t MB_ReadHoldingRegisters(uint8_t slave, uint16_t start_addr, uint16_t num_regs, uint16_t * out)
{
	uint8_t frame[8];
	uint8_t rx[256];
	uint16_t rxlen = 0;
	uint16_t crc;
	uint8_t byte_count;
	uint16_t i;

	//入口参数检查
	if(out == NULL) return -6;
	//广播地址禁止用于读写
	if(slave == 0) return -7;
	if(num_regs == 0 || num_regs > MB_MAX_REGS_PER_READ) return -8;

	//组请求帧:地址+0x03+起始地址+数量+CRC,共8字节
	frame[0] = slave;
	frame[1] = MB_FC_READ_HOLDING_REGISTERS;
	frame[2] = (uint8_t)(start_addr >> 8); //寄存器地址是大端:高字节先发
	frame[3] = (uint8_t)(start_addr & 0xFF);
	frame[4] = (uint8_t)(num_regs >> 8);
	frame[5] = (uint8_t)(num_regs & 0xFF);
	crc = mb_crc16(frame, 6);
	frame[6] = (uint8_t)(crc & 0xFF); //帧尾CRC是低字节先发
	frame[7] = (uint8_t)(crc >> 8);

	//先清掉总线上的陈旧字节再发,否则残留会被当成应答帧头
	mb_flush_rx();
	mb_send_frame(frame, 8);

	//收帧:任何失败都折叠成-1,对上层只报"无应答"
	if(mb_recv_frame(rx, (uint16_t)sizeof(rx), &rxlen, 100) != 0) return -1;
	//应答必须是本从机,本功能码
	if(rx[0] != slave || rx[1] != MB_FC_READ_HOLDING_REGISTERS) return -2;
	if(!mb_check_crc(rx, rxlen)) return -3;

	//严格校验字节数:必须偶数,且正好等于读取的寄存器个数(一个等于两字节)*2
	//短了out[]会留未初始化值,长了下面循环会越界写
	byte_count = rx[2];
	if((byte_count & 0x01U) != 0U || (byte_count / 2) != num_regs) return -4;

	//寄存器值也是大端
	for(i = 0; i < num_regs; i++)
	out[i] = (uint16_t)((uint16_t)rx[3 + 2 * i] << 8) | rx[3 + 2 * i + 1];

	return 0;
}

/*
写单个保持寄存器(FC06):把value写入slave的reg_addr,从机正常应答=原样回显请求帧
0成功,负数为失败码(见文件头)
参数:从机地址 寄存器地址 写入值
*/
int8_t MB_WriteSingleRegister(uint8_t slave, uint16_t reg_addr, uint16_t value)
{
	uint8_t frame[8];
	uint8_t rx[16];
	uint16_t rxlen = 0;
	uint16_t crc;

	//广播地址禁止用于读写
	if(slave == 0) return -7;

	//组请求帧:地址+0x06+寄存器地址+数值+CRC,共8字节
	frame[0] = slave;
	frame[1] = MB_FC_WRITE_SINGLE_REGISTER;
	frame[2] = (uint8_t)(reg_addr >> 8);	//寄存器地址是大端:高字节先发
	frame[3] = (uint8_t)(reg_addr & 0xFF);
	frame[4] = (uint8_t)(value >> 8);	//数值也是大端
	frame[5] = (uint8_t)(value & 0xFF);
	crc = mb_crc16(frame, 6);
	frame[6] = (uint8_t)(crc & 0xFF);	//帧尾CRC是低字节先发
	frame[7] = (uint8_t)(crc >> 8);

	//先清掉总线上的陈旧字节再发,否则残留会被当成应答帧头
	mb_flush_rx();
	mb_send_frame(frame, 8);

	//收帧:任何失败都折叠成-1,对上层只报"无应答"
	if(mb_recv_frame(rx, (uint16_t)sizeof(rx), &rxlen, 100) != 0) return -1;
	//应答必须是本从机,本功能码(从机报异常时回0x86,这里就拦下了)
	if(rx[0] != slave || rx[1] != MB_FC_WRITE_SINGLE_REGISTER) return -2;
	if(!mb_check_crc(rx, rxlen)) return -3;

	//回显核对:正常响应是原样回显请求帧,即rx[2-5]应与frame[2-5]逐字节相同
	//不核对会被"错误回显"骗过去,把其实没生效的写入当成写成功
	if(rx[2] != frame[2] || rx[3] != frame[3] || rx[4] != frame[4] || rx[5] != frame[5]) return -4;

	return 0;
}
