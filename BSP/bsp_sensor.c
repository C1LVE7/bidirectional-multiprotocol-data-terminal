/*
AHT20:温湿度传感器驱动

通信流程
1.初始化:发送0xBE 0x08 0x00(校准),等待10ms
2.测量:发送0xAC 0x33 0x00,等待80ms
3.读取7个字节,索引:0指状态,1-5指数据,6指CRC校验位

数据计算:湿度1-2字节全部,3字节前四位比特,温度3字节后四位比特
4-5字节全部,代码按定点实现,乘1000后除2^20,输出为实际值的10倍
20bit/2^20*1000*量程(1000是100%百分比*扩大10倍)
*/
#include "bsp_sensor.h"
#include "i2c.h"
#include "bsp_uart.h"
#include "FreeRTOS.h"
#include "task.h"

//调度器启动后用vTaskDelay,否则用HAL_Delay
static void sensor_delay_ms(uint32_t ms)
{
    if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) vTaskDelay(pdMS_TO_TICKS(ms));
    else HAL_Delay(ms);
}

/*
判断CRC是否正确 方向:MSB-first
多项式实际为100110001九位 CRCx的第x+1位均为1
为1时左移将剩下的数进行异或(与多项式0x31)即可
直至所有字节均校验完成,总结就是:异或=不进位加法
*/
static uint8_t aht20_crc8(const uint8_t * data, uint8_t len)
{
    uint8_t crc = 0xFF;
    uint8_t i, j;

    for(i = 0; i < len; i++)
    {
        crc ^= data[i];
        for(j = 0; j < 8; j++)
        {
            if(crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x31);
            else crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

//初始化(校准),返回0为成功
int8_t BSP_AHT20_Init(void)
{
	//校准命令
	uint8_t init_cmd[3] = {0xBE, 0x08, 0x00}; 
	//上电后等待40ms,使传感器稳定
	sensor_delay_ms(40);
	//写入并判断是否成功
	if(HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDR, init_cmd, 3, 100) != HAL_OK) return -1;
	//发送校准命令后延时
	sensor_delay_ms(10);
	
	return 0;
}

/* 
读取一次温湿度：
temp:温度 humi:湿度

返回0成功,负数失败
-1 I2C写失败(触发测量命令) -2 I2C读失败(读数据帧)
-3 传感器忙(状态位 bit7)   -4 CRC校验失败且数据超出合理范围
-5 输出指针为NULL          -6 校准未使能(状态位bit3=0,模块未校准/初始化异常)
  
当CRC校验失败但温湿度在合理范围(温度0.0~50.0℃ 湿度0~100.0%)时
按宽容校验降级接受并打印告警,以兼容部分不输出有效CRC的市场模块固件
 */
int8_t BSP_AHT20_Read(int16_t * temp, int16_t * humi)
{
	uint8_t trigger[3] = {0xAC, 0x33, 0x00}; //测量命令
	uint8_t data[7] = {0}; //数据存储
	uint32_t raw_h, raw_t;
	int16_t  t, h;
	uint8_t  crc_ok;
	
	//判断指针是否为空
	if(temp == NULL || humi == NULL) return -5;
	//写入测量命令
	if(HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDR, trigger, 3, 100) != HAL_OK) return -1;
	//按照手册要求写入测量命令后延时
	sensor_delay_ms(80);
	//读取数据
	if(HAL_I2C_Master_Receive(&hi2c1, AHT20_ADDR, data, 7, 100) != HAL_OK) return -2;
	//bit7=1为传感器正忙
	if(data[0] & 0x80) return -3;
	//bit3=0为校准未使能
	if((data[0] & 0x08) == 0)
	{
		bsp_log("AHT20: calibration not enabled (status=0x%02X), data rejected\r\n", data[0]);
		return -6;
	}
	
	//转化数据
	raw_h = ((uint32_t)data[1] << 12) |
			((uint32_t)data[2] << 4) |
			(data[3] >> 4);
	raw_t = (((uint32_t)data[3] & 0x0F) << 16) |
			((uint32_t)data[4] << 8) |
			data[5];
	//两个参数的数据为实际数据的十倍
	h = (int16_t)(raw_h * 1000 / 1048576);
	t = (int16_t)((int32_t)(raw_t * 2000 / 1048576) - 500);
	//CRC宽容校验:失败但数据在合理范围降级接受,超范围会拒收
	crc_ok = (aht20_crc8(data, 6) == data[6]);
	if(!crc_ok)
	{
		if(t >= 0 && t <= 500 && h >= 0 && h <= 1000) bsp_log("AHT20 CRC fail, accepted (data in range)\r\n");
		else return -4;
	}
	//把结果通过指针带出去,返回成功
	*humi = h;
	*temp = t;
	
	return 0;
}

