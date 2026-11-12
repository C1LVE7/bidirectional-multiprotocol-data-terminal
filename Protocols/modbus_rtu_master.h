#ifndef MODBUS_RTU_MASTER_H_
#define MODBUS_RTU_MASTER_H_

#include "app_config.h"
#include <stdint.h>

//功能码
#define MB_FC_READ_HOLDING_REGISTERS   0x03	//读保持寄存器
#define MB_FC_WRITE_SINGLE_REGISTER    0x06	//写单个寄存器

//读保持寄存器单次最大个数:响应字节数<=250,Modbus规范上限
#define MB_MAX_REGS_PER_READ   125U

//读保持寄存器(FC03)
int8_t MB_ReadHoldingRegisters(uint8_t slave, uint16_t start_addr, uint16_t num_regs, uint16_t * out);

//写单个保持寄存器(FC06)
int8_t MB_WriteSingleRegister(uint8_t slave, uint16_t reg_addr, uint16_t value);

#endif
