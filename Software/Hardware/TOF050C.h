#ifndef __TOF050C_H
#define __TOF050C_H

#include "stdint.h"
#include "SoftI2C.h"

// TOF050C 的 I2C 8位地址
#define TOF050C_ADDR_WRITE 0x52
#define TOF050C_ADDR_READ  0x53

/**
 * @brief  初始化模块（只需初始化对应的 SoftI2C 总线）
 * @param  bus: SOFT_I2C_BUS_1 或 SOFT_I2C_BUS_2
 */
void TOF050C_Init(SoftI2C_Bus_e bus);

/**
 * @brief  读取模块的距离
 * @param  bus: SOFT_I2C_BUS_1 或 SOFT_I2C_BUS_2
 * @return 返回距离(单位: 毫米mm)。如果返回 0xFFFF 则说明通信失败或没有连接
 */
uint16_t TOF050C_ReadDistance(SoftI2C_Bus_e bus);

#endif