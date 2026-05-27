#ifndef __IR_H
#define __IR_H

#include "stdint.h"

#define IR_SENSOR_COUNT 9          // 红外传感器数量
#define IR_MAX_DISTANCE 999.0f     // 最大有效距离

/**
 * @brief 红外传感器相关初始化
 */
void IR_Init(void);

/**
 * @brief  计算红外传感器的实际距离
 * @param  pRawData: 红外传感器对应的ADC原始数据数组首地址（需要跳过灰度传感器的通道）
 * @param  pOutDistances: 距离计算结果输出数组首地址 (长度应为 IR_SENSOR_COUNT)
 * @note   低耦合设计，不直接依赖全局ADC数组，支持传入任意合法的数据源
 */
void IR_CalculateDistances(const uint16_t *pRawData, float *pOutDistances);

#endif // __IR_H
