#ifndef __GREY_H
#define __GREY_H

#include <stdint.h>

#define GREY_SENSOR_COUNT 4

/**
 * @brief 灰度模块初始化预留接口
 */
void Grey_Init(void);

/**
 * @brief 由 ADC 原始值计算灰度值
 * @param pRawData 灰度 ADC 原始数组首地址，长度至少为 GREY_SENSOR_COUNT
 * @param pOutValues 输出灰度值数组首地址，长度至少为 GREY_SENSOR_COUNT
 */
void Grey_CalculateValues(const uint16_t *pRawData, float *pOutValues);

#endif
