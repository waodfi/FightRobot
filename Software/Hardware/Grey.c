#include "Grey.h"
#include <stddef.h>

void Grey_Init(void)
{
    // 当前硬件无需额外初始化，保留统一接口
}

void Grey_CalculateValues(const uint16_t *pRawData, float *pOutValues)
{
    int i;

    if (pRawData == NULL || pOutValues == NULL)
    {
        return;
    }

    // 根据你提供的公式化简后: Grey = ADC_Raw / 256
    for (i = 0; i < GREY_SENSOR_COUNT; i++)
    {
        pOutValues[i] = (float)pRawData[i] / 256.0f;
    }
}
