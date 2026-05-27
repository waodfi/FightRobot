#include "IR.h"
#include <math.h>
#include <stddef.h>

void IR_Init(void)
{
    // 如果之后有使能引脚或其它红外专属硬件初始化，可以在这里添加
}

void IR_CalculateDistances(const uint16_t *pRawData, float *pOutDistances)
{
    if (pRawData == NULL || pOutDistances == NULL)
    {
        return;
    }

    for (int i = 0; i < IR_SENSOR_COUNT; i++)
    {
        // 1. 将16位ADC原始值转换为模拟电压 (参考电压3.3V, 分辨率65536)
        float voltage = (float)pRawData[i] * 3.3f / 65536.0f;

        // 2. 避免电压接近0导致指数运算溢出或错误
        if (voltage < 0.001f)
        {
            pOutDistances[i] = IR_MAX_DISTANCE;
        }
        else
        {
            // 3. 根据固定的非线性公式计算距离
            // Formula: Distance = 60.374 * voltage^(-1.16)
            pOutDistances[i] = 60.374f * powf(voltage, -1.16f);

            // 4. 对计算结果进行限幅处理
            if (pOutDistances[i] > IR_MAX_DISTANCE)
            {
                pOutDistances[i] = IR_MAX_DISTANCE;
            }
        }
    }
}
