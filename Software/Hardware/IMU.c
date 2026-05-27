#include "IMU.h"

IMU_Data_t IMU_Data = {0};

/* 接收缓存区与状态机变量 */
static uint8_t rx_buf[11];
static uint8_t rx_cnt = 0;

/**
  * @brief  IMU初始化
  */
void IMU_Init(void)
{
    rx_cnt = 0;
}

/**
  * @brief  解析一字节来自IMU串口的数据（支持解包：加速度包、角速度包、角度包）
  * @param  data: UART接收到的1字节数据
  */
void IMU_ReceiveByte(uint8_t data)
{
    rx_buf[rx_cnt] = data;

    // 起始帧头检查
    if (rx_cnt == 0 && rx_buf[0] != 0x55) 
    {
        return; 
    }

    rx_cnt++;

    // 接收满11个字节进行解析
    if (rx_cnt >= 11)
    {
        rx_cnt = 0; // 重置计数器，准备下一帧

        // 校验和 (低八位校验)
        uint8_t sum = 0;
        for (uint8_t i = 0; i < 10; i++)
        {
            sum += rx_buf[i];
        }

        if (sum != rx_buf[10]) 
        {
            return; // 校验失败，抛弃这一帧
        }

        // 提取数据部分组合为16位数据
        int16_t data_x = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
        int16_t data_y = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
        int16_t data_z = (int16_t)((rx_buf[7] << 8) | rx_buf[6]);
        int16_t data_t = (int16_t)((rx_buf[9] << 8) | rx_buf[8]);

        // 根据包类型进行换算
        switch (rx_buf[1])
        {
            case 0x51: /* 加速度包 */
                IMU_Data.AccX = ((float)data_x / 32768.0f) * 16.0f;
                IMU_Data.AccY = ((float)data_y / 32768.0f) * 16.0f;
                IMU_Data.AccZ = ((float)data_z / 32768.0f) * 16.0f;
                IMU_Data.Temp = ((float)data_t / 340.0f) + 36.53f;
                break;
                
            case 0x52: /* 角速度包 */
                IMU_Data.GyroX = ((float)data_x / 32768.0f) * 2000.0f;
                IMU_Data.GyroY = ((float)data_y / 32768.0f) * 2000.0f;
                IMU_Data.GyroZ = ((float)data_z / 32768.0f) * 2000.0f;
                IMU_Data.Temp  = ((float)data_t / 340.0f) + 36.53f;
                break;
                
            case 0x53: /* 角度包 */
                IMU_Data.Roll  = ((float)data_x / 32768.0f) * 180.0f;
                IMU_Data.Pitch = ((float)data_y / 32768.0f) * 180.0f;
                IMU_Data.Yaw   = ((float)data_z / 32768.0f) * 180.0f;
                IMU_Data.Temp  = ((float)data_t / 340.0f) + 36.53f;
                break;
                
            default:
                break;
        }
    }
}

/**
  * @brief  发送指令使Z轴角度归零
  */
void IMU_ZeroZAxis(void)
{
    uint8_t cmd[3] = {0xFF, 0xAA, 0x52};
    // 调用弱耦合的底层接口，实现跨平台/多串口复用
    IMU_HW_SerialSend(cmd, 3);
}

/**
  * @brief  默认的弱定义发送接口。如果用户在其他文件（如 usart.c）中未实现 IMU_HW_SerialSend，
  *         编译时会自动链接此弱定义，避免引发 Undefined symbol 链接错误。
  */
__attribute__((weak)) void IMU_HW_SerialSend(uint8_t *data, uint16_t len)
{
    /* 弱函数占位，什么也不做。
       如需实际发送，请在 usart.c 或 main.c 中重新实现一个同名的无 weak 函数：
       void IMU_HW_SerialSend(uint8_t *data, uint16_t len)
       {
           HAL_UART_Transmit(&huartx, data, len, 100);
       }
    */
    (void)data;
    (void)len;
}

