#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

/* IMU 数据结构体 */
typedef struct {
    float AccX;
    float AccY;
    float AccZ;

    float GyroX;
    float GyroY;
    float GyroZ;

    float Roll;
    float Pitch;
    float Yaw;

    float Temp;
} IMU_Data_t;

/* 外部声明，供其他文件调用 */
extern IMU_Data_t IMU_Data;

/* 模块初始化 */
void IMU_Init(void);

/* 数据接收与解析（在串口接收中断中调用，传入接收到的1个字节）*/
void IMU_ReceiveByte(uint8_t data);

/* 上位机至模块：Z轴角度归零 */
void IMU_ZeroZAxis(void);

/* --- 用户需要在底层（如 usart.c 中）实现的硬件发送接口 --- */
// 示例: void IMU_HW_SerialSend(uint8_t *data, uint16_t len) { HAL_UART_Transmit(&huart1, data, len, 100); }
extern void IMU_HW_SerialSend(uint8_t *data, uint16_t len);

#endif /* __IMU_H */
