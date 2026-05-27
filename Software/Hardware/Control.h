#ifndef __CONTROL_H
#define __CONTROL_H

#include "stdint.h"
#include "main.h"

// 定义手柄数据结构体
typedef struct {
    int vx_fwd;       // 左摇杆前后 (-100 ~ 100)
    int vy_lr;        // 右摇杆左右 (-100 ~ 100)
    int lt;           // 左扳机 (0 ~ 100)
    int rt;           // 右扳机 (0 ~ 100)
    int buttons;      // 按键掩码
    uint32_t last_update_time; // 上次接收数据的时间戳（用于断连检测）
} PC_ControlData_t;

// 初始化控制模块 (启动 DMA 接收)
void Control_Init(void);

// 获取最新解析的手柄数据
PC_ControlData_t Control_GetData(void);

// 供原生串口回调路由的入口接管函数
void Control_UART_RxCallback(UART_HandleTypeDef *huart, uint16_t Size);

// 保护机制：检测是否掉线 (例如超过 500ms 没收到PC数据)
uint8_t Control_IsOnline(void);

#endif // __CONTROL_H
