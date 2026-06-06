#include "control.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "stdio.h"

#define RX_BUF_SIZE 128

// DMA 接收缓存
static uint8_t rx_buffer[RX_BUF_SIZE];
// 全局手柄数据实例
static PC_ControlData_t g_control_data;

/**
 * @brief 初始化，开启 USART1 IDLE + DMA 接收
 */
void Control_Init(void) {
    memset(&g_control_data, 0, sizeof(PC_ControlData_t));
    g_control_data.last_update_time = xTaskGetTickCount();
    
    // 开启串口1的 DMA 空闲中断接收机制
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUF_SIZE);
}

/**
 * @brief  获取最新数据包
 */
PC_ControlData_t Control_GetData(void) {
    return g_control_data;
}

/**
 * @brief 检查手柄是否掉线 (超时 500ms 判为掉线)
 */
uint8_t Control_IsOnline(void) {
    if ((xTaskGetTickCount() - g_control_data.last_update_time) > 500) {
        return 0; // Offline
    }
    return 1; // Online
}

/**
 * @brief 数据包解析函数
 * @param buf 收到的字符串数据
 */
static void Control_ParseData(const char *buf) {
    int vx, vy, lt, rt, buttons;
    
    // 查找包头 '<'
    char *start = strchr(buf, '<');
    if (start != NULL) {
        // 使用 sscanf 提取数据
        if (sscanf(start, "<%d,%d,%d,%d,%d>", &vx, &vy, &lt, &rt, &buttons) == 5) {
            // 解析成功，更新全局变量
            g_control_data.vx_fwd = vx;
            g_control_data.vy_lr = vy;
            g_control_data.lt = lt;
            g_control_data.rt = rt;
            g_control_data.buttons = buttons;
            
            // 更新时间戳，用于在线检测
            g_control_data.last_update_time = xTaskGetTickCount();
        }
    }
}

extern void Trigger_Debug_Launch(void);
extern void Trigger_Debug_RunOnstage(void);
extern void Trigger_Debug_ClimbScan(void);
extern void Trigger_Debug_Stop(void);

/**
 * @brief 供主外设中断或 HAL_UARTEx_RxEventCallback 调用的串口接收业务函数
 */
void Control_UART_RxCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        // 添加字符串结束符保证 strchr 不越界
        rx_buffer[Size < RX_BUF_SIZE ? Size : (RX_BUF_SIZE - 1)] = '\0';
        
        const char *text = (const char*)rx_buffer;

        if (strstr(text, "@RUN") != NULL || strstr(text, "@run") != NULL) {
            Trigger_Debug_RunOnstage();
        } else if (strstr(text, "@CLIMB") != NULL || strstr(text, "@climb") != NULL) {
            Trigger_Debug_ClimbScan();
        } else if (strstr(text, "@STOP") != NULL || strstr(text, "@stop") != NULL) {
            Trigger_Debug_Stop();
        } else if (strstr(text, "Y") != NULL || strstr(text, "y") != NULL) {
            Trigger_Debug_Launch();
        } else {
            // 解析数据
            Control_ParseData(text);
        }
        
        // 重新开启 DMA 接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUF_SIZE);
    }
}
