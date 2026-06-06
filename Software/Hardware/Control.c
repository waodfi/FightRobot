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
extern void Trigger_Debug_VisionOnly(void);
extern void Trigger_Debug_VisionActive(void);

#define CMD_ACC_SIZE 256
static char s_cmd_acc_buf[CMD_ACC_SIZE];
static uint16_t s_cmd_acc_len = 0;

/**
 * @brief 供主外设中断或 HAL_UARTEx_RxEventCallback 调用的串口接收业务函数
 */
void Control_UART_RxCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        // 将新收到的字节累加到静态缓冲区中
        for (uint16_t i = 0; i < Size; i++) {
            char c = (char)rx_buffer[i];
            if (s_cmd_acc_len < CMD_ACC_SIZE - 1) {
                s_cmd_acc_buf[s_cmd_acc_len++] = c;
            } else {
                // 缓冲区满，整体左移腾出空间
                (void)memmove(s_cmd_acc_buf, s_cmd_acc_buf + 1, CMD_ACC_SIZE - 1);
                s_cmd_acc_buf[CMD_ACC_SIZE - 2] = c;
            }
        }
        s_cmd_acc_buf[s_cmd_acc_len] = '\0';
        
        uint8_t cmd_matched = 0;
        if (strstr(s_cmd_acc_buf, "@RUN") != NULL || strstr(s_cmd_acc_buf, "@run") != NULL) {
            Trigger_Debug_RunOnstage();
            cmd_matched = 1;
        } else if (strstr(s_cmd_acc_buf, "@CLIMB") != NULL || strstr(s_cmd_acc_buf, "@climb") != NULL) {
            Trigger_Debug_ClimbScan();
            cmd_matched = 1;
        } else if (strstr(s_cmd_acc_buf, "@VISION_ACTIVE") != NULL || strstr(s_cmd_acc_buf, "@vision_active") != NULL ||
                   strstr(s_cmd_acc_buf, "@VISIONA") != NULL || strstr(s_cmd_acc_buf, "@visiona") != NULL) {
            Trigger_Debug_VisionActive();
            cmd_matched = 1;
        } else if (strstr(s_cmd_acc_buf, "@VISION") != NULL || strstr(s_cmd_acc_buf, "@vision") != NULL) {
            Trigger_Debug_VisionOnly();
            cmd_matched = 1;
        } else if (strstr(s_cmd_acc_buf, "@STOP") != NULL || strstr(s_cmd_acc_buf, "@stop") != NULL) {
            Trigger_Debug_Stop();
            cmd_matched = 1;
        } else if (strstr(s_cmd_acc_buf, "Y") != NULL || strstr(s_cmd_acc_buf, "y") != NULL) {
            Trigger_Debug_Launch();
            cmd_matched = 1;
        } else {
            // 检查累积缓冲区中是否包含一个完整的控制指令数据包，即包含 '<' 且其后有 '>'
            char *start = strchr(s_cmd_acc_buf, '<');
            char *end = strchr(s_cmd_acc_buf, '>');
            if (start != NULL && end != NULL && end > start) {
                Control_ParseData(s_cmd_acc_buf);
                // 清除已解析过的这包数据
                uint16_t parsed_len = (uint16_t)(end - start + 1);
                (void)memmove(s_cmd_acc_buf, end + 1, (size_t)(s_cmd_acc_len - (end - s_cmd_acc_buf)));
                s_cmd_acc_len -= parsed_len;
                s_cmd_acc_buf[s_cmd_acc_len] = '\0';
            }
        }
        
        if (cmd_matched) {
            // 一旦匹配到文本调试指令，直接清空累积缓冲区
            s_cmd_acc_len = 0;
            s_cmd_acc_buf[0] = '\0';
        }
        
        // 重新开启 DMA 接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUF_SIZE);
    }
}

