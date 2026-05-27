#include "Servo.h"
#include "tim.h"

/* TIM1配置：PSC=2399, ARR=1999 → 周期=20ms
    注意：不同舵机的行程对应的脉宽范围不同。许多舵机的全行程为 0.5ms~2.5ms（2.0ms 脉宽范围），
    如果只使用 1.0ms~2.0ms，可能只能转约 90°。下面使用 0.5ms~2.5ms 范围以覆盖常见的 180° 舵机。 */
#define SERVO_PULSE_MIN   50   /* 0度对应0.5ms */
#define SERVO_PULSE_MAX  250   /* 180度对应2.5ms */

/* 舵机句柄缓存，避免每次重复extern */
static TIM_HandleTypeDef *servo_tim = NULL;
static uint16_t current_angle = 0;
static uint16_t last_set_angle = 0xFFFF;  /* 用于记录上次设置的角度，避免重复设置 */

/**
 * @brief 初始化舵机
 */
void Servo_Init(void)
{
    extern TIM_HandleTypeDef htim1;
    servo_tim = &htim1;  /* 一次性保存句柄，后续直接使用 */
    HAL_TIM_PWM_Start(servo_tim, TIM_CHANNEL_1);
}

/**
 * @brief 设置舵机角度
 */
void Servo_SetAngle(uint16_t angle)
{
    if (servo_tim == NULL) return;  /* 检查初始化 */
    
    if (angle > 180) angle = 180;
    
    current_angle = angle;
    
    /* 如果角度未变化，无需重复设置 */
    if (last_set_angle == angle) return;
    last_set_angle = angle;
    
    /* 线性映射：0~180° 对应 100~200 的脉宽值 */
    uint16_t pulse = SERVO_PULSE_MIN + (angle * (SERVO_PULSE_MAX - SERVO_PULSE_MIN)) / 180;
    
    __HAL_TIM_SetCompare(servo_tim, TIM_CHANNEL_1, pulse);
}

/**
 * @brief 获取舵机当前角度
 */
uint16_t Servo_GetAngle(void)
{
    return current_angle;
}
