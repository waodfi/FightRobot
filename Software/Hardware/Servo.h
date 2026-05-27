#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

/**
 * @brief 舵机初始化（启动TIM1 PWM输出）
 */
void Servo_Init(void);

/**
 * @brief 设置舵机角度
 * @param angle 目标角度，范围 0 ~ 180 度
 */
void Servo_SetAngle(uint16_t angle);

/**
 * @brief 获取舵机当前角度
 * @retval 当前设置角度（0 ~ 180）
 */
uint16_t Servo_GetAngle(void);

#endif /* __SERVO_H */
