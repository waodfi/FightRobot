#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

/* 枚举电机编号，方便调用 */
typedef enum {
    MOTOR_1 = 1,  /* 左前 */
    MOTOR_2 = 2,  /* 右前 */
    MOTOR_3 = 3,  /* 左后 */
    MOTOR_4 = 4   /* 右后 */
} Motor_ID_e;


/**
  * @brief  设置电机速度
  * @param  num:   电机编号 (1~4，或使用 Motor_ID_e 枚举)
  * @param  Speed: 电机速度 (-100 到 +100)
  */
void Motor_SetSpeed(uint8_t num, int8_t Speed);

/* ---- 编码器测速相关 ---- */

/**
  * @brief  以“摇杆语义”控制底盘基础运动
  * @param  X: 前后方向输入 (-100 到 +100)
  * @param  Y: 左右方向输入 (-100 到 +100)
  *
  * 说明：
  * - 该函数只设置四轮的基础目标速度，不直接驱动 PWM
  * - Angle_Task 可在此基础上叠加角度修正量
  */
void Motor_Control(int16_t X, int16_t Y);

/**
  * @brief  电机脉冲中断捕获回调 (在 TIM2 捕获中断里被调用，传入对应电机ID)
  * @param  num: 电机编号 (1-4)
  */
void Motor_PulseCallback(uint8_t num);

/**
  * @brief  读取脉冲并转换为实际转速（RPM），必须在定时器(如10ms或50ms)中周期调用
  * @param  dt_s: 更新周期，单位 秒（如果每10ms调用一次，则传入0.01f）
  */
void Motor_UpdateSpeed(float dt_s);

/**
  * @brief  获取电机的实时转速
  * @param  num: 电机编号 (1-4)
  * @retval 带有方向的当前转速 RPM
  */
float Motor_GetRPM(uint8_t num);

#endif /* __MOTOR_H */
