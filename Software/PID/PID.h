#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/** 
  * @brief PID结构体定义
  */
typedef struct {
    /* 参数区域 */
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数

    float MaxOut;       // PID总输出限幅
    float MaxIOut;      // 积分项限幅

    float DeadBand;     // 误差死区：小于此误差时跳过计算防止频繁震荡
    float I_Separation; // 积分分离阈值：误差在此范围外时停止积分，防超调

    /* 运行数据区域 */
    float Target;       // 目标值
    float Measured;     // 测量值与反馈值

    float Error;        // 偏差
    float LastError;    // 上次偏差
    
    float POut;         // 比例项输出
    float IOut;         // 积分项输出
    float DOut;         // 微分项输出
    float Out;          // 总输出
} PID_t;

/**
  * @brief PID初始化
  * @param pid:      PID结构体指针
  * @param Kp:       比例系数
  * @param Ki:       积分系数
  * @param Kd:       微分系数
  * @param max_out:  总输出限幅
  * @param max_iout: 积分限幅
  * @param deadband: 误差死区(误差小于该值不再产生输出波动)
  * @param i_sep:    积分分离阈值(误差大于该值时不积分)
  */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float max_out, float max_iout, float deadband, float i_sep);

/**
  * @brief PID计算函数 (位置式)
  * @param pid:      PID结构体指
  * @param target:   目标值
  * @param measured: 当前测量反馈值
  * @retval 经过PID计算后受限幅的总输出
  */
float PID_Calc(PID_t *pid, float target, float measured);

/**
  * @brief 清除PID内部积分和微分历史状态
  * @param pid: PID构体指针
  */
void PID_Clear(PID_t *pid);

#endif /* __PID_H */
