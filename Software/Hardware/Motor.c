#include "Motor.h"
#include "main.h"  /* 包含HAL库及GPIO定义 */
#include "tim.h"   /* 包含htim8的声明 */
#include <stdlib.h>

extern float motor_rc_speed[4];

/* 电机硬件配置结构体 */
typedef struct {
    GPIO_TypeDef* GPIOx;        /* 方向控制端口 */
    uint16_t      GPIO_Pin;     /* 方向控制引脚 */
    uint32_t      TIM_Channel;  /* PWM使用的定时器通道 */
    GPIO_PinState FwdState;     /* 正转时的GPIO引脚状态 */
} Motor_Config_t;

/* 定义4个电机的硬件映射表 (依据原理图及TIM8的通道分配) */
/* 映射关系依据：
 * Motor 1: DIR=PB13, PWM=PC8  (TIM8_CH3), 正转=RESET
 * Motor 2: DIR=PB14, PWM=PC7  (TIM8_CH2), 正转=SET
 * Motor 3: DIR=PB15, PWM=PC9  (TIM8_CH4), 正转=RESET
 * Motor 4: DIR=PB12, PWM=PC6  (TIM8_CH1), 正转=SET */
static const Motor_Config_t MotorConfig[4] = {
    {GPIOB, GPIO_PIN_13, TIM_CHANNEL_3, GPIO_PIN_RESET},   /* Motor 1 */
    {GPIOB, GPIO_PIN_14, TIM_CHANNEL_2, GPIO_PIN_SET}, /* Motor 2 */
    {GPIOB, GPIO_PIN_15, TIM_CHANNEL_4, GPIO_PIN_RESET},   /* Motor 3 */
    {GPIOB, GPIO_PIN_12, TIM_CHANNEL_1, GPIO_PIN_SET}  /* Motor 4 */
};

extern TIM_HandleTypeDef htim8;

/* 添加：电机转速测量相关变量 */
static volatile uint32_t Motor_Pulses[4] = {0, 0, 0, 0};  /* 脉冲计数 */
static int8_t            Motor_Dirs[4]   = {1, 1, 1, 1};  /* 当前记录的方向(带符号) */
static float             Motor_RPM[4]    = {0.0f, 0.0f, 0.0f, 0.0f}; /* 实际转速 */

/**
    * @brief  以“摇杆语义”生成四轮基础速度
    * @param  X: 前后方向输入 (-100 到 +100)
    * @param  Y: 左右方向输入 (-100 到 +100)
    */
void Motor_Control(int16_t X, int16_t Y)
{
        if (X > 100) X = 100;
        if (X < -100) X = -100;
        if (Y > 100) Y = 100;
        if (Y < -100) Y = -100;

        /* 与遥控模式的组合方式保持一致：
             左侧轮组 = X - Y
             右侧轮组 = X + Y */
        motor_rc_speed[MOTOR_1 - 1] = (float)(Y - X);
        motor_rc_speed[MOTOR_2 - 1] = (float)(Y + X);
        motor_rc_speed[MOTOR_3 - 1] = (float)(Y - X);
        motor_rc_speed[MOTOR_4 - 1] = (float)(Y + X);
}



/**
  * @brief  电机驱动函数
  * @param  num:   电机编号 (1 到 4)
  * @param  Speed: 电机速度 (-100 到 +100)
  */
void Motor_SetSpeed(uint8_t num, int8_t Speed)
{
    /* 1. 安全检查，防止数组越界 */
    if (num < 1 || num > 4) 
    {
        return; 
    }
    
    /* 2. 限制速度在合理范围 -100 到 100 */
    if (Speed > 100)  Speed = 100;
    if (Speed < -100) Speed = -100;

    /* 3. 获取对应电机的硬件配置参数 */
    const Motor_Config_t* cfg = &MotorConfig[num - 1];

    /* 4. 控制方向并记录方向标志（供测速判断实际旋转正负） */
    if (Speed > 0)
    {
        Motor_Dirs[num - 1] = 1;
        HAL_GPIO_WritePin(cfg->GPIOx, cfg->GPIO_Pin, cfg->FwdState);
    }
    else if (Speed < 0)
    {
        Motor_Dirs[num - 1] = -1;
        /* 反转状态为正转状态的反相 */
        GPIO_PinState revState = (cfg->FwdState == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        HAL_GPIO_WritePin(cfg->GPIOx, cfg->GPIO_Pin, revState);
    }
    else /* Speed == 0 时的原始逻辑：TogglePin */
    {
        Motor_Dirs[num - 1] = 1;
        HAL_GPIO_TogglePin(cfg->GPIOx, cfg->GPIO_Pin);
    }

    /* 5. 输出PWM占空比 (使用绝对值) */
    __HAL_TIM_SetCompare(&htim8, cfg->TIM_Channel, abs(Speed));
}

/**
  * @brief  电机脉冲中断捕获回调 (应在 HAL_TIM_IC_CaptureCallback 等中断中调用)
  * @param  num: 电机编号 (1-4)
  */
void Motor_PulseCallback(uint8_t num)
{
    if (num >= 1 && num <= 4)
    {
        Motor_Pulses[num - 1]++;
    }
}

/**
  * @brief  计算并更新电机的实际转速（RPM），请在定时器（如10ms或50ms）中断中周期调用
  * @param  dt_s: 调用此函数的周期，单位秒 (如 10ms 传入 0.01f)
  */
void Motor_UpdateSpeed(float dt_s)
{
    if(dt_s <= 0.0f) return;

    for (uint8_t i = 0; i < 4; i++)
    {
        /* 
         * 1圈 = 9个脉冲
         * Hz = (累计脉冲数 / 9) / dt_s
         * RPM = Hz * 60 = (脉冲数 / 9) / dt_s * 60
         */
        float speed_rpm = ((float)Motor_Pulses[i] / 9.0f) / dt_s * 60.0f;
        
        /* 附加方向符号，因为单线脉冲无法识别方向，这取决于给定的电压极性 */
        Motor_RPM[i] = speed_rpm * Motor_Dirs[i];

        /* 清零脉冲，准备下一个周期的计数 */
        Motor_Pulses[i] = 0;
    }
}

/**
  * @brief  获取缓存中的电机转速（RPM）
  */
float Motor_GetRPM(uint8_t num)
{
    if (num >= 1 && num <= 4) return Motor_RPM[num - 1];
    return 0.0f;
}



