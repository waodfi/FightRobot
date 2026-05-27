#include "PID.h"
#include <stddef.h>  /* 包含 NULL 的定义 */

/**
  * @brief 宏定义：限幅函数
  */
#define LIMIT_MIN_MAX(x, min, max) (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/**
  * @brief  PID参数初始化
  * @param  pid: 指向PID结构体的指针
  * @param  Kp: 比例系数
  * @param  Ki: 积分系数
  * @param  Kd: 微分系数
  * @param  max_out: PID总输出的绝对值限幅 (如电机PWM的最大值100)
  * @param  max_iout: 积分项输出的绝对值限幅 (防止积分过大引起的超调)
  * @param  deadband: 误差死区
  * @param  i_sep: 积分分离阈值
  */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float max_out, float max_iout, float deadband, float i_sep)
{
    if (pid == NULL) return;

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    
    pid->MaxOut  = max_out;
    pid->MaxIOut = max_iout;

    pid->DeadBand     = deadband;
    pid->I_Separation = i_sep;

    PID_Clear(pid);
}

/**
  * @brief  清空PID运行状态
  */
void PID_Clear(PID_t *pid)
{
    if (pid == NULL) return;

    pid->Target    = 0.0f;
    pid->Measured  = 0.0f;
    pid->Error     = 0.0f;
    pid->LastError = 0.0f;
    pid->POut      = 0.0f;
    pid->IOut      = 0.0f;
    pid->DOut      = 0.0f;
    pid->Out       = 0.0f;
}

/**
  * @brief  位置式 PID 增量计算
  * @param  pid: 传入需要计算的PID结构体
  * @param  target: 目标传速（或角度/位置等）
  * @param  measured: 编码器测量出的实际转速（同单位）
  * @retval pid->Out (计算完毕的总PWM输出/响应)
  */
float PID_Calc(PID_t *pid, float target, float measured)
{
    if (pid == NULL) return 0.0f;

    pid->Target   = target;
    pid->Measured = measured;
    
    // 1. 计算偏差
    pid->Error = pid->Target - pid->Measured;

    // 加入死区：如果偏差足够小，可以认为目标已达到，不再波动。可选择在这个区间里是否冻结当前误差
    if (pid->Error > -pid->DeadBand && pid->Error < pid->DeadBand) {
        pid->Error = 0.0f; 
    }

    // 2. 比例项 P
    pid->POut = pid->Kp * pid->Error;

    // 3. 积分项 I (带积分分离和抗积分饱和)
    // 只有当误差在一个较小范围内（积分分离阈值）时，才累计积分项。偏差过大时仅靠 P 和 D
    if (pid->Error > -pid->I_Separation && pid->Error < pid->I_Separation) {
        pid->IOut += pid->Ki * pid->Error;
        pid->IOut = LIMIT_MIN_MAX(pid->IOut, -pid->MaxIOut, pid->MaxIOut);
    }

    // 4. 微分项 D
    pid->DOut = pid->Kd * (pid->Error - pid->LastError);

    // 5. 求和出总输出
    pid->Out = pid->POut + pid->IOut + pid->DOut;
    // 6. 对总输出进行限幅 （比如对于电机速度就是 +-100 ）
    pid->Out = LIMIT_MIN_MAX(pid->Out, -pid->MaxOut, pid->MaxOut);

    // 7. 记录本次偏差留作下次微分项计算
    pid->LastError = pid->Error;

    return pid->Out;
}
