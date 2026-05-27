#include "Motion.h"
#include "control.h"
#include "cmsis_os.h"
#include "Motor.h"


//自动巡台函数，传入左右两个光电开关和前方灰度传感器的数值，根据不同的情况执行不同的转向逻辑
void Auto_Control_Logic(uint8_t sw1, uint8_t sw3, float grey_front,float grey_left,float grey_right,float grey_back)
{
    Motor_Control(0, 25); 

    if(sw1 == 1 && sw3 == 0 &&( grey_front > 145|| grey_right > 145)) //如果前方灰度传感器数值异常（可能是掉线了），则认为没有检测到白线，执行原有的巡台逻辑         
    {
        Motor_Control(0, -35);                  
        osDelay(600);
        Motor_Control(-60, 0);                  
        osDelay(650);
        Motor_Control(0, 0);
        osDelay(125);
    }
    else if(sw1 == 0 && sw3 == 1 &&( grey_front > 145|| grey_left > 145))
    {
        Motor_Control(0, -35);
        osDelay(600);
        Motor_Control(60, 0);                   
        osDelay(650);
        Motor_Control(0, 0);
        osDelay(125);
    }
    else if(sw1 == 1 && sw3 == 1 && grey_front > 145)
    {
        Motor_Control(0, -35);
        osDelay(600);
        Motor_Control(-60, 0);                  
        osDelay(650);
        Motor_Control(0, 0);
        osDelay(125);
    }
}

//自动推能量块函数
void Detect(volatile uint8_t *target, volatile float *yaw,volatile float *distance_front,volatile uint8_t* SW_L,volatile uint8_t* SW_R,volatile float* grey_front)
{
    if(*target == 2|| *target == 0) //如果视觉识别到的目标是tag_type=2（能量块）或tag_type=3（能量块），则根据偏航角进行转向修正
    {
        //调整姿态，使小车对准能量块
        if(*yaw > 7)
        {
            // 往左转进行修正，非阻塞（交由大循环判断）
            Motor_Control(50, 0);
            osDelay(150); // 适当延时让转向生效
            Motor_Control(0, 0); // 立即停止转向，保持当前位置
            osDelay(150); // 适当延时让转向生效
        }
        else if( *yaw < -7)
        {
            // 往右转进行修正，非阻塞
            Motor_Control(-50, 0); 
            osDelay(150); // 适当延时让转向生效
            Motor_Control(0, 0); // 立即停止转向，保持当前位置
            osDelay(150); // 适当延时让转向生效
        }
        else if(*yaw >= -7 && *yaw <= 7) //如果已经对准了
        {  
            // 已经对准，执行推块动作
            Motor_Control(0,25); // 向前推
            uint32_t push_start_time = HAL_GetTick(); // 记录推块开始时间
            
            while(!((*SW_L == 1 || *SW_R == 1)&& *grey_front > 165)) //持续推，直到至少一个光电开关被触发（检测到能量块被推下）
            {
                osDelay(50);
                // 添加一个10秒超时机制，防止死循环
                if(HAL_GetTick() - push_start_time >= 10000) 
                {
                    break;
                }
            }
            Motor_Control(0, -40); // 推完后立即停止
            osDelay(130); // 推完后稍微延时一下
            Motor_Control(0, 0); // 确保完全停止
            osDelay(250); // 适当延时让动作生效
        }
    }

    if(*target == 1) //如果视觉识别到的目标是tag_type=1（自己的能量块需避让）
    {
        //调整姿态，使小车对准能量块
        if(*yaw >=0&& *yaw <= 20)
        {
            // 往左转进行修正，非阻塞（交由大循环判断）
            Motor_Control(-50, 0);
            osDelay(150); // 适当延时让转向生效
            Motor_Control(0, 0); // 立即停止转向，保持当前位置
            osDelay(150); // 适当延时让转向生效
        }
        else if(*yaw <0 && *yaw >= -20)
        {
            // 往右转进行修正，非阻塞
            Motor_Control(50, 0); 
            osDelay(150); // 适当延时让转向生效
            Motor_Control(0, 0); // 立即停止转向，保持当前位置
            osDelay(150); // 适当延时让转向生效
        }
    }


}
