#include "Motion.h"
#include "control.h"
#include "cmsis_os.h"
#include "Motor.h"
#include "IMU.h"
#include <math.h>


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

//
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

static uint8_t Motion_GreyHighCount(float grey_front, float grey_left, float grey_right, float grey_back)
{
    uint8_t count = 0;
    if (grey_front > 190.0f) count++;
    if (grey_left  > 190.0f) count++;
    if (grey_right > 190.0f) count++;
    if (grey_back  > 190.0f) count++;
    return count;
}

uint8_t Motion_IsEdgeRisk(uint16_t laser1, uint16_t laser2, float grey_front, float grey_left, float grey_right, float grey_back)
{
    uint8_t laser_edge = (laser1 > 260U || laser2 > 260U);
    uint8_t grey_edge = (Motion_GreyHighCount(grey_front, grey_left, grey_right, grey_back) >= 3U);
    return (laser_edge || grey_edge);
}

void Auto_Control_Logic_Laser(uint16_t laser1, uint16_t laser2, float grey_front, float grey_left, float grey_right, float grey_back)
{
    uint8_t grey_high_count = Motion_GreyHighCount(grey_front, grey_left, grey_right, grey_back);
    // 当激光传感器开始测得大于 250mm 时，说明小车已接近边缘，主动降速至 2；否则保持正常速度 18 巡台
    if (laser1 > 250 || laser2 > 250 || grey_high_count >= 2U)
    {
        Motor_Control(0, 2); 
    }
    else
    {
        Motor_Control(0, 18); 
    }

    // 去掉所有灰度传感器判断条件，仅根据激光测距判断边缘
    if(laser1 > 260 && laser2 <= 260)         
    {
        // 强力反向制动 80ms 快速消能，随后后退 220ms（后退总计 300ms）
        Motor_Control(0, -60);
        osDelay(80);
        Motor_Control(0, -22);                  
        osDelay(220);
        // 降低转弯速度（-60 -> -35），转向时间由 650ms 缩短至 450ms
        Motor_Control(-35, 0);                  
        osDelay(450);
        Motor_Control(0, 0);
        osDelay(125);
    }
    else if(laser1 <= 260 && laser2 > 260)
    {
        // 强力反向制动 80ms 快速消能，随后后退 220ms（后退总计 300ms）
        Motor_Control(0, -60);
        osDelay(80);
        Motor_Control(0, -22);
        osDelay(220);
        // 转向时间由 650ms 缩短至 450ms
        Motor_Control(35, 0);                   
        osDelay(450);
        Motor_Control(0, 0);
        osDelay(125);
    }
    else if(laser1 > 260 && laser2 > 260)
    {
        // 强力反向制动 80ms 快速消能，随后后退 220ms（后退总计 300ms）
        Motor_Control(0, -60);
        osDelay(80);
        Motor_Control(0, -22);
        osDelay(220);
        // 转向时间由 650ms 缩短至 450ms
        Motor_Control(-35, 0);                  
        osDelay(450);
        Motor_Control(0, 0);
        osDelay(125);
    }
    else if(grey_high_count >= 3U)
    {
        Motor_Control(0, -60);
        osDelay(80);
        Motor_Control(0, -22);
        osDelay(220);
        Motor_Control(-35, 0);
        osDelay(450);
        Motor_Control(0, 0);
        osDelay(125);
    }
}

// 自动推能量块函数（激光测距版）
// laser1 对应原本的 SW_L，laser2 对应原本 of SW_R
// 当激光测量距离大于 260 mm 时说明能量块已被推下（下方悬空）
void Detect_Laser(volatile uint8_t *target, volatile float *yaw, volatile float *distance_front, volatile uint16_t *laser1, volatile uint16_t *laser2,
                  volatile float *grey_front, volatile float *grey_left, volatile float *grey_right, volatile float *grey_back)
{
    if(*target == 2 || *target == 0) //如果视觉识别到的目标是tag_type=2或tag_type=0，则根据偏航角进行转向修正
    {
        // 调整姿态，使小车对准能量块（降低修正转向速度：由 50 降至 35）
        if(*yaw > 7)
        {
            // 往左转进行修正，非阻塞（交由大循环判断）
            Motor_Control(35, 0);
            osDelay(150); // 适当延时让转向生效
            Motor_Control(0, 0); // 立即停止转向，保持当前位置
            osDelay(150); // 适当延时让转向生效
        }
        else if( *yaw < -7)
        {
            // 往右转进行修正，非阻塞
            Motor_Control(-35, 0); 
            osDelay(150); // 适当延时让转向生效
            Motor_Control(0, 0); // 立即停止转向，保持当前位置
            osDelay(150); // 适当延时让转向生效
        }
        else if(*yaw >= -7 && *yaw <= 7) // 如果已经对准了
        {  
            // 已经对准，执行推块动作（降低速度：由 25 降至 18）
            Motor_Control(0, 18); // 向前推
            uint32_t push_start_time = HAL_GetTick(); // 记录推块开始时间
            
            // 去掉灰度判断，只靠激光大于 260 触发
            while(!Motion_IsEdgeRisk(*laser1, *laser2, *grey_front, *grey_left, *grey_right, *grey_back))
            {
                osDelay(50);
                // 添加一个10秒超时机制，防止死循环
                if(HAL_GetTick() - push_start_time >= 10000) 
                {
                    break;
                }
            }
            // 降低推完后的停止/后退速度（由 -40 降至 -25）
            Motor_Control(0, -25); // 推完后立即停止
            osDelay(130); // 推完后稍微延时一下
            Motor_Control(0, 0); // 确保完全停止
            osDelay(250); // 适当延时让动作生效
        }
    }

    if(*target == 1) // 如果视觉识别到的目标是tag_type=1（自己的能量块需避让）
    {
        // 调整姿态，使小车对准能量块（降低转向速度：由 50 降至 35）
        if(*yaw >= 0 && *yaw <= 20)
        {
            // 往左转进行修正，非阻塞（交由大循环判断）
            Motor_Control(-35, 0);
            osDelay(150); // 适当延时让转向生效
            Motor_Control(0, 0); // 立即停止转向，保持当前位置
            osDelay(150); // 适当延时让转向生效
        }
        else if(*yaw < 0 && *yaw >= -20)
        {
            // 往右转进行修正，非阻塞
            Motor_Control(35, 0); 
            osDelay(150); // 适当延时让转向生效
            Motor_Control(0, 0); // 立即停止转向，保持当前位置
            osDelay(150); // 适当延时让转向生效
        }
    }
}
