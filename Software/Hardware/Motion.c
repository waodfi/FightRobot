#include "Motion.h"
#include "control.h"
#include "cmsis_os.h"
#include "Motor.h"
#include "IMU.h"
#include "config.h"
#include <math.h>
#include "config.h"

extern float grey_value[4];



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

#define EDGE_GREY_HIGH_THRESHOLD 190.0f
#define EDGE_GREY_STRONG_THRESHOLD 230.0f
#define EDGE_LASER_SLOW_MM       250U
#define EDGE_LASER_TRIGGER_MM    260U

static uint8_t Motion_GreyHighCount(float grey_front, float grey_left, float grey_right, float grey_back)
{
    uint8_t count = 0;
    if (grey_front > EDGE_GREY_HIGH_THRESHOLD) count++;
    if (grey_left  > EDGE_GREY_HIGH_THRESHOLD) count++;
    if (grey_right > EDGE_GREY_HIGH_THRESHOLD) count++;
    if (grey_back  > EDGE_GREY_HIGH_THRESHOLD) count++;
    return count;
}

static void Motion_BackAwayWithRearGuard(int16_t speed, uint32_t delay_ms)
{
    Motor_Control(0, speed);
    uint32_t start_tick = HAL_GetTick();
    while (HAL_GetTick() - start_tick < delay_ms)
    {
        if (Grey_Back > 250.0f)
        {
            break;
        }
        osDelay(10);
    }
}

uint8_t Motion_IsEdgeRisk(uint16_t laser1, uint16_t laser2, float grey_front)
{
    uint8_t laser_edge = (laser1 > 260U || laser2 > 260U);
    uint8_t grey_edge = (grey_front > 250.0f);
    return (laser_edge || grey_edge);
}

EdgeDir_e Motion_GetEdgeDir(uint16_t laser1, uint16_t laser2, float grey_front, float grey_left, float grey_right, float grey_back)
{
    uint8_t front_high = (grey_front > EDGE_GREY_HIGH_THRESHOLD);
    uint8_t left_high  = (grey_left  > EDGE_GREY_HIGH_THRESHOLD);
    uint8_t right_high = (grey_right > EDGE_GREY_HIGH_THRESHOLD);
    uint8_t back_high  = (grey_back  > EDGE_GREY_HIGH_THRESHOLD);
    uint8_t front_strong = (grey_front > EDGE_GREY_STRONG_THRESHOLD);
    uint8_t left_strong  = (grey_left  > EDGE_GREY_STRONG_THRESHOLD);
    uint8_t right_strong = (grey_right > EDGE_GREY_STRONG_THRESHOLD);
    uint8_t back_strong  = (grey_back  > EDGE_GREY_STRONG_THRESHOLD);
    uint8_t grey_strong = (front_strong || left_strong || right_strong || back_strong);
    uint8_t high_count = Motion_GreyHighCount(grey_front, grey_left, grey_right, grey_back);
    uint8_t laser_edge = (laser1 > EDGE_LASER_TRIGGER_MM || laser2 > EDGE_LASER_TRIGGER_MM);

    if (!laser_edge && high_count < 3U && !grey_strong)
    {
        return EDGE_DIR_NONE;
    }

    if (high_count >= 3U)
    {
        return EDGE_DIR_UNKNOWN;
    }

    if (laser_edge || front_strong || front_high)
    {
        return EDGE_DIR_FRONT;
    }

    if (back_strong || back_high)
    {
        return EDGE_DIR_BACK;
    }

    if ((left_strong || left_high) && !right_high)
    {
        return EDGE_DIR_LEFT;
    }

    if ((right_strong || right_high) && !left_high)
    {
        return EDGE_DIR_RIGHT;
    }

    return EDGE_DIR_UNKNOWN;
}

void Motion_EscapeEdge(uint16_t laser1, uint16_t laser2, float grey_front, float grey_left, float grey_right, float grey_back)
{
    EdgeDir_e edge_dir = Motion_GetEdgeDir(laser1, laser2, grey_front, grey_left, grey_right, grey_back);
    int16_t turn_speed = -35;

    if (edge_dir == EDGE_DIR_NONE)
    {
        return;
    }

    Motor_Control(0, 0);
    osDelay(40);

    switch (edge_dir)
    {
        case EDGE_DIR_FRONT:
            Motor_Control(0, -60);
            osDelay(80);
            Motion_BackAwayWithRearGuard(-18, 150U);

            if (laser1 > EDGE_LASER_TRIGGER_MM && laser2 <= EDGE_LASER_TRIGGER_MM)
            {
                turn_speed = -35;
            }
            else if (laser1 <= EDGE_LASER_TRIGGER_MM && laser2 > EDGE_LASER_TRIGGER_MM)
            {
                turn_speed = 35;
            }
            else
            {
                turn_speed = -35;
            }
            Motor_Control(turn_speed, 0);
            osDelay(450);
            break;

        case EDGE_DIR_BACK:
            Motor_Control(0, 28);
            osDelay(260);
            Motor_Control(-35, 0);
            osDelay(350);
            break;

        case EDGE_DIR_LEFT:
            Motor_Control(-35, 0);
            osDelay(450);
            Motor_Control(0, 16);
            osDelay(180);
            break;

        case EDGE_DIR_RIGHT:
            Motor_Control(35, 0);
            osDelay(450);
            Motor_Control(0, 16);
            osDelay(180);
            break;

        case EDGE_DIR_UNKNOWN:
        default:
            Motor_Control(0, 0);
            osDelay(80);
            Motor_Control(-35, 0);
            osDelay(550);
            Motor_Control(0, 14);
            osDelay(180);
            break;
    }

    Motor_Control(0, 0);
    osDelay(125);
}

void Auto_Control_Logic_Laser(uint16_t laser1, uint16_t laser2, float grey_front, float grey_left, float grey_right, float grey_back)
{
    // 当激光传感器开始测得大于 250mm 时，或者前灰度接近边缘（> 240.0f），主动降速至 2；否则保持正常速度 18 巡台
    if (laser1 > 250 || laser2 > 250 || grey_front > 240.0f)
    {
        Motor_Control(0, 2); 
    }
    else
    {
        Motor_Control(0, 18); 
    }

    if(laser1 > 260 && laser2 <= 260)         
    {
        // 强力反向制动 80ms 快速消能，随后后退 150ms（后退中若后灰度检测到边缘则停止）
        Motor_Control(0, -60);
        osDelay(80);
        Motor_Control(0, -18);                  
        for (int i = 0; i < 15; i++) {
            if (Grey_Back > 250.0f) break;
            osDelay(10);
        }
        // 降低转弯速度（-60 -> -35），转向时间由 650ms 缩短至 450ms
        Motor_Control(-35, 0);                  
        osDelay(450);
        Motor_Control(0, 0);
        osDelay(125);
    }
    else if(laser1 <= 260 && laser2 > 260)
    {
        // 强力反向制动 80ms 快速消能，随后后退 150ms（后退中若后灰度检测到边缘则停止）
        Motor_Control(0, -60);
        osDelay(80);
        Motor_Control(0, -18);
        for (int i = 0; i < 15; i++) {
            if (Grey_Back > 250.0f) break;
            osDelay(10);
        }
        // 转向时间由 650ms 缩短至 450ms
        Motor_Control(35, 0);                   
        osDelay(450);
        Motor_Control(0, 0);
        osDelay(125);
    }
    else if((laser1 > 260 && laser2 > 260) || (grey_front > 250.0f))
    {
        // 双侧压线或正前方灰度压线：强力反向制动 80ms 快速消能，随后后退 150ms（后退中若后灰度检测到边缘则停止）
        Motor_Control(0, -60);
        osDelay(80);
        Motor_Control(0, -18);
        for (int i = 0; i < 15; i++) {
            if (Grey_Back > 250.0f) break;
            osDelay(10);
        }
        // 转向时间由 650ms 缩短至 450ms
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
            
            // 仅靠激光和前灰度判定边缘风险
            while(!Motion_IsEdgeRisk(*laser1, *laser2, *grey_front))
            {
                osDelay(50);
                // 添加一个10秒超时机制，防止死循环
                if(HAL_GetTick() - push_start_time >= 10000) 
                {
                    break;
                }
            }
            // 降低推完后的停止/后退速度（由 -25 降至 -18），并实时监测后灰度防跌落
            Motor_Control(0, -18); 
            for (int i = 0; i < 13; i++) {
                if (Grey_Back > 250.0f) break;
                osDelay(10);
            }
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
