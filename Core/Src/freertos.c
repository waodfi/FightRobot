/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IMU.h"
#include <stdio.h>
#include "Motor.h"
#include "IR.h"
#include "Grey.h"
#include "tim.h"  // 包含定时器头文件
#include "PID.h"
#include "IR_Sensor.h"
#include "control.h"
#include "Servo.h"
#include "Motion.h"
#include "Machine_Vision.h"
#include "SoftI2C.h"
#include "TOF050C.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#include "config.h"
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint32_t imu_rx_byte_count;

volatile uint8_t global_vision_target = 0;   // 视觉传来的实时目标 tag_type
volatile float global_vision_yaw = 0.0f;     // 视觉传来的实时偏航角 yaw

extern uint16_t adc_raw_data[13];
extern float grey_value[4];
extern float ir_distance[9];

extern uint32_t motor_pulse_diff[4];
extern uint32_t motor_pulse_count[4];

extern uint8_t sw1, sw2, sw3, sw4; // 4路光电开关状态

float motor_rpm[4] = {0.0f};       // 保存4个电机的实时转速 (RPM)
float motor_rpm_norm[4] = {0.0f};  // 归一化后的转速 (-100 ~ 100)
PID_t motor_pid[4];                // 4个电机的PID控制结构体
float motor_target_speed[4] = {0.0f, 0.0f,0.0f, 0.0f}; // 目标转速
//一号轮右前轮，二号轮左前轮，三号轮右后轮，四号轮左后轮


PID_t Angle_pid[1];                   // 姿�?�控制PID结构体（单�?�道，控制机器人绕垂直轴的旋转）
float target_angle = 0.0f;            // 姿�?�控制的目标角度（单位：度，范围 -180 ~ 180）
float current_angle = 0.0f;           // 姿�?�控制的当前角度（单位：度，范围 -180 ~ 180）

/* 运动模式控制 */
uint8_t control_mode = 0;             // 0=遥控模式, 1=自动控制模式
uint8_t angle_control_enabled = 0;    // 0=禁用角度环, 1=启用角度环
uint8_t angle_control_prev = 0;       // 上一个周期的角度环启用状态，用于检测状态变化
uint32_t last_buttons = 0;            // 记录上一次的 button 值，用于检测按钮状态变化
/* 摇杆和角度环的分离控制 */
float motor_rc_speed[4] = {0.0f};     // 摇杆基础速度指令 (前后/左右)
float motor_angle_speed[4] = {0.0f};  // 角度环转向修正指令

/* ===== 自主登台与软启动参数配置（可根据实测微调） ===== */
#define SOFT_START_CONFIRM_CNT     5       /* 连续确认次数(防抖)，5次 x 20ms = 100ms */
#define CLIMB_SPEED                (-100)  /* 后退冲台的速度(-100为全速) */
#define CLIMB_LEFT_SPEED_LIMIT     (-100)  /* 左侧轮组(1,3)冲台速度 (-100为满速，可微调以纠正物理不对称导致的歪斜) */
#define CLIMB_RIGHT_SPEED_LIMIT    (-70)  /* 右侧轮组(2,4)冲台速度 (-100为满速，可微调以纠正物理不对称导致的歪斜) */
#define GREY_GRADUAL_FILTER_CNT    6       /* 连续采样次数，6 x 20ms = 120ms */

#define ONSTAGE_GREY_SUCCESS_THRESHOLD 140.0f /* 台上灰度成功阈值，小于此值代表已成功上台 */
#define DIST_CLOSE_THRESHOLD       30.0f   /* 判定靠墙/边缘距离(cm) */
#define DIST_OPEN_THRESHOLD        100.0f  /* 判定开阔距离(cm) */
#define CORNER_OPEN_THRESHOLD      80.0f   /* 角落检测的开阔判定距离(cm) */
#define TOP_IR_THRESHOLD           60.0f   /* 顶部红外发射检测擂台台面开阔的距离(cm) */

#define SCAN_ROTATE_SPEED          15      /* 台下原地缓慢自转扫描的速度 */
#define SQUARING_SPEED             45      /* 向前顶墙对齐的速度 */
#define SQUARING_DURATION_MS       1200    /* 前进顶墙时间为固定的1.2秒(1200ms) */
#define CORNER_MOVE_MS             600     /* 角落避障开环移动时间(ms) */
#define HEADING_KP                 1.5f    /* 锁角倒退的闭环比例纠偏系数 */
#define HEADING_KD                 0.05f   /* 锁角倒退的闭环微分纠偏系数 */

#define CLIMB_ONSTAGE_DURATION_MS  2000    /* 台上冲坡持续时间(2秒) */
#define CLIMB_BLIND_DURATION_MS    1000    /* 登台盲冲刺阶段时间为1.0秒(1000ms)，期间忽略灰度传感器 */
#define CLIMB_OFFSTAGE_TIMEOUT_MS  3500    /* 登台后退最大安全保护时间为 3.5 秒 (3500ms) */

/* 自主登台与软启动状态枚举 */
typedef enum {
    ROBOT_WAITING           = 0,   /* 等待中：等待用户用手遮挡左右红外 */
    ROBOT_ARMED             = 1,   /* 已就绪：等待用户松开双手 */
    ROBOT_LAUNCH_DECIDE     = 2,   /* 启动决策：自动读取底盘判定台上/台下 */
    ROBOT_CLIMBING_ONSTAGE  = 3,   /* 台上启动：直接静止不动 */
    ROBOT_SCANNING          = 4,   /* 台下扫描：无论如何先自转扫掠 */
    ROBOT_CORNER_ESCAPE     = 5,   /* 角落脱困：自转中发现角落后避障 + 90°对齐 */
    ROBOT_SQUARING          = 6,   /* 物理顶墙：向前以 SQUARING_SPEED 顶墙，并重置IMU Yaw轴 */
    ROBOT_CLIMBING_OFFSTAGE = 7,   /* 闭环冲台：向后倒车 + PD纠偏直走登台 */
    ROBOT_FINISHED          = 8,   /* 登台/测试完成：保持绝对静止 */
    ROBOT_RUNNING           = 9,   /* 运行状态：常规模式 */
    ROBOT_TEST_ROTATE_90    = 10,  /* 测试状态：输入Y后，小车逆时针连续转动90° */
    ROBOT_TEST_ROTATE_PREPARE = 11, /* 准备状态：输入Y后，在任务中安全重置偏航角并等待 */
    ROBOT_CLIMB_ROTATE_TO_DIR = 12, /* 登台状态：转动到指定方向 */
    ROBOT_CLIMB_FORWARD_UNTIL_UP_IR = 13, /* 登台状态：向前直走直到上方红外大于110 */
    ROBOT_CLIMB_FORWARD_UNTIL_WALL = 14,  /* 登台状态：向前直走直到满足红外壁障条件 */
    ROBOT_CLIMB_FORWARD_DELAY = 15,       /* 登台状态：前行延时1秒 */
    ROBOT_CLIMB_BACKWARD_FAST = 16,       /* 登台状态：全速向后冲台1秒 */
    ROBOT_TEST_ATTACK_PREPARE = 17,       /* 测试状态：自主识别并进攻准备 */
    ROBOT_TEST_ATTACK_RUNNING = 18,       /* 测试状态：自主识别并进攻识别中心 */
    ROBOT_TEST_ATTACK_MOVE    = 19,       /* 测试状态：自主识别并进攻后冲刺0.5S */
    ROBOT_FIGHT_ATTACK_MOVE   = 20,       /* 实战状态：追踪敌人直到撞边缘 */
    ROBOT_INIT_CLIMB          = 21,       /* 初始启动直冲登台阶段 */
    ROBOT_ALIGN_ORTHO         = 22        /* 掉台就近90度倍数方向对齐 */
} RobotState_e;

volatile RobotState_e robot_state = ROBOT_WAITING;  /* 状态机当前状态 */
static uint8_t  soft_start_confirm   = 0;  /* 遮挡确认防抖计数器 */
static uint8_t  soft_release_confirm = 0;  /* 松开确认防抖计数器 */
static uint32_t climb_start_tick     = 0;  /* 记录冲台开始的时间 */

/* 脱困/顶墙过程辅助变量 */
static uint8_t  corner_type          = 0;  /* 1:后左(B+L), 2:后右(B+R) */
static uint8_t  corner_phase         = 0;  /* 脱困阶段：0:移动阶段, 1:转向90°阶段 */
static uint32_t corner_start_tick    = 0;  /* 脱困段计时 */
static float    target_angle_lock    = 0.0f; /* 自主控制锁定航向角 */
static uint8_t  climb_pd_init        = 0;  /* 登台 PD 控制器初始化标志 */
static float    last_yaw_err         = 0.0f; /* 上一次偏航角偏差 */
static uint32_t squaring_start_tick  = 0;  /* 顶墙开始时间 */
static uint8_t  is_test_mode         = 0;  /* 是否为串口 'Y' 触发的 270° 测试模式 */

/* 软启动与掉台后登台状态监控变量 */
static uint8_t  climb_imu_tilted      = 0;  /* 登台过程是否检测到过倾斜 */
static uint8_t  climb_imu_prelim_success = 0; /* IMU初步判定成功登台标志 */
static uint16_t grey_gradual_cnt[4]   = {0}; /* 4路灰度渐变变化计数器 */
static uint8_t  grey_gradual_success  = 0;  /* 灰度传感器确认登台成功 */
static uint8_t  onstage_confirmed     = 0;  /* 台上确认状态：0:未确认, 1:已确认 */
static uint32_t onstage_confirm_timer = 0;  /* 台上确认计时器 */
static uint8_t  fall_imu_tilted       = 0;  /* 掉台过程倾斜检测标志 */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Motor_Task */
osThreadId_t Motor_TaskHandle;
const osThreadAttr_t Motor_Task_attributes = {
  .name = "Motor_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Comm_Task */
osThreadId_t Comm_TaskHandle;
const osThreadAttr_t Comm_Task_attributes = {
  .name = "Comm_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Sensor_Task */
osThreadId_t Sensor_TaskHandle;
const osThreadAttr_t Sensor_Task_attributes = {
  .name = "Sensor_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Vision_Task */
osThreadId_t Vision_TaskHandle;
const osThreadAttr_t Vision_Task_attributes = {
  .name = "Vision_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Motion_Task */
osThreadId_t Motion_TaskHandle;
const osThreadAttr_t Motion_Task_attributes = {
  .name = "Motion_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Angle_Task */
osThreadId_t Angle_TaskHandle;
const osThreadAttr_t Angle_Task_attributes = {
  .name = "Angle_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for UART1_Rx_Queue */
osMessageQueueId_t UART1_Rx_QueueHandle;
const osMessageQueueAttr_t UART1_Rx_Queue_attributes = {
  .name = "UART1_Rx_Queue"
};
/* Definitions for IMU_Rx_Queue */
osMessageQueueId_t IMU_Rx_QueueHandle;
const osMessageQueueAttr_t IMU_Rx_Queue_attributes = {
  .name = "IMU_Rx_Queue"
};
/* Definitions for Vision_Rx_Queue */
osMessageQueueId_t Vision_Rx_QueueHandle;
const osMessageQueueAttr_t Vision_Rx_Queue_attributes = {
  .name = "Vision_Rx_Queue"
};
/* Definitions for Sem_IR_1 */
osSemaphoreId_t Sem_IR_1Handle;
const osSemaphoreAttr_t Sem_IR_1_attributes = {
  .name = "Sem_IR_1"
};
/* Definitions for Sem_IR_2 */
osSemaphoreId_t Sem_IR_2Handle;
const osSemaphoreAttr_t Sem_IR_2_attributes = {
  .name = "Sem_IR_2"
};
/* Definitions for Sem_IR_3 */
osSemaphoreId_t Sem_IR_3Handle;
const osSemaphoreAttr_t Sem_IR_3_attributes = {
  .name = "Sem_IR_3"
};
/* Definitions for Sem_IR_4 */
osSemaphoreId_t Sem_IR_4Handle;
const osSemaphoreAttr_t Sem_IR_4_attributes = {
  .name = "Sem_IR_4"
};
/* Definitions for IR_EventGroup */
osEventFlagsId_t IR_EventGroupHandle;
const osEventFlagsAttr_t IR_EventGroup_attributes = {
  .name = "IR_EventGroup"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMotor_Task(void *argument);
void StartComm_Task(void *argument);
void StartSensor_Task(void *argument);
void StartVision_Task(void *argument);
void StartMotion_Task(void *argument);
void StartAngle_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Sem_IR_1 */
  Sem_IR_1Handle = osSemaphoreNew(1, 1, &Sem_IR_1_attributes);

  /* creation of Sem_IR_2 */
  Sem_IR_2Handle = osSemaphoreNew(1, 1, &Sem_IR_2_attributes);

  /* creation of Sem_IR_3 */
  Sem_IR_3Handle = osSemaphoreNew(1, 1, &Sem_IR_3_attributes);

  /* creation of Sem_IR_4 */
  Sem_IR_4Handle = osSemaphoreNew(1, 1, &Sem_IR_4_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UART1_Rx_Queue */
  UART1_Rx_QueueHandle = osMessageQueueNew (256, sizeof(uint8_t), &UART1_Rx_Queue_attributes);

  /* creation of IMU_Rx_Queue */
  IMU_Rx_QueueHandle = osMessageQueueNew (16, sizeof(uint32_t), &IMU_Rx_Queue_attributes);

  /* creation of Vision_Rx_Queue */
  Vision_Rx_QueueHandle = osMessageQueueNew (16, sizeof(uint32_t), &Vision_Rx_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Motor_Task */
  Motor_TaskHandle = osThreadNew(StartMotor_Task, NULL, &Motor_Task_attributes);

  /* creation of Comm_Task */
  Comm_TaskHandle = osThreadNew(StartComm_Task, NULL, &Comm_Task_attributes);

  /* creation of Sensor_Task */
  Sensor_TaskHandle = osThreadNew(StartSensor_Task, NULL, &Sensor_Task_attributes);

  /* creation of Vision_Task */
  Vision_TaskHandle = osThreadNew(StartVision_Task, NULL, &Vision_Task_attributes);

  /* creation of Motion_Task */
  Motion_TaskHandle = osThreadNew(StartMotion_Task, NULL, &Motion_Task_attributes);

  /* creation of Angle_Task */
  Angle_TaskHandle = osThreadNew(StartAngle_Task, NULL, &Angle_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of IR_EventGroup */
  IR_EventGroupHandle = osEventFlagsNew(&IR_EventGroup_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {

    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMotor_Task */
/**
* @brief Function implementing the Motor_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotor_Task */
void StartMotor_Task(void *argument)
{
  /* USER CODE BEGIN StartMotor_Task */
  
  /* 启动定时器PWM输出(依据Motor.c映射TIM8的1,2,3,4通道) */
  extern TIM_HandleTypeDef htim8;
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

  /* 在任务初始化中开启TIM2?????4个?道捕获中断 */
HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

  /* PID 初始化 (电机速度控制) 
     增加: 误差绝对值死区阈值(如 2.0f)，积分分离阈值(如 30.0f)
     当绝对?度离目标超过 30 时切断积分防超调；相差 2 以内视为到达目标平息抖动 */
  for(int i = 0; i < 4; i++) {
      // (pid, Kp, Ki, Kd, MaxOut, MaxIOut, DeadBand, I_Separation)
      // 积分分离阈值 500.0f：防止启动时积分饱和；误差 < 500 时 P 项可能不足，积分逐步参与
      // 纯 P 控制(Kp=0.65)可能难以达到目标，积分项帮助消除稳?误???
      PID_Init(&motor_pid[i], 0.65f, 0.20f, 0.02f, 100.0f, 100.0f, 1.0f, 500.0f); 
  }

  uint32_t last_pulse_count[4] = {0};
  float filtered_speed[4] = {0.0f}; // 新增：用于平滑测速跳动的低?滤波数???
  uint32_t motor_print_tick = 0;

  /* Infinite loop */
  for(;;)
  {    /* 融合摇杆速度和角度环转向修正 */
    for (int i = 0; i < 4; i++) {
      motor_target_speed[i] = motor_rc_speed[i] + motor_angle_speed[i];
      /* 限制在 -100 ~ 100 范围内 */
      if (motor_target_speed[i] > 100.0f) motor_target_speed[i] = 100.0f;
      if (motor_target_speed[i] < -100.0f) motor_target_speed[i] = -100.0f;
    }
        /* 计算并更新转速（50ms周期） */
    for (int i = 0; i < 4; i++)
    {
      uint32_t current_count = motor_pulse_count[i];
      uint32_t delta = current_count - last_pulse_count[i];
      last_pulse_count[i] = current_count;

      /* 1. 计算原始 RPM (仅为当前周期捕获的脚冲数产生的转速) */
      motor_rpm[i] = (float)delta * 133.333333f;

      /* 2. 归一化到绝对速度的模 0 ~ 100 */
      float current_abs_speed = (motor_rpm[i] / 6400.0f) * 100.0f;
      if (current_abs_speed > 100.0f) current_abs_speed = 100.0f;

      /* 引入一阶低通滤波 (Exponential Moving Average Filter)*/

      filtered_speed[i] = 0.6f * filtered_speed[i] + 0.4f * current_abs_speed;

      float output = 0.0f;
      
      /* 当目标转速为0时，直接输出0，并清空PID，防止编码器无方向时死区闭环发散 */
      if (motor_target_speed[i] == 0.0f) {
          motor_rpm_norm[i] = 0.0f;
          filtered_speed[i] = 0.0f; // 滤波清零，防止再度起步时毛刺
          PID_Clear(&motor_pid[i]); // 清除PID积分状态
          output = 0.0f;
      } else {
          /* 我们依据目标速度的方向(正反)，为滤后的绝对转速加上符号 */ 
          if (motor_target_speed[i] < 0.0f) {
               motor_rpm_norm[i] = -filtered_speed[i];
          } else {
               motor_rpm_norm[i] = filtered_speed[i];
          }

          /* 3. PID 计算得出控制输出 [-100, 100] */
          if (robot_state == ROBOT_INIT_CLIMB) {
              if (i == 0 || i == 2) {
                  output = CLIMB_LEFT_SPEED_LIMIT;
              } else {
                  output = CLIMB_RIGHT_SPEED_LIMIT;
              }
              PID_Clear(&motor_pid[i]);
          } else {
              output = PID_Calc(&motor_pid[i], motor_target_speed[i], motor_rpm_norm[i]);
          }
      }

      /* 4. 更新电机 PWM 输出 (-100~100) 
            下层 Motor_SetSpeed 会拆分符号控制反转引脚，和大小控制PWM */
      Motor_SetSpeed(MOTOR_1 + i, output); 
    }

        /* 降低打印频率至 1Hz，使串口更清爽 */
        if (HAL_GetTick() - motor_print_tick >= 1000U) {
          motor_print_tick = HAL_GetTick();
          printf("RPM: %.1f, %.1f, %.1f, %.1f\r\n",
            motor_rpm_norm[0], motor_rpm_norm[1], motor_rpm_norm[2], motor_rpm_norm[3]);
        }

    osDelay(50);
  }
  /* USER CODE END StartMotor_Task */
}

/* USER CODE BEGIN Header_StartComm_Task */
/**
* @brief Function implementing the Comm_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartComm_Task */
void StartComm_Task(void *argument)
{
  /* USER CODE BEGIN StartComm_Task */
  uint32_t msg;
  uint32_t comm_print_tick = 0;
  /* Infinite loop */
  for(;;)
  {
    /* 等待 Sensor_Task 每 100ms 发来的触发信号 */
    if (osMessageQueueGet(IMU_Rx_QueueHandle, &msg, NULL, osWaitForever) == osOK)
    {
      /* 将多行传感器输出降频至 1Hz，加快上位机显示，使串口更清爽 */
      if (HAL_GetTick() - comm_print_tick >= 1000U) {
        comm_print_tick = HAL_GetTick();
        printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, Temp: %.2f\r\n", IMU_Data.Roll, IMU_Data.Pitch, IMU_Data.Yaw, IMU_Data.Temp);
        printf("IR Dist: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\r\n",
               ir_distance[0], ir_distance[1], ir_distance[2],
               ir_distance[3], ir_distance[4], ir_distance[5],
               ir_distance[6], ir_distance[7], ir_distance[8]);
        printf("Grey(F,B,L,R): %.1f, %.1f, %.1f, %.1f\r\n",grey_value[0], grey_value[1], grey_value[2], grey_value[3]);

        printf("Photoelectric(SW1~SW4): %d, %d, %d, %d\r\n", sw1, sw2, sw3, sw4);
        printf("Laser(1,2): %d, %d mm\r\n", laser_dist_1, laser_dist_2);
      }
      
      //printf("Motor Pulses: %lu, %lu, %lu, %lu\r\n", 
      //        motor_pulse_count[0], motor_pulse_count[1], motor_pulse_count[2], motor_pulse_count[3]);
    }
  }
  /* USER CODE END StartComm_Task */
}

/* USER CODE BEGIN Header_StartSensor_Task */
/**
* @brief Function implementing the Sensor_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensor_Task */
void StartSensor_Task(void *argument)
{
  /* USER CODE BEGIN StartSensor_Task */
  uint32_t trigger_msg = 1;
  Grey_Init();
  IR_Init();
  
  // 初始化两路 TOF200C 激光测距传感器 (软件 I2C 总线)
  TOF050C_Init(SOFT_I2C_BUS_1);
  TOF050C_Init(SOFT_I2C_BUS_2);
  
  /* Infinite loop */
  for(;;)
  {
    /* 先计算前4路灰度值 */
    Grey_CalculateValues(&adc_raw_data[0], grey_value);

    /* 计算红外测距 */
    IR_CalculateDistances(&adc_raw_data[4], ir_distance);

    /* 读取两路激光测距 */
    laser_dist_1 = TOF050C_ReadDistance(SOFT_I2C_BUS_1);
    laser_dist_2 = TOF050C_ReadDistance(SOFT_I2C_BUS_2);

    /* 
     * 原红外光电引脚已被复用为激光软件 I2C 通信引脚。
     * 为避免 I2C 数据通信时的电平变化被误判为光电开关触发，
     * 禁用原物理引脚读取逻辑，将原光电状态变量恒设为 0。
     */
    sw1 = 0;
    sw2 = 0;
    sw3 = 0;
    sw4 = 0;

    /* 向队列发送触发标志给 Comm_Task */
    osMessageQueuePut(IMU_Rx_QueueHandle, &trigger_msg, 0, 0);
    
    /* 延迟10ms */
    osDelay(10);
  }
  /* USER CODE END StartSensor_Task */
}

/* USER CODE BEGIN Header_StartVision_Task */
/**
* @brief Function implementing the Vision_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVision_Task */
void StartVision_Task(void *argument)
{
  /* USER CODE BEGIN StartVision_Task */
  uint32_t msg;
  MV_ParsedFrame_t frame;
  MV_Stats_t stats;
  uint32_t vision_print_tick = 0;

  /* Infinite loop */
  for(;;)
  {
    (void)osMessageQueueGet(Vision_Rx_QueueHandle, &msg, NULL, 100);

    while (MachineVision_GetFrame(&frame) != 0U)
    {
      if (frame.type == MV_FRAME_VISION)
      {
        global_vision_target = frame.data.vision.tag_type;
        global_vision_yaw = frame.data.vision.yaw_cdeg / 100.0f;

        printf("VISION seq=%u tag_type=%u yaw=%.2f\r\n",
               frame.data.vision.seq,
               frame.data.vision.tag_type,
               frame.data.vision.yaw_cdeg / 100.0f);
      }
    }

    if (HAL_GetTick() - vision_print_tick >= 1000U) {
      vision_print_tick = HAL_GetTick();
      MachineVision_GetStats(&stats);
      //视觉解析统计部分
      /*
      printf("VISION_STAT rx=%lu ok_v=%lu crc_err=%lu fmt_err=%lu ovf=%lu\r\n",
             stats.total_rx_bytes,
             stats.parsed_vision_frames,
             stats.crc_error_frames,
             stats.format_error_frames,
             stats.ring_overflow_bytes);
      */
    }
  }
  /* USER CODE END StartVision_Task */
}

/* USER CODE BEGIN Header_StartMotion_Task */
/**
* @brief Function implementing the Motion_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotion_Task */
void StartMotion_Task(void *argument)
{
  /* USER CODE BEGIN StartMotion_Task */
  
  /* 初始化舵机 */
  Servo_Init();
  IMU_Init(); // 初始化IMU以接收数据
  osDelay(100);
  IMU_ZeroZAxis(); // 开启电源后，MPU6050清零
  osDelay(100);
  
  uint16_t last_servo_angle = 90;  /* 记录上一个有效的舵机角度 */
  const uint16_t servo_step = 2;    /* 固定步进角度：每个周期移动2度 */
  static uint8_t init_rotate_pd = 0; /* 用于连续转90°测试的 PD 控制器初始化标志 */
  static float last_yaw_err = 0.0f;   /* 上一次的偏航角误差，用于微分项计算 */
  static uint32_t climb_wall_align_start_tick = 0; /* 记录开始顶墙对齐的时间戳 */
  static uint8_t rotate_step = 0;    /* 记录完成了几次 90° 旋转 (0~3) */
  static float up_ir_values[4] = {0.0f}; /* 记录 4 个停顿位置的高位红外数值 */
  static float climb_target_angle = 0.0f; /* 登台阶段转向的目标角度 */
  static RobotState_e climb_next_state = ROBOT_FINISHED; /* 转向完成后的下一个状态 */
  static uint32_t climb_timer = 0;    /* 登台用计时器 */
  static int saved_D_smaller = 0;     /* 角落判定下，暂存数值较小的传感器方向 */

  /* 自适应软启动基准校准（防上电时手拿开的误差） */
  /* 给 Sensor_Task 充裕的通电稳定与采样等待时间，防止开机读取 0 盲值 */
  osDelay(1000); 
  
  float sum_L = 0.0f;
  float sum_R = 0.0f;
  const int sample_cnt = 20;
  for (int i = 0; i < sample_cnt; i++)
  {
    osDelay(50); // 50ms x 20 = 1000ms 动态校准采样
    sum_L += IR_Distance_L;
    sum_R += IR_Distance_R;
  }
  float avg_L = sum_L / sample_cnt;
  float avg_R = sum_R / sample_cnt;
  
  float L_block_threshold   = 40.0f;  /* 30cm 内判定为手遮挡 */
  float R_block_threshold   = 40.0f;
  float L_release_threshold = 40.0f;  /* 40cm 外判定为手移开 */
  float R_release_threshold = 40.0f;
  
  printf("[SoftStart] Calibrated baselines - L: avg=%.1f, block=%.1f, release=%.1f\r\n", avg_L, L_block_threshold, L_release_threshold);
  printf("[SoftStart] Calibrated baselines - R: avg=%.1f, block=%.1f, release=%.1f\r\n", avg_R, R_block_threshold, R_release_threshold);

  
  /* Infinite loop */
  for(;;)
  {
    /* 串口全程打印小车“在台上”或“在台下”，便于调试 */
    static uint32_t last_status_print_tick = 0;
    if (HAL_GetTick() - last_status_print_tick >= 500)
    {
      last_status_print_tick = HAL_GetTick();
      if (robot_state == ROBOT_RUNNING && onstage_confirmed == 1)
      {
        printf("[Status] 在台上\r\n");
      }
      else
      {
        printf("[Status] 在台下\r\n");
      }
    }

    /* 实时更新底盘灰度渐变检测滤波器 (历史样本与单调变化判定，区分突变) */
    static float grey_history[4][4] = {0};
    
    float g_sensors[4] = {Grey_Front, Grey_Back, Grey_Left, Grey_Right};
    for (int i = 0; i < 4; i++) {
      /* 滑动窗口更新历史记录 */
      grey_history[i][3] = grey_history[i][2];
      grey_history[i][2] = grey_history[i][1];
      grey_history[i][1] = grey_history[i][0];
      grey_history[i][0] = g_sensors[i];
      
      /* 检查 4 个历史样本是否全部落在 [50, 200] 区间内 */
      uint8_t in_range = 1;
      for (int k = 0; k < 4; k++) {
        if (grey_history[i][k] < 50.0f || grey_history[i][k] > 200.0f) {
          in_range = 0;
          break;
        }
      }
      
      if (in_range) {
        /* 计算连续差值 */
        float diff1 = grey_history[i][0] - grey_history[i][1];
        float diff2 = grey_history[i][1] - grey_history[i][2];
        float diff3 = grey_history[i][2] - grey_history[i][3];
        
        /* 判定是否为单调递增或单调递减，且变化幅度适中（防噪点突变，防静态漂移） */
        uint8_t is_gradual = 0;
        
        /* 1. 检查各步进差值在合理范围 [1.0, 45.0] 内 (过滤突变) */
        if (fabs(diff1) >= 1.0f && fabs(diff1) <= 45.0f &&
            fabs(diff2) >= 1.0f && fabs(diff2) <= 45.0f &&
            fabs(diff3) >= 1.0f && fabs(diff3) <= 45.0f)
        {
          /* 2. 检查方向单调性 (同号，确认是缓慢增大或减小) */
          if ((diff1 > 0.0f && diff2 > 0.0f && diff3 > 0.0f) ||
              (diff1 < 0.0f && diff2 < 0.0f && diff3 < 0.0f))
          {
            is_gradual = 1;
          }
        }
        
        if (is_gradual) {
          grey_gradual_success = 1;
        }
      }
    }
    
    /* 如果车身处于水平状态且处于巡台模式，清除渐变状态防止陈旧数据误触发 */
    static uint32_t flat_timer = 0;
    if (fabs(IMU_Data.Pitch) <= 12.0f && fabs(IMU_Data.Roll) <= 12.0f) {
      if (flat_timer == 0) {
        flat_timer = HAL_GetTick();
      } else if (HAL_GetTick() - flat_timer > 500) {
        if (robot_state == ROBOT_RUNNING && fall_imu_tilted == 0) {
          grey_gradual_success = 0;
          for (int i = 0; i < 4; i++) grey_gradual_cnt[i] = 0;
        }
      }
    } else {
      flat_timer = 0;
    }

    /* 紧急控制检测（在循环最前部运行，无视当前状态，确保开启自愈、软启动或爬台阶等任何时期均可急停） */
    if (Control_IsOnline())
    {
      PC_ControlData_t ctrl = Control_GetData();
      
      /* 紧急停机：按下遥控器 B 键 (buttons & 2) 立即停止所有电机并锁死在 FINISHED 状态 */
      if (ctrl.buttons & 2)
      {
        robot_state = ROBOT_FINISHED;
        Motor_Control(0, 0);
        printf("已停止\r\n");
        last_buttons = ctrl.buttons;
        osDelay(20);
        continue;
      }
      
      /* 恢复状态：按下遥控器 X 键 (buttons & 4) 解除停机状态并恢复正常运行 */
      if (ctrl.buttons & 4)
      {
        if (robot_state == ROBOT_FINISHED)
        {
          robot_state = ROBOT_RUNNING;
        }
        printf("已恢复\r\n");
        last_buttons = ctrl.buttons;
        osDelay(20);
        continue;
      }
    }

    /* 如果处于自动登台的各种准备或冲刺状态，则完全接管常规控制流，执行独立的高频状态机 */
    if (robot_state != ROBOT_RUNNING)
    {
      switch (robot_state)
      {
        case ROBOT_WAITING:
        {
          /* 等待状态：停止电机 */
          Motor_Control(0, 0);
          
          /* 调试打印：每500ms打印一次当前阻挡检测状态 */
          static uint32_t last_wait_print = 0;
          if (HAL_GetTick() - last_wait_print >= 500)
          {
            last_wait_print = HAL_GetTick();
            printf("[SoftStart] WAITING. L_dist=%.1f (Thres=%.1f), R_dist=%.1f (Thres=%.1f)\r\n", 
                   IR_Distance_L, L_block_threshold, IR_Distance_R, R_block_threshold);
          }
          
          /* 检测双手是否挡住左右红外传感器 */
          if (IR_Distance_L < L_block_threshold && IR_Distance_R < R_block_threshold)
          {
            soft_start_confirm++;
            if (soft_start_confirm >= SOFT_START_CONFIRM_CNT)
            {
              robot_state = ROBOT_ARMED;
              soft_start_confirm = 0;
              printf("[SoftStart] Ready! Release hands to launch. State -> ROBOT_ARMED\r\n");
            }
          }
          else
          {
            soft_start_confirm = 0;
          }
          break;
        }
        
        case ROBOT_ARMED:
        {
          /* 就绪状态：停止电机 */
          Motor_Control(0, 0);
          
          /* 调试打印：每500ms打印一次当前释放检测状态 */
          static uint32_t last_armed_print = 0;
          if (HAL_GetTick() - last_armed_print >= 500)
          {
            last_armed_print = HAL_GetTick();
            printf("[SoftStart] ARMED. L_dist=%.1f (Thres=%.1f), R_dist=%.1f (Thres=%.1f)\r\n", 
                   IR_Distance_L, L_release_threshold, IR_Distance_R, R_release_threshold);
            
            /* 在就绪状态下，定时清零 Z 轴，确保释放瞬间 Yaw 接近绝对 0° */
            IMU_ZeroZAxis();
          }
          
          /* 检测左右红外距离是否都大于自适应松开距离 */
          if (IR_Distance_L > L_release_threshold && IR_Distance_R > R_release_threshold)
          {
            soft_release_confirm++;
            if (soft_release_confirm >= SOFT_START_CONFIRM_CNT)
            {
              robot_state = ROBOT_INIT_CLIMB;
              soft_release_confirm = 0;
              climb_start_tick = HAL_GetTick();
              grey_gradual_success = 0;
              for(int i = 0; i < 4; i++) grey_gradual_cnt[i] = 0;
              climb_imu_tilted = 0;
              climb_imu_prelim_success = 0;
              target_angle_lock = 0.0f;          // 已经清零，目标设定为绝对 0.0°
              climb_pd_init = 0;                 // 重置 PD 控制器初始化标志
              printf("[SoftStart] Hands released! State -> ROBOT_INIT_CLIMB. Climbing up...\r\n");
            }
          }
          else
          {
            soft_release_confirm = 0;
          }
          break;
        }
        
        case ROBOT_INIT_CLIMB:
        {
          /* 初始登台冲刺：开环全速倒车，彻底禁用陀螺仪和速度渐变软启动，以获得最大初始扭矩和冲力 */
          Motor_Control(0, -100);
          
          /* 登台时间固定为 1.0 秒 (1000ms) */
          if (HAL_GetTick() - climb_start_tick >= 1000U)
          {
            /* 1S后立即刹车（闭环电机设为0速即为强力刹车） */
            Motor_Control(0, 0);
            osDelay(200); // 延时200ms以便让车体完全停稳消能
            
            /* 进入常规巡台 + 识别敌人模式 */
            onstage_confirmed = 1; // 标记确切在台上，启动巡逻
            control_mode = 1;      // 开启自动模式
            robot_state = ROBOT_RUNNING;
            
            printf("[SoftClimb] Initial climb 1.0s complete. Braked. Transition to ROBOT_RUNNING (onstage).\r\n");
          }
          break;
        }

        case ROBOT_ALIGN_ORTHO:
        {
          /* 掉台就近角度对齐：计算最近的 0°, 90°, -90°, 180° / -180° 并原位旋转对齐 */
          float yaw_err = IMU_Data.Yaw - target_angle_lock;
          while (yaw_err > 180.0f)  yaw_err -= 360.0f;
          while (yaw_err < -180.0f) yaw_err += 360.0f;
          
          if (fabs(yaw_err) < 3.0f)
          {
            /* 转向完成：停止电机，并切换到台下自动登台程序（ROBOT_SCANNING） */
            Motor_Control(0, 0);
            osDelay(200);
            robot_state = ROBOT_SCANNING;
            init_rotate_pd = 0;
            printf("[OrthoAlign] Target angle %.1f aligned. Initiating auto climb SCANNING...\r\n", target_angle_lock);
          }
          else
          {
            /* 转向 PD 控制 */
            if (init_rotate_pd == 0)
            {
              last_yaw_err = yaw_err;
              init_rotate_pd = 1;
            }
            float d_err = (yaw_err - last_yaw_err);
            last_yaw_err = yaw_err;
            
            float turn_speed = yaw_err * 0.8f + d_err * 7.5f;
            if (turn_speed > 18.0f)  turn_speed = 18.0f;
            if (turn_speed < -18.0f) turn_speed = -18.0f;
            
            if (fabs(yaw_err) > 5.0f)
            {
              if (turn_speed > 0.0f && turn_speed < 8.0f)  turn_speed = 8.0f;
              if (turn_speed < 0.0f && turn_speed > -8.0f) turn_speed = -8.0f;
            }
            Motor_Control((int16_t)turn_speed, 0);
          }
          break;
        }
        
        case ROBOT_LAUNCH_DECIDE:
        {
          is_test_mode = 0; /* 确认为官方正式登台流程 */
          /* 启动决策：利用 几何距离(不含前红外) + 底盘灰度 融合双重锁进行判定，达到 100% 绝对可靠性。
             - 台上特征：
               1. 灰度特征：底盘处于擂台之上，灰度反射率高，读数明显低于台下（擂台正中央及边缘灰度较低，任意底盘灰度传感器 < 180.0f）。
               2. 几何特征：由于擂台是 6cm 的凸起平台，水平测距传感器（左L、右R、后B）会射向空中直达围栏，读数非常开阔（至少有两侧 > 60.0cm）。
             - 台下特征：
               1. 灰度特征：身处纯黑地表通道，所有灰度传感器读数均较高（均 > 200.0f）。
               2. 几何特征：处于狭窄通道中（通道宽度仅约45cm），水平传感器（L、R、B）必然会有多侧受限，无法满足开阔条件。
          */
          uint8_t low_grey_count = 0;
          if (Grey_Front < 180.0f)  low_grey_count++;
          if (Grey_Back < 180.0f)   low_grey_count++;
          if (Grey_Left < 180.0f)   low_grey_count++;
          if (Grey_Right < 180.0f)  low_grey_count++;

          uint8_t open_sides = 0;
          if (IR_Distance_L > 60.0f) open_sides++;
          if (IR_Distance_R > 60.0f) open_sides++;
          if (IR_Distance_B > 60.0f) open_sides++;

          uint8_t is_onstage = 0;
          if (low_grey_count >= 3 || (low_grey_count >= 1 && open_sides >= 2))
          {
            is_onstage = 1;
          }

          printf("[AutoClimb] Launch check details:\r\n");
          printf("  - Grey (F,B,L,R): %.1f, %.1f, %.1f, %.1f | low_grey_count = %d\r\n", 
                 Grey_Front, Grey_Back, Grey_Left, Grey_Right, low_grey_count);
          printf("  - IR Dist (L,R,B): %.1f, %.1f, %.1f | open_sides = %d\r\n", 
                 IR_Distance_L, IR_Distance_R, IR_Distance_B, open_sides);
          
          if (is_onstage)
          {
            /* 台上启动：直接静止不动 */
            robot_state = ROBOT_CLIMBING_ONSTAGE;
            climb_start_tick = HAL_GetTick();
            printf("[AutoClimb] Position: ON-STAGE. Platform detected, staying static. State -> ROBOT_CLIMBING_ONSTAGE\r\n");
          }
          else
          {
            /* 台下启动：进入台下扫描与智能测距寻向状态 */
            robot_state = ROBOT_SCANNING;
            printf("[AutoClimb] Position: OFF-STAGE. Initiating intelligent lane scanning... State -> ROBOT_SCANNING\r\n");
          }
          break;
        }
        
        case ROBOT_CLIMBING_ONSTAGE:
        {
          /* 台上启动：保持绝对静止 */
          Motor_Control(0, 0);
          robot_state = ROBOT_FINISHED;
          printf("[AutoClimb] On-stage static mode activated. State -> ROBOT_FINISHED.\r\n");
          break;
        }
        
        case ROBOT_SCANNING:
        {
          /* 台下自动寻向与对齐：无论如何，启动后先缓慢自转扫掠环境。
             利用通道极其狭窄（约45cm）的物理约束进行动态扫掠判定，彻底避开前红外测距(max=30cm)的物理量程限制！
          */
          
          /* 启动自转扫掠 */
          Motor_Control(SCAN_ROTATE_SPEED, 0);
          
          /* 1. 动态扫掠过程中，首先实时判定是否处于角落 (后方和左/右同时贴墙，不包含前方红外) */
          if (IR_Distance_B < 40.0f && IR_Distance_L < 40.0f)
          {
            /* 后左角 (B+L) */
            corner_type = 1; 
            corner_phase = 0;
            corner_start_tick = HAL_GetTick();
            robot_state = ROBOT_CORNER_ESCAPE;
            Motor_Control(0, 0);
            printf("[AutoClimb] Corner detected during sweep: BACK-LEFT. Escaping...\r\n");
            break;
          }
          else if (IR_Distance_B < 40.0f && IR_Distance_R < 40.0f)
          {
            /* 后右角 (B+R) */
            corner_type = 2; 
            corner_phase = 0;
            corner_start_tick = HAL_GetTick();
            robot_state = ROBOT_CORNER_ESCAPE;
            Motor_Control(0, 0);
            printf("[AutoClimb] Corner detected during sweep: BACK-RIGHT. Escaping...\r\n");
            break;
          }
          
          /* 2. 动态扫掠过程中，实时判定是否正对通道（垂直于通道：后侧贴墙，左右极大开阔）
             前侧红外 F 由于量程太短(max 30cm)在此处完全不参与判定，保证在扫掠时能有最高稳定性！ */
          else if (IR_Distance_B < 40.0f && IR_Distance_L > 60.0f && IR_Distance_R > 60.0f)
          {
            /* 已经垂直于通道！此时由于初始平行零偏(Yaw=0°)，我们判定转向目标一定是 90° 的整数倍 */
            
            /* 【核心物理修正】：由于顶部高位红外测距 IR_Distance_UP 实际安装在机器人【后部（顶部靠后）】且朝后发射：
               - 当小车【屁股面对台阶】（正确方向：前铲面对高围栏，后部面对6cm低台阶）时，高位红外会射入空中，读数极大（>= TOP_IR_THRESHOLD）。
               - 当小车【屁股面对围栏】（反向：后部面对高围栏，前铲面对低台阶）时，高位红外会直接照在近距离围栏高墙上，读数极小（< TOP_IR_THRESHOLD）。
            */
            if (IR_Distance_UP >= TOP_IR_THRESHOLD)
            {
              /* 完美！屁股（后部）面对的是低台阶（射空，说明后方是低台），铲子（前部）正对高围栏。直接进入顶墙对齐！ */
              /* 将目标对齐角度锁定至完美的 90° 整数倍角度 (90.0° 或 -90.0°) */
              target_angle_lock = (IMU_Data.Yaw > 0.0f) ? 90.0f : -90.0f;
              robot_state = ROBOT_SQUARING;
              squaring_start_tick = HAL_GetTick();
              Motor_Control(0, 0);
              printf("[AutoClimb] Edge alignment detected! Front faces fence (correct). Locked Target Yaw to %.1f. Squaring...\r\n", target_angle_lock);
            }
            else
            {
              /* 反了！屁股（后部）面对的是围栏高墙（读数近，仅为 %.1f），车头（前部）正对低台阶。我们需要原地旋转 180 度！ */
              float base_snap = (IMU_Data.Yaw > 0.0f) ? 90.0f : -90.0f;
              target_angle_lock = base_snap + 180.0f;
              while (target_angle_lock > 180.0f)  target_angle_lock -= 360.0f;
              while (target_angle_lock < -180.0f) target_angle_lock += 360.0f;
              
              corner_type = 1;   /* 借用脱困逻辑变量 */
              corner_phase = 1;  /* 直接进入旋转阶段 */
              robot_state = ROBOT_CORNER_ESCAPE;
              Motor_Control(0, 0);
              printf("[AutoClimb] Edge alignment detected! Reversed (Butt faces fence, UP_IR = %.1f). Adjusting 180 deg to snapped Target Yaw %.1f...\r\n", IR_Distance_UP, target_angle_lock);
            }
            break;
          }
          break;
        }
        
        case ROBOT_CORNER_ESCAPE:
        {
          if (corner_phase == 0)
          {
            /* 阶段 0：开环避障移动 */
            if (corner_type == 1 || corner_type == 2)
            {
              /* 后侧有墙，向前行驶避开 */
              Motor_Control(0, 30);
            }
            else
            {
              /* 备用兜底：前侧有墙，向后倒退避开 */
              Motor_Control(0, -30);
            }
            
            if (HAL_GetTick() - corner_start_tick >= CORNER_MOVE_MS)
            {
              Motor_Control(0, 0);
              corner_phase = 1;
              
              /* 计算精确且完美的 90° 的倍数转向目标角度，完全基于 Yaw=0° 的初始平行态 */
              if (corner_type == 1)      target_angle_lock = 90.0f;  /* 后左角：转至左转 90° 面向围栏 */
              else if (corner_type == 2) target_angle_lock = -90.0f; /* 后右角：转至右转 90° 面向围栏 */
              else                       target_angle_lock = 90.0f;
              
              printf("[AutoClimb] Corner move done. Next: turn to snapped Target Yaw: %.1f...\r\n", target_angle_lock);
            }
          }
          else
          {
            /* 阶段 1：使用陀螺仪反馈，原位旋转至完美的 90° 的倍数对准边缘 */
            float yaw_err = IMU_Data.Yaw - target_angle_lock;
            while (yaw_err > 180.0f)  yaw_err -= 360.0f;
            while (yaw_err < -180.0f) yaw_err += 360.0f;
            
            if (fabs(yaw_err) < 5.0f)
            {
              /* 转向完成：进入顶墙对齐状态 */
              Motor_Control(0, 0);
              robot_state = ROBOT_SQUARING;
              squaring_start_tick = HAL_GetTick();
              printf("[AutoClimb] Turn complete! Transition to ROBOT_SQUARING...\r\n");
            }
            else
            {
              /* 简单的比例控制旋转速度 */
              float turn_speed = yaw_err * 1.2f;
              if (turn_speed > 25.0f)  turn_speed = 25.0f;
              if (turn_speed < -25.0f) turn_speed = -25.0f;
              
              /* 设定转向死区，防止低速电机不转 */
              if (turn_speed > 0 && turn_speed < 12.0f)  turn_speed = 12.0f;
              if (turn_speed < 0 && turn_speed > -12.0f) turn_speed = -12.0f;
              
              Motor_Control((int16_t)turn_speed, 0);
            }
          }
          break;
        }
        
        case ROBOT_SQUARING:
        {
          /* 物理对齐：以 SQUARING_SPEED 向前顶墙，物理顶墙时间为固定的 1200ms */
          Motor_Control(0, SQUARING_SPEED);
          
          if (HAL_GetTick() - squaring_start_tick >= SQUARING_DURATION_MS)
          {
            /* 停止电机，等待 100ms 以实现物理惯性静止，然后清零 IMU 偏航角 */
            Motor_Control(0, 0);
            osDelay(100);
            
            IMU_ZeroZAxis();
            osDelay(100); /* 等待零偏写入完成 */
            
            target_angle_lock = 0.0f; /* 锁定完美的 0° 倒退直走 */
            climb_pd_init = 0;        /* 清零 PD 控制器初始化标志 */
            grey_gradual_success = 0;
            for(int i = 0; i < 4; i++) grey_gradual_cnt[i] = 0;
            climb_imu_tilted = 0;
            climb_imu_prelim_success = 0;
            robot_state = ROBOT_CLIMBING_OFFSTAGE;
            climb_start_tick = HAL_GetTick();
            printf("[AutoClimb] Squaring complete! Yaw Z-axis reset to 0. Starting closed-loop climb...\r\n");
          }
          break;
        }
        
        case ROBOT_CLIMBING_OFFSTAGE:
        {
          /* 台下闭环登台：全速倒车 + 陀螺仪锁角PD纠偏（闭环控制） */
          float yaw_err = IMU_Data.Yaw - target_angle_lock;
          while (yaw_err > 180.0f)  yaw_err -= 360.0f;
          while (yaw_err < -180.0f) yaw_err += 360.0f;
          
          if (climb_pd_init == 0)
          {
            last_yaw_err = yaw_err;
            climb_pd_init = 1;
          }
          
          /* 计算微分项 */
          float d_err = (yaw_err - last_yaw_err) / 0.020f; /* 20ms 的刷新周期 */
          last_yaw_err = yaw_err;
          
          /* PD 纠偏控制输出 */
          float heading_correction = yaw_err * HEADING_KP + d_err * HEADING_KD;
          if (heading_correction > 30.0f)  heading_correction = 30.0f;
          if (heading_correction < -30.0f) heading_correction = -30.0f;
          
          /* 闭环负反馈进行纠偏：对于反相安装的硬件，使用 -heading_correction 纠错。直接使用全速 CLIMB_SPEED，无软启动 */
          Motor_Control((int16_t)-heading_correction, CLIMB_SPEED);
          
          uint32_t elapsed = HAL_GetTick() - climb_start_tick;
          
          /* 登台成功判定：
             如果是串口 'Y' 测试模式：运行时间到达 1.0 秒 (1000ms) 时立即刹车停止，进入 ROBOT_FINISHED。
             如果是官方正式登台：
               1. 首个 1.0 秒 (CLIMB_BLIND_DURATION_MS) 为盲爬行阶段，完全忽略灰度传感器，全速后退；
               2. 1.0 秒后，启动主动成功判定：当 Grey_Front < ONSTAGE_GREY_SUCCESS_THRESHOLD 时说明已完全登台；
               3. 3.5 秒 (CLIMB_OFFSTAGE_TIMEOUT_MS) 超时保护。 */
          if (is_test_mode)
          {
            if (elapsed >= 1000U)
            {
              Motor_Control(0, 0);
              robot_state = ROBOT_FINISHED;
              printf("[TestClimb] Test climbing complete! Elapsed: %lums. State -> ROBOT_FINISHED. Staying static.\r\n", elapsed);
            }
          }
          else
          {
            /* 监测倾斜状态 */
            if (fabs(IMU_Data.Pitch) > 12.0f || fabs(IMU_Data.Roll) > 12.0f)
            {
              climb_imu_tilted = 1;
            }
            
            /* 如果检测到倾斜后又恢复水平 */
            if (climb_imu_tilted && (fabs(IMU_Data.Pitch) <= 12.0f && fabs(IMU_Data.Roll) <= 12.0f))
            {
              Motor_Control(0, 0);
              robot_state = ROBOT_RUNNING;
              onstage_confirmed = 0; // 开启台上 1 秒灰度渐变确认
              onstage_confirm_timer = HAL_GetTick();
              climb_imu_tilted = 0;
              printf("[AutoClimb] IMU tilt recovery detected! Preliminary success. Transition to ROBOT_RUNNING. Verifying grey gradual change...\r\n");
            }
            /* 3.5秒超时保护 */
            else if (elapsed >= CLIMB_OFFSTAGE_TIMEOUT_MS)
            {
              Motor_Control(0, 0);
              robot_state = ROBOT_SCANNING; // 登台失败，退回扫描
              climb_imu_tilted = 0;
              printf("[AutoClimb] Timeout (%dms) without IMU recovery. Climb failed! Fallback to ROBOT_SCANNING...\r\n", CLIMB_OFFSTAGE_TIMEOUT_MS);
            }
          }
          break;
        }
        
        case ROBOT_FINISHED:
        {
          /* 登台完成或台上待机：保持绝对静止 */
          Motor_Control(0, 0);
          break;
        }

        case ROBOT_TEST_ROTATE_PREPARE:
        {
          is_test_mode = 1; /* 标记为测试模式 */
          /* 1. 停止小车电机，准备校准 */
          Motor_Control(0, 0);
          
          /* 2. 发送指令重置 Z 轴角度（偏航角） */
          IMU_ZeroZAxis();
          
          /* 3. 在任务上下文中安全地等待最多 500ms 直到读取到真实的零偏数据 */
          uint32_t wait_start = HAL_GetTick();
          while (HAL_GetTick() - wait_start < 500)
          {
            osDelay(20);
            if (fabs(IMU_Data.Yaw) < 1.0f)
            {
              break;
            }
          }
          
          /* 4. 开始和每次停顿的时候记录后方靠上的传感器数值 (S0) */
          up_ir_values[0] = IR_Distance_UP;
          rotate_step = 0;
          
          /* 5. 初始化转向参数并跳转到 90° 旋转测试状态（本车逆时针/左转会导致偏航角 Yaw 增加） */
          target_angle_lock = 90.0f;
          init_rotate_pd = 0; /* 标志位重置 */
          robot_state = ROBOT_TEST_ROTATE_90;
          
          printf("[TestRotate] Z-axis zeroed. Target locked to 90.0. Start IR: S0=%.1f. Entering ROBOT_TEST_ROTATE_90 loop...\r\n", up_ir_values[0]);
          break;
        }
        
        case ROBOT_TEST_ROTATE_90:
        {
          /* 逆时针转动 90° */
          float yaw_err = IMU_Data.Yaw - target_angle_lock;
          while (yaw_err > 180.0f)  yaw_err -= 360.0f;
          while (yaw_err < -180.0f) yaw_err += 360.0f;
          
          /* 降低实时调试打印频率至 300ms 一次 */
          static uint32_t last_print_tick = 0;
          if (HAL_GetTick() - last_print_tick >= 300)
          {
            last_print_tick = HAL_GetTick();
            printf("[TestRotate] Step: %d, Target: %.1f, Yaw: %.1f, Err: %.1f\r\n", rotate_step, target_angle_lock, IMU_Data.Yaw, yaw_err);
          }

          /* 缩紧死区为更精确的 3.0° */
          if (fabs(yaw_err) < 3.0f)
          {
            /* 达到当前目标角度，停顿 0.3s (300ms) */
            Motor_Control(0, 0);
            osDelay(300);
            
            /* 每次停顿的时候记录后方靠上的传感器数值 */
            rotate_step++;
            if (rotate_step <= 3)
            {
              up_ir_values[rotate_step] = IR_Distance_UP;
              printf("[TestRotate] Stop recorded: S%d=%.1f\r\n", rotate_step, up_ir_values[rotate_step]);
            }
            
            /* 如果转完了 270°（即 rotate_step 达到 3），进行边缘/角落判定，并启动对应登台逻辑 */
            if (rotate_step >= 3)
            {
              uint8_t blocked[4] = {0};
              int blocked_count = 0;
              for (int i = 0; i < 4; i++)
              {
                if (up_ir_values[i] < 95.0f)
                {
                  blocked[i] = 1;
                  blocked_count++;
                }
              }
              
              // 首尾（index 0 和 3）也算相邻，环形相邻检查
              uint8_t has_adjacent = (blocked[0] && blocked[1]) || 
                                     (blocked[1] && blocked[2]) || 
                                     (blocked[2] && blocked[3]) || 
                                     (blocked[3] && blocked[0]);
              
              printf("[TestRotate] Scan complete! S0=%.1f, S1=%.1f, S2=%.1f, S3=%.1f\r\n", 
                     up_ir_values[0], up_ir_values[1], up_ir_values[2], up_ir_values[3]);
              printf("[TestRotate] Blocked status: [%d, %d, %d, %d] (count=%d, has_adj=%d)\r\n",
                     blocked[0], blocked[1], blocked[2], blocked[3], blocked_count, has_adjacent);
              
              if (has_adjacent)
              {
                printf("在角落\r\n");
                
                // 寻找具有最小传感器数值之和的相邻被阻挡方向对，以确保在噪声干扰或多向阻挡时精准定位真实墙面
                int idx1 = -1, idx2 = -1;
                float min_sum = 9999.0f;
                
                for (int i = 0; i < 4; i++)
                {
                  int next_i = (i + 1) % 4;
                  if (blocked[i] && blocked[next_i])
                  {
                    float sum = up_ir_values[i] + up_ir_values[next_i];
                    if (sum < min_sum)
                    {
                      min_sum = sum;
                      idx1 = i;
                      idx2 = next_i;
                    }
                  }
                }
                
                if (idx1 != -1 && idx2 != -1)
                {
                  // 比较这两个方向上靠上的传感器数值大小
                  int D_larger = (up_ir_values[idx1] > up_ir_values[idx2]) ? idx1 : idx2;
                  int D_smaller = (up_ir_values[idx1] > up_ir_values[idx2]) ? idx2 : idx1;
                  
                  saved_D_smaller = D_smaller; // 暂存较小值方向
                  
                  // 计算较大方向的物理角度（由于探测时车尾朝墙，此方向即为车头朝向开阔侧的方向）
                  float angle_larger = D_larger * 90.0f;
                  while (angle_larger > 180.0f) angle_larger -= 360.0f;
                  while (angle_larger < -180.0f) angle_larger += 360.0f;
                  
                  climb_target_angle = angle_larger;
                  climb_next_state = ROBOT_CLIMB_FORWARD_UNTIL_UP_IR;
                  robot_state = ROBOT_CLIMB_ROTATE_TO_DIR;
                  init_rotate_pd = 0;
                  
                  printf("[TestClimb] Corner detected! Selected adjacent pair %d and %d. Turning to D_larger=%d (angle=%.1f), saved D_smaller=%d\r\n", 
                         idx1, idx2, D_larger, climb_target_angle, saved_D_smaller);
                }
                else
                {
                  robot_state = ROBOT_FINISHED;
                }
              }
              else if (blocked_count == 1)
              {
                printf("在边缘\r\n");
                
                // 找出阻碍的那一个方向
                int D_blocked = 0;
                for (int i = 0; i < 4; i++)
                {
                  if (blocked[i]) { D_blocked = i; break; }
                }
                
                // 计算相反方向物理角度 (D_blocked * 90 + 180)
                float opp_angle = (D_blocked * 90.0f) + 180.0f;
                while (opp_angle > 180.0f) opp_angle -= 360.0f;
                while (opp_angle < -180.0f) opp_angle += 360.0f;
                
                climb_target_angle = opp_angle;
                climb_next_state = ROBOT_CLIMB_FORWARD_UNTIL_WALL;
                robot_state = ROBOT_CLIMB_ROTATE_TO_DIR;
                init_rotate_pd = 0;
                
                printf("[TestClimb] Edge detected! Turning to opposite of D_blocked=%d (opp_angle=%.1f) to face fence\r\n", 
                       D_blocked, climb_target_angle);
              }
              else
              {
                printf("不在边缘且不在角落 (障碍数: %d)，小车停下\r\n", blocked_count);
                robot_state = ROBOT_FINISHED;
              }
            }
            else
            {
              /* 设定下一个逆时针转动 90° 的目标角（逆时针/左转使 Yaw 增加，所以每次加上 90°） */
              target_angle_lock += 90.0f;
              while (target_angle_lock > 180.0f)  target_angle_lock -= 360.0f;
              while (target_angle_lock < -180.0f) target_angle_lock += 360.0f;
              
              /* 重新初始化 PD 控制器的基准值，防止目标突变产生错误的微分冲击 */
              init_rotate_pd = 0;
              
              printf("[TestRotate] Moving to Next Target Yaw: %.1f\r\n", target_angle_lock);
            }
          }
          else
          {
            /* 引入 PD 控制以获得阻尼效果 */
            if (init_rotate_pd == 0)
            {
              last_yaw_err = yaw_err;
              init_rotate_pd = 1;
            }
            float d_err = (yaw_err - last_yaw_err);
            last_yaw_err = yaw_err;
            
            float turn_speed = yaw_err * 0.8f + d_err * 7.5f;
            
            /* 限制最大输出转速，防止电机满载滑行过冲 */
            if (turn_speed > 18.0f)  turn_speed = 18.0f;
            if (turn_speed < -18.0f) turn_speed = -18.0f;
            
            /* 误差限幅控制 */
            if (fabs(yaw_err) > 5.0f)
            {
              if (turn_speed > 0.0f && turn_speed < 8.0f)  turn_speed = 8.0f;
              if (turn_speed < 0.0f && turn_speed > -8.0f) turn_speed = -8.0f;
            }
            
            Motor_Control((int16_t)turn_speed, 0);
          }
          break;
        }
        
        case ROBOT_CLIMB_ROTATE_TO_DIR:
        {
          float yaw_err = IMU_Data.Yaw - climb_target_angle;
          while (yaw_err > 180.0f)  yaw_err -= 360.0f;
          while (yaw_err < -180.0f) yaw_err += 360.0f;
          
          if (fabs(yaw_err) < 3.0f)
          {
            /* 转向完成：停止电机，并切换到下一阶段 */
            Motor_Control(0, 0);
            osDelay(200);
            init_rotate_pd = 0; // 重置 PD 标志
            robot_state = climb_next_state;
            if (climb_next_state == ROBOT_SQUARING)
            {
              squaring_start_tick = HAL_GetTick();
            }
            if (climb_next_state == ROBOT_CLIMB_FORWARD_UNTIL_WALL || climb_next_state == ROBOT_CLIMB_FORWARD_UNTIL_UP_IR || climb_next_state == ROBOT_TEST_ATTACK_MOVE)
            {
              climb_wall_align_start_tick = HAL_GetTick();
            }
            printf("[TestClimb] Rotation to %.1f complete. Next state: %d\r\n", climb_target_angle, climb_next_state);
          }
          else
          {
            /* 转向 PD 控制 */
            if (init_rotate_pd == 0)
            {
              last_yaw_err = yaw_err;
              init_rotate_pd = 1;
            }
            float d_err = (yaw_err - last_yaw_err);
            last_yaw_err = yaw_err;
            
            float turn_speed = yaw_err * 0.8f + d_err * 7.5f;
            if (turn_speed > 18.0f)  turn_speed = 18.0f;
            if (turn_speed < -18.0f) turn_speed = -18.0f;
            
            if (fabs(yaw_err) > 5.0f)
            {
              if (turn_speed > 0.0f && turn_speed < 8.0f)  turn_speed = 8.0f;
              if (turn_speed < 0.0f && turn_speed > -8.0f) turn_speed = -8.0f;
            }
            Motor_Control((int16_t)turn_speed, 0);
          }
          break;
        }
        
        case ROBOT_CLIMB_FORWARD_UNTIL_UP_IR:
        {
          /* 前进避开角落障碍（前行 1.5 秒以实打实走离角落侧墙） */
          Motor_Control(0, 25);
          
          if (HAL_GetTick() - climb_wall_align_start_tick >= 1500U)
          {
            /* 成功越过，停止并准备转弯到 D_smaller 的相反方向 */
            Motor_Control(0, 0);
            osDelay(200);
            
            float opposite_smaller = (saved_D_smaller * 90.0f) + 180.0f;
            while (opposite_smaller > 180.0f) opposite_smaller -= 360.0f;
            while (opposite_smaller < -180.0f) opposite_smaller += 360.0f;
            
            climb_target_angle = opposite_smaller;
            climb_next_state = ROBOT_CLIMB_FORWARD_UNTIL_WALL;
            robot_state = ROBOT_CLIMB_ROTATE_TO_DIR;
            init_rotate_pd = 0;
            
            printf("[TestClimb] Corner escape forward complete (1.5s). Next: turn to opposite of D_smaller (%.1f) and move forward until wall\r\n", climb_target_angle);
          }
          break;
        }
        
        case ROBOT_CLIMB_FORWARD_UNTIL_WALL:
        {
          /* 向前前行开路 */
          Motor_Control(0, 25);
          
          uint32_t elapsed_align = HAL_GetTick() - climb_wall_align_start_tick;
          
          // 过滤掉 999.0f 等无效超距值以防止误触发；结合前红外近距离判断与超时保护
          if ((IR_Distance_F > 48.0f && IR_Distance_F < 150.0f) || 
              (IR_Distance_B > 60.0f && IR_Distance_B < 150.0f) || 
              (IR_Distance_F < 20.0f && IR_Distance_F > 1.0f) || 
              elapsed_align >= 2000U)
          {
            /* 条件触发，开启 1 秒延时 */
            climb_timer = HAL_GetTick();
            robot_state = ROBOT_CLIMB_FORWARD_DELAY;
            printf("[TestClimb] Wall condition met (F=%.1f, B=%.1f, elapsed=%lums). Starting 1s forward delay...\r\n", 
                   IR_Distance_F, IR_Distance_B, elapsed_align);
          }
          break;
        }
        
        case ROBOT_CLIMB_FORWARD_DELAY:
        {
          /* 继续前行 1 秒 */
          Motor_Control(0, 25);
          
          if (HAL_GetTick() - climb_timer >= 1000U)
          {
            /* 1秒到，全速向后倒退冲上擂台 */
            climb_timer = HAL_GetTick();
            robot_state = ROBOT_CLIMB_BACKWARD_FAST;
            printf("[TestClimb] Delay complete. Launching full speed backward for 1s...\r\n");
          }
          break;
        }
        
        case ROBOT_CLIMB_BACKWARD_FAST:
        {
          /* 全速向后冲台 */
          Motor_Control(0, -100);
          
          if (HAL_GetTick() - climb_timer >= 1000U)
          {
            /* 登台结束，刹车停止 */
            Motor_Control(0, 0);
            robot_state = ROBOT_FINISHED;
            printf("[TestClimb] Onstage complete! Robot stopped. State -> ROBOT_FINISHED\r\n");
          }
          break;
        }

        case ROBOT_TEST_ATTACK_PREPARE:
        {
          /* 1. 停止电机 */
          Motor_Control(0, 0);
          /* 2. MPU6050 清零 */
          IMU_ZeroZAxis();
          
          /* 3. 在任务上下文中安全地等待最多 500ms 直到读取到真实的零偏数据 */
          uint32_t wait_start = HAL_GetTick();
          while (HAL_GetTick() - wait_start < 500)
          {
            osDelay(20);
            if (fabs(IMU_Data.Yaw) < 1.0f)
            {
              break;
            }
          }
          
          printf("[TestAttack] MPU6050 zeroed. Entering ROBOT_TEST_ATTACK_RUNNING...\r\n");
          robot_state = ROBOT_TEST_ATTACK_RUNNING;
          break;
        }

        case ROBOT_TEST_ATTACK_RUNNING:
        {
          /* 获取 IR 距离，依据自主识别敌人步骤计算转角 */
          float dist[8];
          dist[0] = ir_distance[0]; // 前方
          dist[1] = ir_distance[1]; // 左前
          dist[2] = ir_distance[8]; // 左侧 (交换后)
          dist[3] = ir_distance[3]; // 左后
          dist[4] = ir_distance[5]; // 后方
          dist[5] = ir_distance[4]; // 右后
          dist[6] = ir_distance[6]; // 右侧 (交换后)
          dist[7] = ir_distance[2]; // 右前
          
          /* 判断各方向是否触发，前方<20，其余<50 */
          int trig[8];
          for(int i = 0; i < 8; i++) {
              if (i == 0) trig[i] = (dist[i] < 20.0f) ? 1 : 0;
              else trig[i] = (dist[i] < 50.0f) ? 1 : 0;
          }
          
          int match_3 = -1;
          for(int i = 0; i < 8; i++) {
              int prev = (i + 7) % 8;
              int next = (i + 1) % 8;
              if (trig[prev] && trig[i] && trig[next]) {
                  match_3 = i;
                  break;
              }
          }
          
          int match_2_1 = -1, match_2_2 = -1;
          if (match_3 == -1) {
              for(int i = 0; i < 8; i++) {
                  int next = (i + 1) % 8;
                  if (trig[i] && trig[next]) {
                      match_2_1 = i;
                      match_2_2 = next;
                      break;
                  }
              }
          }
          
          int match_1 = -1;
          if (match_3 == -1 && match_2_1 == -1) {
              for(int i = 0; i < 8; i++) {
                  if (trig[i]) {
                      match_1 = i;
                      break;
                  }
              }
          }
          
          float t_angle = 0.0f;
          int found = 0;
          
          if (match_3 != -1) {
              t_angle = match_3 * 45.0f;
              found = 1;
              printf("[TestAttack] 3 adjacent found at index %d\r\n", match_3);
          } else if (match_2_1 != -1) {
              if (match_2_1 == 7 && match_2_2 == 0) {
                  t_angle = 337.5f; /* (-45+0)/2 = -22.5 => 337.5 */
              } else {
                  t_angle = (match_2_1 + match_2_2) * 45.0f / 2.0f;
              }
              found = 1;
              printf("[TestAttack] 2 adjacent found at %d, %d\r\n", match_2_1, match_2_2);
          } else if (match_1 != -1) {
              t_angle = match_1 * 45.0f;
              found = 1;
              printf("[TestAttack] Single target found at index %d\r\n", match_1);
          }
          
          if (found) {
              /* 角度标准化到 -180 ~ 180 */
              while (t_angle > 180.0f) t_angle -= 360.0f;
              while (t_angle < -180.0f) t_angle += 360.0f;
              
              if (fabs(t_angle) < 5.0f) { // 直行
                 robot_state = ROBOT_TEST_ATTACK_MOVE;
                 climb_wall_align_start_tick = HAL_GetTick();
                 printf("[TestAttack] Target is front. Moving immediately...\r\n");
              } else {
                 climb_target_angle = t_angle;
                 climb_next_state = ROBOT_TEST_ATTACK_MOVE;
                 robot_state = ROBOT_CLIMB_ROTATE_TO_DIR;
                 init_rotate_pd = 0;
                 printf("[TestAttack] Turning to %.1f...\r\n", climb_target_angle);
              }
          }
          break;
        }

        case ROBOT_TEST_ATTACK_MOVE:
        {
          /* 向前移动0.5S */
          Motor_Control(0, 40); /* 假设速度设为40 */
          
          if (HAL_GetTick() - climb_wall_align_start_tick >= 500U)
          {
            /* 0.5s时间到 */
            Motor_Control(0, 0);
            robot_state = ROBOT_FINISHED;
            printf("[TestAttack] Move complete. Test attack finished.\r\n");
          }
          break;
        }

        case ROBOT_FIGHT_ATTACK_MOVE:
        {
          /* 实战冲击追击状态：以正常巡台速度向敌人方向行驶，并在前进中实时微调转向 */
          
          /* 获取 IR 距离，依据自主识别敌人步骤计算转角 */
          float dist[8];
          dist[0] = ir_distance[0]; // 前方
          dist[1] = ir_distance[1]; // 左前
          dist[2] = ir_distance[8]; // 左侧 (交换后)
          dist[3] = ir_distance[3]; // 左后
          dist[4] = ir_distance[5]; // 后方
          dist[5] = ir_distance[4]; // 右后
          dist[6] = ir_distance[6]; // 右侧 (交换后)
          dist[7] = ir_distance[2]; // 右前
          
          /* 判断各方向是否触发，前方 < 20.0f，其余 < 50.0f */
          int trig[8];
          for(int i = 0; i < 8; i++) {
              if (i == 0) trig[i] = (dist[i] < 20.0f) ? 1 : 0;
              else trig[i] = (dist[i] < 50.0f) ? 1 : 0;
          }
          
          int match_3 = -1;
          for(int i = 0; i < 8; i++) {
              int prev = (i + 7) % 8;
              int next = (i + 1) % 8;
              if (trig[prev] && trig[i] && trig[next]) {
                  match_3 = i;
                  break;
              }
          }
          
          int match_2_1 = -1, match_2_2 = -1;
          if (match_3 == -1) {
              for(int i = 0; i < 8; i++) {
                  int next = (i + 1) % 8;
                  if (trig[i] && trig[next]) {
                      match_2_1 = i;
                      match_2_2 = next;
                      break;
                  }
              }
          }
          
          int match_1 = -1;
          if (match_3 == -1 && match_2_1 == -1) {
              for(int i = 0; i < 8; i++) {
                  if (trig[i]) {
                      match_1 = i;
                      break;
                  }
              }
          }
          
          float t_angle = 0.0f;
          int found = 0;
          
          if (match_3 != -1) {
              t_angle = match_3 * 45.0f;
              found = 1;
          } else if (match_2_1 != -1) {
              if (match_2_1 == 7 && match_2_2 == 0) {
                  t_angle = -22.5f;
              } else {
                  t_angle = (match_2_1 + match_2_2) * 45.0f / 2.0f;
              }
              found = 1;
          } else if (match_1 != -1) {
              t_angle = match_1 * 45.0f;
              found = 1;
          }
          
          float turn_speed = 0.0f;
          if (found) {
              /* 角度标准化到 -180 ~ 180 */
              while (t_angle > 180.0f) t_angle -= 360.0f;
              while (t_angle < -180.0f) t_angle += 360.0f;
              
              static uint8_t re_align_filter = 0;
              if (fabs(t_angle) > 60.0f) {
                  re_align_filter++;
                  if (re_align_filter >= 4) {
                      /* 敌人偏移角度过大（偏到侧方或后方），停止向前，重新原地自转对齐 */
                      climb_target_angle = IMU_Data.Yaw + t_angle;
                      while (climb_target_angle > 180.0f)  climb_target_angle -= 360.0f;
                      while (climb_target_angle < -180.0f) climb_target_angle += 360.0f;
                      
                      climb_next_state = ROBOT_FIGHT_ATTACK_MOVE;
                      robot_state = ROBOT_CLIMB_ROTATE_TO_DIR;
                      init_rotate_pd = 0;
                      Motor_Control(0, 0);
                      printf("[FightAttack] Enemy moved to side/behind (%.1f). Re-aligning...\r\n", t_angle);
                      re_align_filter = 0;
                      break; // 跳出当前 state 逻辑
                  }
              } else {
                  re_align_filter = 0;
                  /* 敌人在前方扇区内，计算转向速度（正值左转，负值右转） */
                  float Kp_track = 0.3f;
                  turn_speed = -t_angle * Kp_track;
                  if (turn_speed > 15.0f)  turn_speed = 15.0f;
                  if (turn_speed < -15.0f) turn_speed = -15.0f;
              }
          }
          
          /* 融合前行速度（正常巡台速度 18）与转向速度 */
          Motor_Control((int16_t)turn_speed, 18);
          
          /* 实时检测边缘：一旦有一侧激光检测到边缘值（> 260），立即制动、退后、转身，重回巡台 */
          if (laser_dist_1 > 260 || laser_dist_2 > 260)
          {
            printf("[FightAttack] Edge detected (Laser1: %d, Laser2: %d). Canceling attack, backing up and turning...\r\n", laser_dist_1, laser_dist_2);
            
            /* 1. 强力反向制动 80ms 快速消能，随后后退 220ms */
            Motor_Control(0, -60);
            osDelay(80);
            Motor_Control(0, -22);
            osDelay(220);
            
            /* 2. 转向避让：降低转弯速度（-60 -> -35），转向时间 450ms */
            if (laser_dist_1 > 260 && laser_dist_2 <= 260) {
              Motor_Control(-35, 0); // 左侧偏离，向右后退左转
            } else if (laser_dist_1 <= 260 && laser_dist_2 > 260) {
              Motor_Control(35, 0);  // 右侧偏离，向左转
            } else {
              Motor_Control(-35, 0); // 双侧或异常，默认左转
            }
            osDelay(450);
            
            /* 3. 停止转向 */
            Motor_Control(0, 0);
            osDelay(125);
            
            /* 4. 重置状态为 ROBOT_RUNNING，恢复常规巡台 */
            robot_state = ROBOT_RUNNING;
            printf("[FightAttack] Edge escape complete. Resuming patrol...\r\n");
          }
          break;
        }
        
        default:
          break;
      }
      
      /* 在自主登台及软启动非运行状态下，拦截常规控制，保持 20ms 的刷新周期 */
      osDelay(20);
      continue;
    }

    /* ========== ROBOT_RUNNING 台上运行时的确认与掉台检测 ========== */
    if (robot_state == ROBOT_RUNNING)
    {
      static uint32_t onstage_settle_start_tick = 0;
      
      if (onstage_confirmed == 0)
      {
        onstage_settle_start_tick = 0; // 确保在验证期清零，以便进入确认期时重新计时
        
        /* 阶段 2：在台上运行的 1 秒钟内最终确认灰度渐变 */
        if (grey_gradual_success == 1)
        {
          onstage_confirmed = 1;
          printf("[OnstageConfirm] Grey gradual transition detected! Climb SUCCESS confirmed.\r\n");
        }
        else if (HAL_GetTick() - onstage_confirm_timer >= 1000U)
        {
          /* 超时未检测到渐变，最终确诊登台失败 */
          Motor_Control(0, 0);
          robot_state = ROBOT_SCANNING; // 重新执行自动登台
          printf("[OnstageConfirm] Timeout (1s) without grey gradual change. Climb FAILED! Fallback to ROBOT_SCANNING...\r\n");
          osDelay(20);
          continue;
        }
      }
      else
      {
        if (onstage_settle_start_tick == 0)
        {
          onstage_settle_start_tick = HAL_GetTick();
          fall_imu_tilted = 0; // 开启防跌落监测前，清空登台阶段遗留的倾斜标志
          printf("[FallDetection] onstage confirmed! Settle timer started (1s).\r\n");
        }
        
        /* 阶段 3：已确定在台上并稳住 1s 后，开启实时掉台检测 */
        if (HAL_GetTick() - onstage_settle_start_tick >= 1000U)
        {
          if (fabs(IMU_Data.Pitch) > 12.0f || fabs(IMU_Data.Roll) > 12.0f)
          {
            fall_imu_tilted = 1;
          }
          
          if (fall_imu_tilted && (fabs(IMU_Data.Pitch) <= 12.0f && fabs(IMU_Data.Roll) <= 12.0f))
          {
            /* IMU 倾斜后恢复水平 */
            if (Grey_Front > 170.0f && Grey_Back > 170.0f && Grey_Left > 170.0f && Grey_Right > 170.0f)
            {
              /* 灰度读数均高（代表身处台下黑色地面），确诊掉下擂台！ */
              Motor_Control(0, 0);
              
              /* 计算相对于比赛开始时起始角度最近的 0°, 90°, -90°, 180° */
              float yaw_div = IMU_Data.Yaw / 90.0f;
              int rounded_index = (yaw_div >= 0.0f) ? (int)(yaw_div + 0.5f) : (int)(yaw_div - 0.5f);
              target_angle_lock = (float)rounded_index * 90.0f;
              while (target_angle_lock > 180.0f)  target_angle_lock -= 360.0f;
              while (target_angle_lock < -180.0f) target_angle_lock += 360.0f;
              
              fall_imu_tilted = 0;
              init_rotate_pd = 0;
              onstage_settle_start_tick = 0;
              robot_state = ROBOT_ALIGN_ORTHO; // 进行方向校正
              printf("[FallDetection] Fall CONFIRMED! Target alignment Yaw: %.1f. Transition to ROBOT_ALIGN_ORTHO...\r\n", target_angle_lock);
              osDelay(20);
              continue;
            }
            else
            {
              /* 灰度读数显示仍在台上，判定为台上撞击颠簸 */
              fall_imu_tilted = 0;
              printf("[FallDetection] IMU tilted but grey sensors represent ON-STAGE. Assumed collision/bump. Resuming running.\r\n");
            }
          }
        }
      }
    }

    if (control_mode == 0 && !Control_IsOnline() && !is_test_mode) {
        motor_target_speed[0] = 0.0f;
        motor_target_speed[1] = 0.0f;
        motor_target_speed[2] = 0.0f;
        motor_target_speed[3] = 0.0f;
        Motor_SetSpeed(MOTOR_1, 0);
        Motor_SetSpeed(MOTOR_2, 0); 
        Motor_SetSpeed(MOTOR_3, 0); 
        Motor_SetSpeed(MOTOR_4, 0); 
    } else {
        PC_ControlData_t ctrl = Control_GetData();
        uint32_t prev_buttons = last_buttons;
        
        /* ========== 模式切换处理 ========== */
        /* button=48 使用上升沿切换遥控模式与自动控制模式 */
        if ((ctrl.buttons & 48) && !(prev_buttons & 48)) 
        {
          control_mode = !control_mode;  /* 切换模式 */
          printf("Mode switched: %s\r\n", control_mode ? "Auto Control" : "Remote Control");
          
          /* 切换模式时，关闭角度环控制 */
          if (control_mode == 1) {
            /* 进入自动控制模式，禁用角度环，并刷新(归零) IMU */
            IMU_ZeroZAxis();
            
            angle_control_enabled = 0;
            angle_control_prev = 0;
            /* 暂停角度环任务 */
            if (Angle_TaskHandle != NULL) {
              vTaskSuspend(Angle_TaskHandle);
            }
            /* 清空摇杆和角度速度 */
            motor_rc_speed[0] = 0.0f;
            motor_rc_speed[1] = 0.0f;
            motor_rc_speed[2] = 0.0f;
            motor_rc_speed[3] = 0.0f;
            motor_angle_speed[0] = 0.0f;
            motor_angle_speed[1] = 0.0f;
            motor_angle_speed[2] = 0.0f;
            motor_angle_speed[3] = 0.0f;
            target_angle = 0.0f;
          } else {
            /* 回到遥控模式，恢复 Angle_Task 准备随时启用 */
            if (Angle_TaskHandle != NULL) {
              vTaskResume(Angle_TaskHandle);
            }
          }
        }
        
        /* ========== 遥控模式 ========== */
        if (control_mode == 0) 
        { 
          /* 在遥控模式下，按下 gamepad Y 键 (buttons & 8) 立即进入自主寻找敌人进攻测试 */
          if ((ctrl.buttons & 8) && !(prev_buttons & 8))
          {
            robot_state = ROBOT_TEST_ATTACK_PREPARE;
            climb_pd_init = 0; /* 清空 PD 标志 */
            printf("[TestAttack] Gamepad 'Y' Trigger! State -> ROBOT_TEST_ATTACK_PREPARE.\r\n");
          }

          /* 始终通过统一入口写入基础摇杆指令，角度环会在 Angle_Task 中叠加修正 */
          Motor_Control((int16_t)ctrl.vy_lr, (int16_t)ctrl.vx_fwd);
          
          /* 舵机控制：固定速率转动
             - 只按lt：持续正向转动
             - 只按rt：持续反向转动
             - 松开：保持当前位置
             这样不会再因为输入�?�变化导致舵机频繁跳动 */
          if (ctrl.lt > 10 && ctrl.rt <= 10) {
            if (last_servo_angle + servo_step <= 180) {
              last_servo_angle += servo_step;
            } else {
              last_servo_angle = 180;
            }
            Servo_SetAngle(last_servo_angle);
          } else if (ctrl.rt > 10 && ctrl.lt <= 10) {
            if (last_servo_angle >= servo_step) {
              last_servo_angle -= servo_step;
            } else {
              last_servo_angle = 0;
            }
            Servo_SetAngle(last_servo_angle);
          }
          
          /* 在遥控模式下�???�??? button=4 的上升沿切换角度环模式 */
          if ((ctrl.buttons & (1U << 4)) && !(prev_buttons & (1U << 4))) 
          {
            angle_control_enabled = !angle_control_enabled;
            printf("Angle Control: %s\r\n", angle_control_enabled ? "Enabled" : "Disabled");
              
            if (angle_control_enabled) {
              /* 启用角度环 */
              if (Angle_TaskHandle != NULL) {
                vTaskResume(Angle_TaskHandle);
              }
              target_angle = IMU_Data.Yaw;
              angle_control_prev = 0;
            } else {
              /* 禁用角度环，清除转向修正 */
              target_angle = 0.0f;
              PID_Clear(&Angle_pid[0]);
              motor_angle_speed[0] = 0.0f;
              motor_angle_speed[1] = 0.0f;
              motor_angle_speed[2] = 0.0f;
              motor_angle_speed[3] = 0.0f;
            }
          }
        }
        /* ========== 自动控制模式 ========== */
        else 
        {
            /* 获取 IR 距离，依据自主识别敌人步骤计算转角 */
            float dist[8];
            dist[0] = ir_distance[0]; // 前方
            dist[1] = ir_distance[1]; // 左前
            dist[2] = ir_distance[8]; // 左侧 (交换后)
            dist[3] = ir_distance[3]; // 左后
            dist[4] = ir_distance[5]; // 后方
            dist[5] = ir_distance[4]; // 右后
            dist[6] = ir_distance[6]; // 右侧 (交换后)
            dist[7] = ir_distance[2]; // 右前
            
            /* 判断各方向是否触发，前方<20，其余<50 */
            int trig[8];
            for(int i = 0; i < 8; i++) {
                if (i == 0) trig[i] = (dist[i] < 20.0f) ? 1 : 0;
                else trig[i] = (dist[i] < 50.0f) ? 1 : 0;
            }
            
            int match_3 = -1;
            for(int i = 0; i < 8; i++) {
                int prev = (i + 7) % 8;
                int next = (i + 1) % 8;
                if (trig[prev] && trig[i] && trig[next]) {
                    match_3 = i;
                    break;
                }
            }
            
            int match_2_1 = -1, match_2_2 = -1;
            if (match_3 == -1) {
                for(int i = 0; i < 8; i++) {
                    int next = (i + 1) % 8;
                    if (trig[i] && trig[next]) {
                        match_2_1 = i;
                        match_2_2 = next;
                        break;
                    }
                }
            }
            
            int match_1 = -1;
            if (match_3 == -1 && match_2_1 == -1) {
                for(int i = 0; i < 8; i++) {
                    if (trig[i]) {
                        match_1 = i;
                        break;
                    }
                }
            }
            
            float t_angle = 0.0f;
            int found = 0;
            
            if (match_3 != -1) {
                t_angle = match_3 * 45.0f;
                found = 1;
            } else if (match_2_1 != -1) {
                if (match_2_1 == 7 && match_2_2 == 0) {
                    t_angle = -22.5f; /* 337.5 相当于 -22.5 */
                } else {
                    t_angle = (match_2_1 + match_2_2) * 45.0f / 2.0f;
                }
                found = 1;
            } else if (match_1 != -1) {
                t_angle = match_1 * 45.0f;
                found = 1;
            }

            static uint8_t enemy_filter_cnt = 0;
            if (found) {
                if (enemy_filter_cnt < 10) enemy_filter_cnt++;
            } else {
                enemy_filter_cnt = 0;
            }

            if (found && enemy_filter_cnt >= 4) {
                /* 角度标准化到 -180 ~ 180 */
                while (t_angle > 180.0f) t_angle -= 360.0f;
                while (t_angle < -180.0f) t_angle += 360.0f;
                
                if (fabs(t_angle) < 5.0f) { // 直行
                    robot_state = ROBOT_FIGHT_ATTACK_MOVE;
                    climb_wall_align_start_tick = HAL_GetTick();
                    printf("[FightAttack] Enemy straight ahead. Launching charge attack immediately...\r\n");
                } else {
                    climb_target_angle = IMU_Data.Yaw + t_angle;
                    while (climb_target_angle > 180.0f)  climb_target_angle -= 360.0f;
                    while (climb_target_angle < -180.0f) climb_target_angle += 360.0f;
                    
                    climb_next_state = ROBOT_FIGHT_ATTACK_MOVE;
                    robot_state = ROBOT_CLIMB_ROTATE_TO_DIR;
                    init_rotate_pd = 0;
                    printf("[FightAttack] Steering to enemy at relative %.1f (target absolute %.1f)...\r\n", t_angle, climb_target_angle);
                }
            } else {
                // 新激光测距版边缘巡台与检测（当激光检测距离大于260时判定为边缘）
                Auto_Control_Logic_Laser(laser_dist_1, laser_dist_2, Grey_Front, Grey_Left, Grey_Right, Grey_Back);     //自动巡台
                Detect_Laser(&global_vision_target, &global_vision_yaw, &IR_Distance_F, &laser_dist_1, &laser_dist_2, &Grey_Front);  //自动检测能量块并推下
            }
        }
        

        last_buttons = ctrl.buttons;
    }



    osDelay(20);
  }
  /* USER CODE END StartMotion_Task */
}

/* USER CODE BEGIN Header_StartAngle_Task */
/**
* @brief Function implementing the Angle_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAngle_Task */
void StartAngle_Task(void *argument)
{
  /* USER CODE BEGIN StartAngle_Task */
  
  /* 初始化角度控制 PID（Yaw 偏航角控制）
     Kp=0.8, Ki=0.15, Kd=0.05：需根据实际机械动�?�调�???
     MaxOut=100：最大电机�?�度指令
     MaxIOut=50：最大积分输出
     DeadBand=2：接近目标时的死区（防止抖动）
     I_Separation=10：积分分离阈值 */
  PID_Init(&Angle_pid[0], 0.8f, 0.15f, 0.05f, 100.0f, 50.0f, 2.0f, 10.0f);
  
  /* Infinite loop */
  for(;;)
  {
    /* 检查状态变化：从禁用切换到启用 */
    if (angle_control_enabled && !angle_control_prev) {
      /* 刚启用，清除 PID 积分防止突跳，但保留上一周期的控制平稳性 */
      PID_Clear(&Angle_pid[0]);
    }
    angle_control_prev = angle_control_enabled;
        /* 清除上一周期的角度环转向修正 */
    for (int i = 0; i < 4; i++) {
      motor_angle_speed[i] = 0.0f;
    }
        /* 仅当角度环被启用时执行控制 */
    if (angle_control_enabled) {
      /* 从 IMU 读取当前偏航角作为反馈 */
      current_angle = IMU_Data.Yaw;
      
      /* PID 计算：计算达到目标偏航角�???�???的电机�?�度 
         由于启用时 target_angle == current_angle，首个周期 PID 输出会接近 0
         这是正常的 - 用户手动启用时应该是静止状态�，然后再�?�过摇杆操作改变目标角度 */
      float angle_output = PID_Calc(&Angle_pid[0], target_angle, current_angle);
      

      /*
      motor_angle_speed[0] = angle_output;   
      motor_angle_speed[1] = -angle_output;  
      motor_angle_speed[2] = angle_output;   
      motor_angle_speed[3] = -angle_output;  
      */

      /* 打印调试信息 */
      //printf("Yaw Control: Target=%.2f°, Current=%.2f°, Output=%.1f\r\n", target_angle, current_angle, angle_output);

    }
    
    /* 50ms 周期与电机控制任务同步 */
    osDelay(50);
  }
  /* USER CODE END StartAngle_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Trigger_Debug_Launch(void)
{
  /* 串口输入 Y / y 启动自动巡台测试模式 */
  is_test_mode = 1;             /* 标记为测试模式以绕过遥控离线检查 */
  control_mode = 1;             /* 强制切换到自动控制模式 */
  robot_state = ROBOT_RUNNING;  /* 设为常规运行状态，以便进入自动巡台逻辑 */
  printf("[SerialTrigger] Y received! Starting Auto Patrol Laser Test...\r\n");
}
/* USER CODE END Application */

