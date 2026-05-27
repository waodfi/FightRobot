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

  /* 在任务初始化中开启TIM2�?????4个�?�道捕获中断 */
HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

  /* PID 初始化 (电机速度控制) 
     增加: 误差绝对值死区阈值(如 2.0f)，积分分离阈值(如 30.0f)
     当绝对�?�度离目标超过 30 时切断积分防超调；相差 2 以内视为到达目标平息抖动 */
  for(int i = 0; i < 4; i++) {
      // (pid, Kp, Ki, Kd, MaxOut, MaxIOut, DeadBand, I_Separation)
      // 积分分离阈值 500.0f：防止启动时积分饱和；误差 < 500 时 P 项可能不足，积分逐步参与
      // 纯 P 控制(Kp=0.65)可能难以达到目标，积分项帮助消除稳�?�误�???
      PID_Init(&motor_pid[i], 0.65f, 0.20f, 0.02f, 100.0f, 100.0f, 1.0f, 500.0f); 
  }

  uint32_t last_pulse_count[4] = {0};
  float filtered_speed[4] = {0.0f}; // 新增：用于平滑测速跳动的低�?�滤波数�???
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
          output = PID_Calc(&motor_pid[i], motor_target_speed[i], motor_rpm_norm[i]);
      }

      /* 4. 更新电机 PWM 输出 (-100~100) 
            下层 Motor_SetSpeed 会拆分符号控制反转引脚，和大小控制PWM */
      Motor_SetSpeed(MOTOR_1 + i, output); 
    }

        /* 降低打印频率至 10Hz，加快上位机显示刷新 */
        if (HAL_GetTick() - motor_print_tick >= 100U) {
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
      /* 将多行传感器输出降频至 10Hz，加快上位机显示 */
      if (HAL_GetTick() - comm_print_tick >= 100U) {
        comm_print_tick = HAL_GetTick();
        printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, Temp: %.2f\r\n", IMU_Data.Roll, IMU_Data.Pitch, IMU_Data.Yaw, IMU_Data.Temp);
        printf("IR Dist: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\r\n",
               ir_distance[0], ir_distance[1], ir_distance[2],
               ir_distance[3], ir_distance[4], ir_distance[5],
               ir_distance[6], ir_distance[7], ir_distance[8]);
        printf("Grey(F,B,L,R): %.1f, %.1f, %.1f, %.1f\r\n",grey_value[0], grey_value[1], grey_value[2], grey_value[3]);

        printf("Photoelectric(SW1~SW4): %d, %d, %d, %d\r\n", sw1, sw2, sw3, sw4);
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
  IR_Sensor_Init();
  /* Infinite loop */
  for(;;)
  {
    /* 先计算前4路灰度值 */
    Grey_CalculateValues(&adc_raw_data[0], grey_value);

    /* 计算红外测距 */
    IR_CalculateDistances(&adc_raw_data[4], ir_distance);

    /* 计算光电开关状态 */
    sw1 = IR_Sensor_Read(IR_SENSOR_1);
    sw2 = IR_Sensor_Read(IR_SENSOR_2);
    sw3 = IR_Sensor_Read(IR_SENSOR_3);
    sw4 = IR_Sensor_Read(IR_SENSOR_4);

    /* 向队列发送触发标志给 Comm_Task */
    osMessageQueuePut(IMU_Rx_QueueHandle, &trigger_msg, 0, 0);
    
    /* 延迟100ms */
    osDelay(100);
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
  
  uint16_t last_servo_angle = 90;  /* 记录上一个有效的舵机角度 */
  const uint16_t servo_step = 2;    /* 固定步进角度：每个周期移动2度 */

  
  /* Infinite loop */
  for(;;)
  {
    if (!Control_IsOnline()) {
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
            /* 清空摇杆和角度�?�度 */
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

            Auto_Control_Logic(sw1, sw3, Grey_Front,Grey_Left,Grey_Right,Grey_Back);     //自动巡台
            Detect(&global_vision_target, &global_vision_yaw,&IR_Distance_F, &IR_Sensor_L, &IR_Sensor_R,&Grey_Front);  //自动检测能量块并推下
            
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

/* USER CODE END Application */

