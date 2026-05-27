/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "quadspi.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "IMU.h"
#include "control.h"
#include "Motor.h"
#include "IR_Sensor.h"
#include "Machine_Vision.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define IMU_RX_BUFFER_SIZE  64     // 长度足够容纳至少几帧数据 (1帧是11字节)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t motor_capture_val[4] = {0}; // 记录每个通道�??新的计数�??
uint32_t motor_pulse_diff[4] = {0};  // 两个上升沿之间的差�??
uint32_t motor_pulse_count[4] = {0}; // 累计脉冲�??

uint32_t imu_rx_byte_count = 0;
__attribute__((aligned(32))) uint8_t imu_rx_buffer[IMU_RX_BUFFER_SIZE];

uint16_t adc_raw_data[13];
float grey_value[4];
float ir_distance[9];

uint8_t sw1, sw2, sw3, sw4; // 4???????

extern float motor_target_speed[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern osMessageQueueId_t UART1_Rx_QueueHandle;
extern osMessageQueueId_t Vision_Rx_QueueHandle;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_QUADSPI_Init();
  MX_RTC_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw_data, 13);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, imu_rx_buffer, IMU_RX_BUFFER_SIZE);
  MachineVision_Init(&huart2);

  Control_Init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#if defined ( __CC_ARM ) || defined (__ARMCC_VERSION) 
#pragma import(__use_no_semihosting)
struct __FILE {
    int handle;
};
FILE __stdout;
FILE __stdin;
FILE __stderr;
void _sys_exit(int x) {
    x = x;
}
void _ttywrch(int ch) {
}
#endif

int fputc(int ch, FILE *f)
{
    uint8_t temp = (uint8_t)ch;
    HAL_UART_Transmit(&huart1, &temp, 1, HAL_MAX_DELAY);
    return ch;
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{    // 调用遥控器解析�?�辑(USART1在里面会被识别处�??)
    Control_UART_RxCallback(huart, Size);
  MachineVision_RxEventCallback(huart, Size);

  if ((huart->Instance == USART2) && (Vision_Rx_QueueHandle != NULL))
  {
    uint32_t notify_msg = 1;
    (void)osMessageQueuePut(Vision_Rx_QueueHandle, &notify_msg, 0, 0);
  }

    static uint16_t old_pos = 0;
    if (huart->Instance == USART3)  // 匹配 IMU 的串�?????
    {
        /* Invalidate D-Cache to ensure CPU reads the latest data written by DMA */
        SCB_InvalidateDCache_by_Addr((uint32_t *)((uint32_t)imu_rx_buffer & ~0x1F), (IMU_RX_BUFFER_SIZE + 31) & ~0x1F);

        /* 遍历 DMA 刚刚搬运完的这批字节，挨个喂给状态机去解�????? */
        uint16_t new_pos = Size;
        /* Calculate how many bytes have been received since last callback */
        uint16_t length = (new_pos + IMU_RX_BUFFER_SIZE - old_pos) % IMU_RX_BUFFER_SIZE;
        /* If TC interrupt sets Size to IMU_RX_BUFFER_SIZE and old_pos is 0, length would be 0, so handle full buffer */
        if (length == 0 && new_pos != old_pos)
        {
            length = IMU_RX_BUFFER_SIZE;
        }
        
        for (uint16_t i = 0; i < length; i++)
        {
            IMU_ReceiveByte(imu_rx_buffer[old_pos]);
            old_pos++;
            if (old_pos >= IMU_RX_BUFFER_SIZE) 
            {
                old_pos = 0;
            }
        }
    }
}

// 这个回调函数�??? HAL 库在 TIM2 捕获事件发生时调用，负责计算每个电机的脉冲差值和累计脉冲�???
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        uint32_t current_val = 0;
        
        switch (htim->Channel) 
        {
            case HAL_TIM_ACTIVE_CHANNEL_1: // 电机 4 (MOTOR_4) -> 数组索引 3
                current_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                motor_pulse_diff[3] = current_val - motor_capture_val[3];
                motor_capture_val[3] = current_val;
                motor_pulse_count[3]++; // 累计脉冲
                break;
                
            case HAL_TIM_ACTIVE_CHANNEL_2: // 电机 1 (MOTOR_1) -> 数组索引 0
                current_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
                motor_pulse_diff[0] = current_val - motor_capture_val[0];
                motor_capture_val[0] = current_val;
                motor_pulse_count[0]++; // 累计脉冲
                break;
                
            case HAL_TIM_ACTIVE_CHANNEL_3: // 电机 2 (MOTOR_2) -> 数组索引 1
                current_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
                motor_pulse_diff[1] = current_val - motor_capture_val[1];
                motor_capture_val[1] = current_val;
                motor_pulse_count[1]++; // 累计脉冲
                break;
                
            case HAL_TIM_ACTIVE_CHANNEL_4: // 电机 3 (MOTOR_3) -> 数组索引 2
                current_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
                motor_pulse_diff[2] = current_val - motor_capture_val[2];
                motor_capture_val[2] = current_val;
                motor_pulse_count[2]++; // 累计脉冲
                break;
                
            default:
                break;
        }
    }
}


void IMU_HW_SerialSend(uint8_t *data, uint16_t len)
{
  HAL_UART_Transmit(&huart3, data, len, 100);
}


/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_HFNMI_PRIVDEF);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
