#include "IR_Sensor.h"

void IR_Sensor_Init(void)
{
    /* 
     * GPIO已在CubeMX(gpio.c中的MX_GPIO_Init)中初始化配置为Input。
     * 此处无需重复初始化，保留该函数仅为后续可能扩展结构体或变量初始化做准备。
     */
}

uint8_t IR_Sensor_Read(IR_Sensor_Channel ch)
{
    switch (ch) {
        case IR_SENSOR_1: // PE3
            return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);
        case IR_SENSOR_2: // PD0
            return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0);
        case IR_SENSOR_3: // PD10
            return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10);
        case IR_SENSOR_4: // PD14
            return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14);
        default:
            return 0;
    }
}
