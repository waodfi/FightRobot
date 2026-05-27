#ifndef __IR_SENSOR_H
#define __IR_SENSOR_H

#include "stm32h7xx_hal.h"

typedef enum {
    IR_SENSOR_1 = 0, // PE3
    IR_SENSOR_2,     // PD0
    IR_SENSOR_3,     // PD10
    IR_SENSOR_4      // PD14
} IR_Sensor_Channel;

void IR_Sensor_Init(void);
uint8_t IR_Sensor_Read(IR_Sensor_Channel ch);

#endif // __IR_SENSOR_H
