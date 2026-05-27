#ifndef __MOTION_H
#define __MOTION_H

#include "main.h"

void Auto_Control_Logic(uint8_t sw1, uint8_t sw3, float grey_front,float grey_left,float grey_right,float grey_back);
void Detect(volatile uint8_t *target, volatile float *yaw, volatile float *distance_front, volatile uint8_t *SW_L, volatile uint8_t *SW_R, volatile float *grey_front);

#endif /* __MOTION_H */