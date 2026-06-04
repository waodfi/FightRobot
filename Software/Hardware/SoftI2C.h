#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h" /* For STM32 HAL GPIO defines */
#include <stdint.h>

/**
 * @brief Software I2C Bus IDs
 */
typedef enum {
    SOFT_I2C_BUS_1 = 0, // PE3 (SCL), PD0  (SDA)
    SOFT_I2C_BUS_2 = 1  // PD10(SCL), PD14 (SDA)
} SoftI2C_Bus_e;

/* I2C APIs */
void SoftI2C_Init(SoftI2C_Bus_e bus);
void SoftI2C_Start(SoftI2C_Bus_e bus);
void SoftI2C_Stop(SoftI2C_Bus_e bus);

uint8_t SoftI2C_WaitAck(SoftI2C_Bus_e bus);
void SoftI2C_Ack(SoftI2C_Bus_e bus);
void SoftI2C_NAck(SoftI2C_Bus_e bus);

void SoftI2C_WriteByte(SoftI2C_Bus_e bus, uint8_t data);
uint8_t SoftI2C_ReadByte(SoftI2C_Bus_e bus, uint8_t ack);
void SoftI2C_Recover(SoftI2C_Bus_e bus);

#ifdef __cplusplus
}
#endif

#endif /* __SOFT_I2C_H */