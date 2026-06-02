#include "SoftI2C.h"

// ==========================================
// 硬件引脚定义 (完全使用原红外光电开关对应的引脚)
// ==========================================
// BUS 1
#define SCL1_PORT   GPIOE
#define SCL1_PIN    GPIO_PIN_3
#define SDA1_PORT   GPIOD
#define SDA1_PIN    GPIO_PIN_0

// BUS 2
#define SCL2_PORT   GPIOD
#define SCL2_PIN    GPIO_PIN_10
#define SDA2_PORT   GPIOD
#define SDA2_PIN    GPIO_PIN_14

/**
 * @brief  微秒级粗略延时，用于I2C时序
 */
static void SoftI2C_Delay(void)
{
    for(volatile uint32_t i = 0; i < 40; i++) {
        __NOP();
    }
}

static inline void SoftI2C_SetSCL(SoftI2C_Bus_e bus, uint8_t level)
{
    if (bus == SOFT_I2C_BUS_1) {
        HAL_GPIO_WritePin(SCL1_PORT, SCL1_PIN, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(SCL2_PORT, SCL2_PIN, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static inline void SoftI2C_SetSDA(SoftI2C_Bus_e bus, uint8_t level)
{
    if (bus == SOFT_I2C_BUS_1) {
        HAL_GPIO_WritePin(SDA1_PORT, SDA1_PIN, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(SDA2_PORT, SDA2_PIN, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static inline uint8_t SoftI2C_ReadSDA(SoftI2C_Bus_e bus)
{
    if (bus == SOFT_I2C_BUS_1) {
        return (HAL_GPIO_ReadPin(SDA1_PORT, SDA1_PIN) == GPIO_PIN_SET) ? 1 : 0;
    } else {
        return (HAL_GPIO_ReadPin(SDA2_PORT, SDA2_PIN) == GPIO_PIN_SET) ? 1 : 0;
    }
}

// === 下方为标准 I2C 协议实现 ===

void SoftI2C_Init(SoftI2C_Bus_e bus)
{
    SoftI2C_SetSCL(bus, 1);
    SoftI2C_SetSDA(bus, 1);
    SoftI2C_Delay();
}

void SoftI2C_Start(SoftI2C_Bus_e bus)
{
    SoftI2C_SetSDA(bus, 1);
    SoftI2C_SetSCL(bus, 1);
    SoftI2C_Delay();
    SoftI2C_SetSDA(bus, 0); 
    SoftI2C_Delay();
    SoftI2C_SetSCL(bus, 0); 
}

void SoftI2C_Stop(SoftI2C_Bus_e bus)
{
    SoftI2C_SetSCL(bus, 0);
    SoftI2C_SetSDA(bus, 0);
    SoftI2C_Delay();
    SoftI2C_SetSCL(bus, 1); 
    SoftI2C_Delay();
    SoftI2C_SetSDA(bus, 1); 
    SoftI2C_Delay();
}

uint8_t SoftI2C_WaitAck(SoftI2C_Bus_e bus)
{
    uint8_t ack_status;
    SoftI2C_SetSDA(bus, 1); 
    SoftI2C_Delay();
    SoftI2C_SetSCL(bus, 1);
    SoftI2C_Delay();
    ack_status = SoftI2C_ReadSDA(bus);
    SoftI2C_SetSCL(bus, 0);
    SoftI2C_Delay();
    return ack_status; // 0为应答(有效)
}

void SoftI2C_Ack(SoftI2C_Bus_e bus)
{
    SoftI2C_SetSCL(bus, 0);
    SoftI2C_SetSDA(bus, 0); 
    SoftI2C_Delay();
    SoftI2C_SetSCL(bus, 1);
    SoftI2C_Delay();
    SoftI2C_SetSCL(bus, 0);
    SoftI2C_SetSDA(bus, 1); 
}

void SoftI2C_NAck(SoftI2C_Bus_e bus)
{
    SoftI2C_SetSCL(bus, 0);
    SoftI2C_SetSDA(bus, 1); 
    SoftI2C_Delay();
    SoftI2C_SetSCL(bus, 1);
    SoftI2C_Delay();
    SoftI2C_SetSCL(bus, 0);
}

void SoftI2C_WriteByte(SoftI2C_Bus_e bus, uint8_t data)
{
    for (uint8_t t = 0; t < 8; t++)
    {
        SoftI2C_SetSCL(bus, 0);
        SoftI2C_SetSDA(bus, (data & 0x80) ? 1 : 0);
        data <<= 1;
        SoftI2C_Delay();
        SoftI2C_SetSCL(bus, 1); 
        SoftI2C_Delay();
    }
    SoftI2C_SetSCL(bus, 0);   
}

uint8_t SoftI2C_ReadByte(SoftI2C_Bus_e bus, uint8_t ack)
{
    uint8_t receive_data = 0;
    SoftI2C_SetSDA(bus, 1); 
    
    for (uint8_t t = 0; t < 8; t++)
    {
        SoftI2C_SetSCL(bus, 0);
        SoftI2C_Delay();
        SoftI2C_SetSCL(bus, 1);
        
        receive_data <<= 1;
        if (SoftI2C_ReadSDA(bus)) {
            receive_data++;
        }
        SoftI2C_Delay();
    }
    SoftI2C_SetSCL(bus, 0); 
    if (ack) SoftI2C_Ack(bus);
    else SoftI2C_NAck(bus);
    
    return receive_data;
}