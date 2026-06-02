#include "TOF050C.h"

void TOF050C_Init(SoftI2C_Bus_e bus)
{
    // 初始化 I2C 总线引脚状态
    SoftI2C_Init(bus);
    
    // 该智能模块上电默认自动连续测距，无需发送复杂的寄存器初始化序列。
}

uint16_t TOF050C_ReadDistance(SoftI2C_Bus_e bus)
{
    uint8_t dist_high = 0;
    uint8_t dist_low = 0;
    uint8_t ack;

    // 第一步：写入寄存器地址
    SoftI2C_Start(bus);
    SoftI2C_WriteByte(bus, TOF050C_ADDR_WRITE);
    ack = SoftI2C_WaitAck(bus);
    if (ack != 0) {
        SoftI2C_Stop(bus);
        return 0xFFFF;
    }

    SoftI2C_WriteByte(bus, 0x00);
    ack = SoftI2C_WaitAck(bus);
    if (ack != 0) {
        SoftI2C_Stop(bus);
        return 0xFFFF;
    }

    // 第二步：读2字节
    SoftI2C_Start(bus);
    SoftI2C_WriteByte(bus, TOF050C_ADDR_READ);
    SoftI2C_WaitAck(bus);

    dist_high = SoftI2C_ReadByte(bus, 1);
    dist_low  = SoftI2C_ReadByte(bus, 0);
    SoftI2C_Stop(bus);

    uint16_t distance_mm = (dist_high << 8) | dist_low;
    return distance_mm;
}