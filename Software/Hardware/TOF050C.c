#include "TOF050C.h"
#include <stdio.h>
#include "vl53l0x.h"
#include "SoftI2C.h"
#include "cmsis_os.h" // For osDelay

static vl53l0x_dev_t vl53l0x_dev1;
static vl53l0x_ll_t vl53l0x_ll1;
static uint8_t dev1_initialized = 0;
static uint8_t dev1_fail_count = 0;

static vl53l0x_dev_t vl53l0x_dev2;
static vl53l0x_ll_t vl53l0x_ll2;
static uint8_t dev2_initialized = 0;
static uint8_t dev2_fail_count = 0;

#define MAX_CONSECUTIVE_FAILS 5

// Delay implementation
static void delay_ms_imp(uint32_t ms) {
    osDelay(ms);
}

// I2C Write multi helper
static void i2c_write_multi(SoftI2C_Bus_e bus, uint8_t reg, uint8_t *data, uint16_t len) {
    SoftI2C_Start(bus);
    SoftI2C_WriteByte(bus, TOF050C_ADDR_WRITE);
    SoftI2C_WaitAck(bus);
    SoftI2C_WriteByte(bus, reg);
    SoftI2C_WaitAck(bus);
    for (uint16_t i = 0; i < len; i++) {
        SoftI2C_WriteByte(bus, data[i]);
        SoftI2C_WaitAck(bus);
    }
    SoftI2C_Stop(bus);
}

// I2C Read multi helper
static void i2c_read_multi(SoftI2C_Bus_e bus, uint8_t reg, uint8_t *data, uint16_t len) {
    SoftI2C_Start(bus);
    SoftI2C_WriteByte(bus, TOF050C_ADDR_WRITE);
    SoftI2C_WaitAck(bus);
    SoftI2C_WriteByte(bus, reg);
    SoftI2C_WaitAck(bus);
    
    SoftI2C_Start(bus);
    SoftI2C_WriteByte(bus, TOF050C_ADDR_READ);
    SoftI2C_WaitAck(bus);
    for (uint16_t i = 0; i < len; i++) {
        data[i] = SoftI2C_ReadByte(bus, (i == len - 1) ? 0 : 1);
    }
    SoftI2C_Stop(bus);
}

// BUS 1 implementations
static void write_reg_bus1(uint8_t reg, uint8_t val) {
    uint8_t temp = val;
    i2c_write_multi(SOFT_I2C_BUS_1, reg, &temp, 1);
}
static void write_reg_16_bus1(uint8_t reg, uint16_t val) {
    uint8_t temp[2];
    temp[0] = (val >> 8) & 0xFF;
    temp[1] = val & 0xFF;
    i2c_write_multi(SOFT_I2C_BUS_1, reg, temp, 2);
}
static void write_reg_32_bus1(uint8_t reg, uint32_t val) {
    uint8_t temp[4];
    temp[0] = (val >> 24) & 0xFF;
    temp[1] = (val >> 16) & 0xFF;
    temp[2] = (val >> 8) & 0xFF;
    temp[3] = val & 0xFF;
    i2c_write_multi(SOFT_I2C_BUS_1, reg, temp, 4);
}
static void write_reg_multi_bus1(uint8_t reg, uint8_t *src_buf, size_t count) {
    i2c_write_multi(SOFT_I2C_BUS_1, reg, src_buf, count);
}
static uint8_t read_reg_bus1(uint8_t reg) {
    uint8_t temp = 0;
    i2c_read_multi(SOFT_I2C_BUS_1, reg, &temp, 1);
    return temp;
}
static uint16_t read_reg_16_bus1(uint8_t reg) {
    uint8_t temp[2] = {0};
    i2c_read_multi(SOFT_I2C_BUS_1, reg, temp, 2);
    return (temp[0] << 8) | temp[1];
}
static uint32_t read_reg_32_bus1(uint8_t reg) {
    uint8_t temp[4] = {0};
    i2c_read_multi(SOFT_I2C_BUS_1, reg, temp, 4);
    return ((uint32_t)temp[0] << 24) | ((uint32_t)temp[1] << 16) | ((uint32_t)temp[2] << 8) | temp[3];
}
static void read_reg_multi_bus1(uint8_t reg, uint8_t *dst_buf, size_t count) {
    i2c_read_multi(SOFT_I2C_BUS_1, reg, dst_buf, count);
}

// BUS 2 implementations
static void write_reg_bus2(uint8_t reg, uint8_t val) {
    uint8_t temp = val;
    i2c_write_multi(SOFT_I2C_BUS_2, reg, &temp, 1);
}
static void write_reg_16_bus2(uint8_t reg, uint16_t val) {
    uint8_t temp[2];
    temp[0] = (val >> 8) & 0xFF;
    temp[1] = val & 0xFF;
    i2c_write_multi(SOFT_I2C_BUS_2, reg, temp, 2);
}
static void write_reg_32_bus2(uint8_t reg, uint32_t val) {
    uint8_t temp[4];
    temp[0] = (val >> 24) & 0xFF;
    temp[1] = (val >> 16) & 0xFF;
    temp[2] = (val >> 8) & 0xFF;
    temp[3] = val & 0xFF;
    i2c_write_multi(SOFT_I2C_BUS_2, reg, temp, 4);
}
static void write_reg_multi_bus2(uint8_t reg, uint8_t *src_buf, size_t count) {
    i2c_write_multi(SOFT_I2C_BUS_2, reg, src_buf, count);
}
static uint8_t read_reg_bus2(uint8_t reg) {
    uint8_t temp = 0;
    i2c_read_multi(SOFT_I2C_BUS_2, reg, &temp, 1);
    return temp;
}
static uint16_t read_reg_16_bus2(uint8_t reg) {
    uint8_t temp[2] = {0};
    i2c_read_multi(SOFT_I2C_BUS_2, reg, temp, 2);
    return (temp[0] << 8) | temp[1];
}
static uint32_t read_reg_32_bus2(uint8_t reg) {
    uint8_t temp[4] = {0};
    i2c_read_multi(SOFT_I2C_BUS_2, reg, temp, 4);
    return ((uint32_t)temp[0] << 24) | ((uint32_t)temp[1] << 16) | ((uint32_t)temp[2] << 8) | temp[3];
}
static void read_reg_multi_bus2(uint8_t reg, uint8_t *dst_buf, size_t count) {
    i2c_read_multi(SOFT_I2C_BUS_2, reg, dst_buf, count);
}

static uint32_t last_init_tick1 = 0;
static uint32_t last_init_tick2 = 0;
#define INIT_RETRY_DELAY_MS 2000

void TOF050C_Init(SoftI2C_Bus_e bus)
{
    // SoftI2C init
    SoftI2C_Init(bus);
    
    if (bus == SOFT_I2C_BUS_1) {
        // Debug read
        uint8_t id1 = read_reg_bus1(0xC0);
        printf("Bus 1 (Laser 1) Reg 0xC0: 0x%02X (Expected: 0xEE)\r\n", id1);
        
        vl53l0x_ll1.delay_ms = delay_ms_imp;
        vl53l0x_ll1.i2c_write_reg = write_reg_bus1;
        vl53l0x_ll1.i2c_write_reg_16bit = write_reg_16_bus1;
        vl53l0x_ll1.i2c_write_reg_32bit = write_reg_32_bus1;
        vl53l0x_ll1.i2c_write_reg_multi = write_reg_multi_bus1;
        vl53l0x_ll1.i2c_read_reg = read_reg_bus1;
        vl53l0x_ll1.i2c_read_reg_16bit = read_reg_16_bus1;
        vl53l0x_ll1.i2c_read_reg_32bit = read_reg_32_bus1;
        vl53l0x_ll1.i2c_read_reg_multi = read_reg_multi_bus1;
        vl53l0x_ll1.xshut_set = NULL;
        vl53l0x_ll1.xshut_reset = NULL;
        
        vl53l0x_dev1.ll = &vl53l0x_ll1;
        vl53l0x_dev1.gpio_func = VL53L0X_GPIO_FUNC_NEW_MEASURE_READY;
        
        last_init_tick1 = HAL_GetTick();
        if (vl53l0x_init(&vl53l0x_dev1) == VL53L0X_OK) {
            dev1_initialized = 1;
            vl53l0x_start_continuous_measurements(&vl53l0x_dev1); // 开启硬件连续测量模式
            printf("Bus 1 (Laser 1) Init OK and continuous started\r\n");
        } else {
            dev1_initialized = 0;
            printf("Bus 1 (Laser 1) Init FAIL\r\n");
        }
    } else {
        // Debug read
        uint8_t id2 = read_reg_bus2(0xC0);
        printf("Bus 2 (Laser 2) Reg 0xC0: 0x%02X (Expected: 0xEE)\r\n", id2);
        
        vl53l0x_ll2.delay_ms = delay_ms_imp;
        vl53l0x_ll2.i2c_write_reg = write_reg_bus2;
        vl53l0x_ll2.i2c_write_reg_16bit = write_reg_16_bus2;
        vl53l0x_ll2.i2c_write_reg_32bit = write_reg_32_bus2;
        vl53l0x_ll2.i2c_write_reg_multi = write_reg_multi_bus2;
        vl53l0x_ll2.i2c_read_reg = read_reg_bus2;
        vl53l0x_ll2.i2c_read_reg_16bit = read_reg_16_bus2;
        vl53l0x_ll2.i2c_read_reg_32bit = read_reg_32_bus2;
        vl53l0x_dev2.ll = &vl53l0x_ll2;
        vl53l0x_ll2.i2c_read_reg_multi = read_reg_multi_bus2;
        vl53l0x_ll2.xshut_set = NULL;
        vl53l0x_ll2.xshut_reset = NULL;
        
        vl53l0x_dev2.ll = &vl53l0x_ll2;
        vl53l0x_dev2.gpio_func = VL53L0X_GPIO_FUNC_NEW_MEASURE_READY;
        
        last_init_tick2 = HAL_GetTick();
        if (vl53l0x_init(&vl53l0x_dev2) == VL53L0X_OK) {
            dev2_initialized = 1;
            vl53l0x_start_continuous_measurements(&vl53l0x_dev2); // 开启硬件连续测量模式
            printf("Bus 2 (Laser 2) Init OK and continuous started\r\n");
        } else {
            dev2_initialized = 0;
            printf("Bus 2 (Laser 2) Init FAIL\r\n");
        }
    }
}

uint16_t TOF050C_ReadDistance(SoftI2C_Bus_e bus)
{
    uint16_t range = 0xFFFF;
    uint32_t current_tick = HAL_GetTick();
    
    if (bus == SOFT_I2C_BUS_1) {
        if (!dev1_initialized) {
            if (current_tick - last_init_tick1 < INIT_RETRY_DELAY_MS) {
                return 0xFFFF;
            }
            last_init_tick1 = current_tick;
            if (vl53l0x_init(&vl53l0x_dev1) == VL53L0X_OK) {
                dev1_initialized = 1;
                dev1_fail_count = 0;
                vl53l0x_start_continuous_measurements(&vl53l0x_dev1);
                last_init_tick1 = current_tick; // 记录成功初始化的时刻作为时间基准
                printf("Laser 1 re-initialized successfully\r\n");
            } else {
                return 0xFFFF;
            }
        }
        // 瞬间读取连续测量结果寄存器（不含任何等待和阻塞，耗时 < 1ms）
        vl53l0x_get_range_mm_continuous(&vl53l0x_dev1, &range);

        // 自愈判定：如果读到 0、8191 或 0xFFFF，说明测量失效或总线异常
        if (range == 0 || range == 8191 || range == 0xFFFF) {
            // 在刚初始化的 300ms 内，忽略 8191，允许芯片完成首个周期的转换；对于 0 和 0xFFFF（总线锁死/断线）依然即时处理
            if (range != 8191 || (current_tick - last_init_tick1 > 300)) {
                dev1_fail_count++;
                if (dev1_fail_count >= MAX_CONSECUTIVE_FAILS) {
                    // 连续多次失效，执行总线恢复并标记为未初始化以触发重新初始化
                    SoftI2C_Recover(SOFT_I2C_BUS_1);
                    dev1_initialized = 0;
                    last_init_tick1 = current_tick;
                    printf("Laser 1 consecutive failures (%d). Lockup detected (range: %d). Triggering recovery...\r\n", dev1_fail_count, range);
                    dev1_fail_count = 0;
                }
            }
            return 0xFFFF; // 失效时返回最大值，触发边缘防跌落保护，确保安全
        } else {
            dev1_fail_count = 0;
        }
    } else {
        if (!dev2_initialized) {
            if (current_tick - last_init_tick2 < INIT_RETRY_DELAY_MS) {
                return 0xFFFF;
            }
            last_init_tick2 = current_tick;
            if (vl53l0x_init(&vl53l0x_dev2) == VL53L0X_OK) {
                dev2_initialized = 1;
                dev2_fail_count = 0;
                vl53l0x_start_continuous_measurements(&vl53l0x_dev2);
                last_init_tick2 = current_tick; // 记录成功初始化的时刻作为时间基准
                printf("Laser 2 re-initialized successfully\r\n");
            } else {
                return 0xFFFF;
            }
        }
        // 瞬间读取连续测量结果寄存器（不含任何等待和阻塞，耗时 < 1ms）
        vl53l0x_get_range_mm_continuous(&vl53l0x_dev2, &range);

        // 自愈判定：如果读到 0、8191 或 0xFFFF，说明测量失效或总线异常
        if (range == 0 || range == 8191 || range == 0xFFFF) {
            // 在刚初始化的 300ms 内，忽略 8191，允许芯片完成首个周期的转换；对于 0 和 0xFFFF（总线锁死/断线）依然即时处理
            if (range != 8191 || (current_tick - last_init_tick2 > 300)) {
                dev2_fail_count++;
                if (dev2_fail_count >= MAX_CONSECUTIVE_FAILS) {
                    // 连续多次失效，执行总线恢复并标记为未初始化以触发重新初始化
                    SoftI2C_Recover(SOFT_I2C_BUS_2);
                    dev2_initialized = 0;
                    last_init_tick2 = current_tick;
                    printf("Laser 2 consecutive failures (%d). Lockup detected (range: %d). Triggering recovery...\r\n", dev2_fail_count, range);
                    dev2_fail_count = 0;
                }
            }
            return 0xFFFF; // 失效时返回最大值，触发边缘防跌落保护，确保安全
        } else {
            dev2_fail_count = 0;
        }
    }
    return range;
}