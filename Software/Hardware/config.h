#ifndef CONFIG_H
#define CONFIG_H

#define IR_Sensor_L sw1                 // 左侧光电传感器
#define IR_Sensor_R sw3                 // 右侧光电传感器

#define IR_Distance_F ir_distance[0]    // 前方距离传感器
#define IR_Distance_FL ir_distance[1]   // 前左距离传感器
#define IR_Distance_FR ir_distance[2]   // 前右距离传感器   
#define IR_Distance_L ir_distance[8]    // 左侧距离传感器（数据飘得厉害）
#define IR_Distance_R ir_distance[6]    // 右侧距离传感器
#define IR_Distance_BL ir_distance[3]   // 后左距离传感器
#define IR_Distance_BR ir_distance[4]   // 后右距离传感器
#define IR_Distance_B ir_distance[5]    // 后方距离传感器
#define IR_Distance_UP ir_distance[7]   // 上方距离传感器

#define Grey_Front grey_value[1]       // 前方灰度传感器
#define Grey_Back grey_value[3]        // 后方灰度传感器
#define Grey_Left grey_value[2]        // 左侧灰度传感器
#define Grey_Right grey_value[0]       // 右侧灰度传感器

// 新增两路 TOF200C 激光测距全局变量声明
extern uint16_t laser_dist_1;          // 激光测距 1 (SCL=PD0, SDA=PE3)
extern uint16_t laser_dist_2;          // 激光测距 2 (SCL=PD10, SDA=PD14)

#endif /* CONFIG_H */

