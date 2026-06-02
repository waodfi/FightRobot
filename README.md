以下是该项目的核心架构、文件组织以及工作流的完整解析：

📂 目录结构与模块划分


FightRobot/
├── Core/ & Drivers/          # STM32CubeMX 生成的 HAL 驱动与底层核心文件
├── MDK-ARM/                  # Keil v5 工程目录
├── FightRobot.ioc            # STM32CubeMX 配置文件（基于 STM32H7 系列）
├── Software/                 # 核心控制与算法（下位机灵魂）
│   ├── Hardware/             # 硬件驱动适配层
│   │   ├── config.h          # 传感器引脚与通道映射映射表
│   │   ├── Motor.c/.h        # 四路直流减速电机驱动与脉冲计数
│   │   ├── Control.c/.h      # 上位机遥控数据接收与解析 (UART DMA + IDLE)
│   │   ├── Machine_Vision.c  # 视觉串口通信解析器 (支持 CRC-8/Dallas 校验)
│   │   ├── IMU.c/.h          # 维特智能(WitMotion)九轴姿态传感器解析与清零
│   │   ├── TOF050C.c/.h      # 高精度 TOF 激光测距传感器驱动
│   │   ├── SoftI2C.c/.h      # 软件模拟 I2C 总线驱动 (PE3/PD0 等引脚)
│   │   ├── Grey.c/.h         # 灰度传感器采集 (用于边缘/白线检测)
│   │   ├── IR.c/.h & Servo.c # 模拟红外测距、光电开关以及舵机控制
│   └── PID/                  # 控制算法层
│       └── PID.c/.h          # 包含死区、积分分离与抗饱和的位置式 PID 算法
└── Vision/                   # 上位机交互与 3D 监控（Python 客户端）
    ├── Control.py            # 基于 Pygame 的主控制台（手柄/键盘下发 + 实时传感器监控）
    ├── modern_demo.py        # 基于 ModernGL + Pyglet 的三维硬件加速姿态动态演示
    └── requirements_3d.txt   # 上位机依赖包


🛠️ 下位机核心技术亮点 (STM32H7 + FreeRTOS)
下位机采用了多任务协同的 FreeRTOS 架构，任务优先级分配非常合理（控制任务 > 传感器与视觉任务 > 显示任务）：

Motor_Task (实时性最高 PriorityRealtime, 50ms周期):

PWM 输出驱动：利用 TIM8 的四个独立通道控制 4 个电机速度。
编码器测速：配置 TIM2 四通道双沿输入捕获，计算两脉冲时间差并结合累计脉冲计算实时 RPM。
一阶低通滤波：对原始测速进行 EMA 滤波（0.6 * prev + 0.4 * current），滤除脉冲抖动毛刺。
位置式双闭环 PID：实现了带 误差死区 (DeadBand) 和 积分分离 (I_Separation) 的 PID 闭环。在零速时自动清空 PID 状态，有效避免了静止抖动。
Motion_Task (PriorityNormal, 20ms周期) —— 机器人状态机与策略:

安全断连保护：通过 Control_IsOnline() 实时检测，一旦上位机心跳丢失（超时 500ms），立即刹车停转，确保安全。
遥控模式 (control_mode == 0)：解析上位机下发的摇杆数据，并支持固定的舵机旋转速率（按住 LT 持续正向，按住 RT 持续反向，松开保持）。
偏航角锁死（角度环）：启用后由 Angle_Task 叠加偏航角修正量，保证机器人走直线。
自动控制模式 (control_mode == 1)：
自动巡台 (Auto_Control_Logic)：利用前后左右 4 路灰度传感器和 2 路边缘光电开关，在边缘（白线）触发时执行退后、大角度旋转避让的“边缘防跌落”逻辑。
目标追击 (Detect)：如果视觉系统检测到“目标能量块” (tag_type 0/2)，下位机结合视觉 Yaw 角实时微调对齐，一旦对准立即高速前冲推下能量块，并利用光电触发和 10 秒超时进行动作收尾；如果检测到“友军能量块” (tag_type 1) 则执行主动避让。
Sensor_Task (PriorityAboveNormal, 100ms周期):

采用 ADC1 + DMA 循环读取多路模拟红外传感器和灰度值。
软 I2C 驱动：使用软件延时实现高兼容性的模拟 I2C 接口，读取 TOF050C 激光测距数据，高精度覆盖重写最关键的“正前方距离”值 (ir_distance[0])。
Vision_Task 与 Comm_Task (通信层):

串口通讯采用 UART DMA + 空闲中断(IDLE) 的高效接收机制。
视觉解析：MachineVision_GetFrame() 采用精密的字节流状态机解析机制，支持包头识别（0xAA 0x55）及 CRC-8/MAXIM (Dallas 1-Wire) 算法校验，过滤了环境电磁带来的串口乱码。


🖥️ 上位机交互与姿态可视化 (Python)
上位机部分提供了一个科技感与实用性并存的控制台：

Control.py (PyGame 主控制台):

手柄与键盘双模输入：支持标准的 Xbox/PS 游戏手柄，在手柄未连接时无缝退化为键盘平滑控制（WSAD/方向键控制底盘，Q/E控制扳机）。
50Hz 极速下发：每 20ms 组装一次标准遥控数据包 <vx, vy, lt, rt, buttons_mask>\n 下发至串口。
传感器监控面板：实时将串口反馈的机器人的姿态（IMU）、距离（TOF激光）、灰度值、光电开关、电机转速、视觉识别序列解析并在暗黑风格的 UI 界面以和谐色彩分类呈现。
实时 3D 姿态立方体：直接在主界面右侧绘制一个根据 IMU 的 Roll/Pitch/Yaw 实时旋转的 3D ROS 风格坐标轴。
modern_demo.py (3D 硬件加速演示):

采用了基于 ModernGL (OpenGL 3.3 Core Profile) 与 Pyglet 的真正硬件加速 3D 渲染，不仅包含网格地面、彩色坐标轴，还有一个高精度的三维姿态展示立方体，为调试机器人的运动学解算与平衡状态提供了极大便利。