# STM32 智能小车项目 (STM32 Smart Car Project)

## 项目概述 (Project Overview)

本项目旨在构建一个基于 **STM32C103F8** 微控制器的智能小车，该小车能够使用 **MPU6050** 六轴传感器模块实现自我修正前进方向的功能。通过实时读取陀螺仪和加速度计数据，小车可以保持直线行驶，即使在遇到外部干扰时也能自动调整方向。

This project aims to build an intelligent small car based on the **STM32C103F8** microcontroller, which can use the **MPU6050** six-axis sensor module to self-correct its forward direction. By reading gyroscope and accelerometer data in real-time, the car can maintain straight-line driving and automatically adjust its direction even when encountering external disturbances.

---

## 硬件需求 (Hardware Requirements)

### 核心组件 (Core Components)

- **STM32F103C8T6** 微控制器开发板 (Microcontroller Development Board)
  - ARM Cortex-M3 内核 (ARM Cortex-M3 Core)
  - 64KB Flash, 20KB SRAM
  - 72MHz 最大工作频率 (Max Clock Frequency)

- **MPU6050** 六轴传感器模块 (6-Axis Sensor Module)
  - 3轴陀螺仪 (3-axis Gyroscope)
  - 3轴加速度计 (3-axis Accelerometer)
  - I2C 通信接口 (I2C Communication Interface)

- **电机驱动模块** (Motor Driver Module)
  - L298N 双H桥电机驱动器 (Dual H-Bridge Motor Driver)
  - 支持 5V-35V 输入电压 (Supports 5V-35V Input Voltage)

- **直流减速电机** x 4 (DC Gear Motors x 4)
  - 工作电压: 6V-12V (Operating Voltage: 6V-12V)
  - 带编码器的减速电机（可选）(Gear Motors with Encoders - Optional)

- **电源模块** (Power Supply Module)
  - 7.4V 锂电池或 4节18650电池盒 (7.4V Li-Po Battery or 4x18650 Battery Holder)
  - 5V 稳压模块 (5V Voltage Regulator Module)

- **其他配件** (Other Accessories)
  - 小车底盘套件 (Car Chassis Kit)
  - 杜邦线 (Jumper Wires)
  - 面包板（可选）(Breadboard - Optional)

---

## 系统架构 (System Architecture)

### 连接图 (Connection Diagram)

```
STM32F103C8T6          MPU6050
    PB6 (I2C1_SCL) --> SCL
    PB7 (I2C1_SDA) --> SDA
    3.3V           --> VCC
    GND            --> GND

STM32F103C8T6          L298N Motor Driver
    PA0 (TIM2_CH1) --> ENA (Motor A Enable)
    PA1 (TIM2_CH2) --> IN1 (Motor A Direction 1)
    PA2 (TIM2_CH3) --> IN2 (Motor A Direction 2)
    PA3 (TIM2_CH4) --> ENB (Motor B Enable)
    PA4            --> IN3 (Motor B Direction 1)
    PA5            --> IN4 (Motor B Direction 2)
    GND            --> GND
```

### 工作原理 (Working Principle)

1. **初始化系统** - 配置 I2C 接口与 MPU6050 通信
2. **读取传感器数据** - 实时获取陀螺仪 Z 轴角速度
3. **计算偏航角** - 积分角速度得到当前偏航角度
4. **PID 控制算法** - 根据偏航角误差计算修正量
5. **调整电机速度** - 差速控制左右电机，修正行驶方向

---

## 软件实现 (Software Implementation)

### 开发环境设置 (Development Environment Setup)

- **IDE**: Keil MDK-ARM 或 STM32CubeIDE (Keil MDK-ARM or STM32CubeIDE)
- **HAL库**: STM32Cube HAL Library
- **编程语言**: C语言 (C Language)

### MPU6050 初始化代码 (MPU6050 Initialization Code)

```c
#include "stm32f1xx_hal.h"
#include "mpu6050.h"

// I2C handle
I2C_HandleTypeDef hi2c1;

// MPU6050 地址 (MPU6050 Address)
#define MPU6050_ADDR 0xD0

// 初始化 MPU6050 (Initialize MPU6050)
void MPU6050_Init(void) {
    uint8_t check;
    uint8_t data;
    
    // 检查 WHO_AM_I 寄存器 (Check WHO_AM_I register)
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
    
    if (check == 0x68) {
        // 唤醒 MPU6050 (Wake up MPU6050)
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 1000);
        
        // 设置陀螺仪量程 ±250°/s (Set gyroscope range ±250°/s)
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &data, 1, 1000);
        
        // 设置加速度计量程 ±2g (Set accelerometer range ±2g)
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &data, 1, 1000);
    }
}

// 读取陀螺仪数据 (Read Gyroscope Data)
void MPU6050_Read_Gyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    uint8_t data[6];
    
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, data, 6, 1000);
    
    *gyro_x = (int16_t)(data[0] << 8 | data[1]);
    *gyro_y = (int16_t)(data[2] << 8 | data[3]);
    *gyro_z = (int16_t)(data[4] << 8 | data[5]);
}
```

### 方向修正算法 (Direction Correction Algorithm)

```c
// PID 控制器参数 (PID Controller Parameters)
typedef struct {
    float Kp;        // 比例系数 (Proportional gain)
    float Ki;        // 积分系数 (Integral gain)
    float Kd;        // 微分系数 (Derivative gain)
    float setpoint;  // 目标值 (Setpoint)
    float integral;  // 积分累积 (Integral accumulation)
    float prev_error;// 上次误差 (Previous error)
} PID_Controller;

PID_Controller direction_pid = {
    .Kp = 2.0f,
    .Ki = 0.1f,
    .Kd = 0.5f,
    .setpoint = 0.0f,
    .integral = 0.0f,
    .prev_error = 0.0f
};

// PID 计算函数 (PID Calculation Function)
float PID_Calculate(PID_Controller *pid, float current_value, float dt) {
    float error = pid->setpoint - current_value;
    
    // 比例项 (Proportional term)
    float p_term = pid->Kp * error;
    
    // 积分项 (Integral term)
    pid->integral += error * dt;
    float i_term = pid->Ki * pid->integral;
    
    // 微分项 (Derivative term)
    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->Kd * derivative;
    
    pid->prev_error = error;
    
    return p_term + i_term + d_term;
}

// 主控制循环 (Main Control Loop)
void Car_Direction_Control(void) {
    int16_t gyro_x, gyro_y, gyro_z;
    static float yaw_angle = 0.0f;
    float dt = 0.01f; // 10ms 采样周期 (10ms sampling period)
    
    // 读取陀螺仪 Z 轴数据 (Read gyroscope Z-axis data)
    MPU6050_Read_Gyro(&gyro_x, &gyro_y, &gyro_z);
    
    // 转换为角速度 (°/s) (Convert to angular velocity in °/s)
    float gyro_z_dps = gyro_z / 131.0f;
    
    // 积分得到偏航角 (Integrate to get yaw angle)
    yaw_angle += gyro_z_dps * dt;
    
    // PID 控制计算修正量 (PID control calculates correction)
    float correction = PID_Calculate(&direction_pid, yaw_angle, dt);
    
    // 基础速度 (Base speed)
    int base_speed = 50; // PWM 占空比 (PWM duty cycle): 0-100
    
    // 应用修正到左右电机 (Apply correction to left and right motors)
    int left_speed = base_speed + (int)correction;
    int right_speed = base_speed - (int)correction;
    
    // 限制速度范围 (Limit speed range)
    left_speed = (left_speed > 100) ? 100 : (left_speed < 0) ? 0 : left_speed;
    right_speed = (right_speed > 100) ? 100 : (right_speed < 0) ? 0 : right_speed;
    
    // 设置电机速度 (Set motor speed)
    Motor_SetSpeed(left_speed, right_speed);
}
```

### 电机控制函数 (Motor Control Functions)

```c
// 设置电机速度 (Set Motor Speed)
void Motor_SetSpeed(int left_speed, int right_speed) {
    // 左侧电机 (Left Motor)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_speed);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // IN1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // IN2
    
    // 右侧电机 (Right Motor)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, right_speed);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // IN3
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // IN4
}

// 电机前进 (Motor Forward)
void Motor_Forward(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

// 电机停止 (Motor Stop)
void Motor_Stop(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}
```

---

## 实现步骤 (Implementation Steps)

### 任务清单 (Task Checklist)

- [x] 准备所有硬件组件 (Prepare all hardware components)
- [x] 组装小车底盘和安装电机 (Assemble car chassis and install motors)
- [ ] 连接 STM32 与 MPU6050 传感器 (Connect STM32 with MPU6050 sensor)
- [ ] 连接 STM32 与 L298N 电机驱动器 (Connect STM32 with L298N motor driver)
- [ ] 配置 STM32CubeMX 工程 (Configure STM32CubeMX project)
  - [ ] 启用 I2C1 外设 (Enable I2C1 peripheral)
  - [ ] 配置 TIM2 PWM 输出 (Configure TIM2 PWM output)
  - [ ] 配置 GPIO 引脚 (Configure GPIO pins)
- [ ] 编写 MPU6050 驱动程序 (Write MPU6050 driver)
- [ ] 实现 PID 控制算法 (Implement PID control algorithm)
- [ ] 编写电机控制函数 (Write motor control functions)
- [ ] 调试和校准传感器 (Debug and calibrate sensors)
- [ ] 调整 PID 参数以获得最佳性能 (Tune PID parameters for optimal performance)
- [ ] 测试直线行驶功能 (Test straight-line driving function)
- [ ] 优化和完善代码 (Optimize and refine code)

---

## 调试技巧 (Debugging Tips)

### MPU6050 传感器校准 (MPU6050 Sensor Calibration)

1. **零点校准** - 将小车静止放置，读取多次陀螺仪数据并计算平均值作为零点偏移
2. **数据滤波** - 使用互补滤波器或卡尔曼滤波器减少噪声
3. **温度补偿** - MPU6050 对温度敏感，需要进行温度漂移补偿

### PID 参数调整 (PID Parameter Tuning)

| 参数 | 作用 | 调整方法 |
|------|------|----------|
| **Kp** | 快速响应 | 逐步增大直到出现振荡，然后减小 |
| **Ki** | 消除稳态误差 | 从0开始缓慢增加，避免积分饱和 |
| **Kd** | 抑制振荡 | 增加以减少超调和振荡 |

---

## 扩展功能 (Extended Features)

### 可能的改进方向 (Possible Improvements)

1. :robot: **障碍物检测** - 添加超声波传感器实现避障功能
2. :iphone: **蓝牙控制** - 集成 HC-05 模块实现手机遥控
3. :mag: **循迹功能** - 添加红外传感器实现黑线循迹
4. :electric_plug: **电池管理** - 添加电压监测和低电量报警
5. :camera: **视觉导航** - 集成摄像头模块实现视觉识别

---

## 参考资源 (References)

- [STM32F103 数据手册](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf)
- [MPU6050 寄存器映射](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
- [STM32CubeIDE 用户指南](https://www.st.com/en/development-tools/stm32cubeide.html)
- [PID 控制理论](https://en.wikipedia.org/wiki/PID_controller)

---

## 许可证 (License)

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

---

## 贡献 (Contributing)

欢迎提交问题报告和拉取请求！

Welcome to submit issue reports and pull requests!

---

### 项目图片示例 (Project Image Example)

![STM32 Smart Car](https://img.shields.io/badge/STM32-Smart_Car-blue?style=for-the-badge&logo=stmicroelectronics)
![MPU6050](https://img.shields.io/badge/Sensor-MPU6050-green?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-In_Development-yellow?style=for-the-badge)

---

**作者**: Ji-Xuan  
**日期**: 2026年1月  
**版本**: 1.0
