# 🎯 2025 TI杯全国大学生电子设计竞赛 E题：简易自行瞄准装置 🚀

🌟 **获奖作品**：省级一等奖 | 🕒 2025年TI杯全国大学生电子设计竞赛  
🚗 **核心系统**：智能小车 + ZDT42步进电机云台 | 🔍 **视觉模块**：Maxicam Pro AI视觉模组

## 📌 项目概述

本项目实现了一套高度集成的智能瞄准系统，结合了：
- 🧭 **精准运动控制**（PID双电机控制）
- 🤖 **AI视觉识别**（Maxicam Pro目标检测）
- 🎯 **实时动态瞄准**（三维空间坐标转换）
- 📊 **多模态交互系统**（OLED+矩阵键盘）

系统在复杂环境下表现出优异的稳定性和精确性，成功解决了"移动平台上的实时瞄准"这一核心挑战。

## 🧩 系统架构

```
+-----------------+     +----------------+     +---------------+
|  视觉处理模块     |<--->| 主控制器(MSPM0) |<--->| 云台控制系统    |
| (Maxicam Pro)   |     |                |     | (ZDT42步进)   |
+-----------------+     +-------+--------+     +---------------+
            ↑                   |                    ↑
            |                   |                    |
            |                   ↓                    |
      +------------+     +---------------+     +---------------+
      | 路径识别    |     | 电机控制系统    |     |   激光发射装置  |
      |  K230      |     | (PID+编码器)   |     |               |
      +------------+     +---------------+     +---------------+
```

## 💡 核心创新点

1. **动态PID调整系统** 🔄
   ```c
   void adj_perf4_pid() { // 实时调整巡线/转向参数
     KP_p4 = InputFloat(8, 2, 0); 
     KD_p4 = InputFloat(8, 2, 0);
   }
   ```

2. **三维空间坐标映射** 🗺️
   ```c
   // 从小车坐标系转换到世界坐标系
   void calculate_angles_for_perf4(float x, float y, float height, int car_direction, 
                         float target_x, float target_y, float target_height,
                         float *yaw, float *pitch) {
     // 空间几何解算核心算法
   }
   ```

3. **复合任务处理器** ⚙️
   ```c
   void Perform4() { // 巡线+打靶复合任务
     while(car_stage != 's'){
       go_on_line(0.25);      // 巡线控制
       update_motor_speed();  // 电机更新
       if(perf_4_flag) {      // 打靶条件检测
         calculate_target();  // 目标解算
       }
     }
   }
   ```

## 🧠 智能核心模块

### 1. 运动控制系统 (`motor.c`)
```c
// 自适应巡线算法
int go_on_line(float speed_m_per_s) {
  float current_error = line_error; // 视觉误差输入
  float p_term = KP * current_error; 
  float d_term = KD * (current_error - last_error);
  // ...智能速度分配算法...
}

// 精确转向控制
void turn_pid(float base_speed, float target_angle, float threshold, float Kp) {
  while (fabsf(calculate_yaw_error(target_angle, yaw)) >= threshold) {
    // 实时调整转向参数
  }
}
```

### 2. 云台瞄准系统 (`PTZ_controller.c`)
```c
// 世界坐标系转换
void calculate_angles(float x, float y, float* theta_x, float* theta_y) {
  *theta_x = atan2f(x, dist_to_canvas) * (180.0f / M_PI);
  float effective_dist = sqrtf(x*x + dist_to_canvas*dist_to_canvas);
  *theta_y = atan2f(y - pen_height, effective_dist) * (180.0f / M_PI);
}

// 交互式云台控制
void contrl_ptz() {
  int x = (int)InputFloat(5, 0, 0); // X轴角度
  int y = (int)InputFloat(5, 0, 0); // Y轴角度
  set_xy_offsets(x,y); // 设置云台位置
}
```

### 3. 视觉通信系统 (`uart_handler.c`)
```c
// 鲁棒性通信协议
void extract_template(uint8_t *data, uint8_t size) {
  if(data[0]=='<' && data[7]=='>') { // 帧校验
    // 数据提取与转换
    convert_to_float(nums); // 视觉误差值转换
  }
}

// DMA双缓冲传输
void uart_init(void) {
  DL_UART_Main_enableDMAReceiveEvent(UART_K230_INST, DL_UART_DMA_INTERRUPT_RX);
  // ...双缓冲配置...
}
```

### 4. 多任务调度系统 (`main.c`)
```c
while(1) {
  char key = KEY_SCAN(); // 矩阵键扫描
  switch (key) {
    case 1: Perform1(); break; // 基础巡线
    case 4: Perform4(); break; // 巡线+打靶
    case 7: contrl_ptz(); break; // 手动云台
    case 11: adj_line_pid(); break; // PID调整
  }
}
```

## 🛠️ 硬件配置

| 模块         | 型号/规格                     | 功能特点                     |
|--------------|-------------------------------|-----------------------------|
| 主控制器     | TI MSPM0G3507                 | 80MHz Cortex-M0+核心       |
| 驱动电机     | MG520X直流减速电机         | 6V/300RPM，集成光电编码器   |
| 云台电机     | ZDT42步进电机                 | 1.8°步进角，高精度闭环控制  |
| 视觉模块     | Maxicam Pro AI视觉模组        | 支持YOLOv5目标检测          |
| 姿态传感器  | LSM6DSV16X 6轴IMU             | ±2000dps，低噪声|
| 显示模块     | SSD1306 0.96寸OLED            | 128×64分辨率，I^2^C接口       |
| 输入设备     | 4×4矩阵键盘                   | 16键多功能控制              |

## 💻 开发环境

1. **编译环境**：  
    TI Code Composer Studio  
   - 集成编译+调试+烧录一体化环境
   - 完整MSPM0芯片支持包
   - 实时变量监控与分析工具

2. **烧录工具**：  
    TI XDS110 Debug Probe  
   - 通过USB直接烧录和调试
   - 支持实时变量监控
   - 4线JTAG接口

3. **依赖库**：
   ```bash
   DriverLib - MSPM0外设驱动库
   grlib - 图形显示库
   math.h - 数学运算库
   ```

## 📋 使用指南

1. **环境配置**：
   ```bash
   1. 安装CCS
   2. 导入项目工程
   3. 连接XDS110调试器
   ```

2. **操作流程**：
   ```mermaid
   graph TD
     A[上电初始化] --> B{按键选择模式}
     B -->|按键1| C[基础巡线]
     B -->|按键4| D[巡线+打靶]
     B -->|按键7| E[手动云台控制]
     C --> F[执行任务]
     D --> F
     E --> G[输入坐标参数]
   ```

3. **参数调整**：
   ```c
   // 关键性能参数
   #define KP 0.0021f    // 巡线比例系数
   #define KD 0.00021f   // 巡线微分系数
   #define MOVE_DELAY 20.0f // 云台运动延时
   ```


## 📜 版权声明

© 2025 生为刺猬队（东南大学） - 保留所有权利  
代码授权：MIT License  
TI驱动库版权归Texas Instruments Incorporated所有