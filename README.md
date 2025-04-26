# 自平衡两轮小车项目

## 项目简介

本项目是基于STM32F103C8T6单片机的自平衡两轮小车，通过MPU6050陀螺仪和加速度计传感器实现平衡控制，可以实现直立平衡、遥控行驶以及避障等功能。

[[`<video src="video.mp4" controls="controls" width="500" height="300"></video>`](https://github.com/user-attachments/assets/a6912871-1006-47c7-8a1c-eaa25079c848)
](https://github.com/user-attachments/assets/1233247d-ceec-42a4-b4fa-d42ac265dcec)

## 功能特点

- **自动平衡功能**：利用PID控制算法和MPU6050 DMP算法保持小车直立平衡
- **无线遥控**：可通过无线串口模块远程控制小车前进、后退、转向
- **超声波避障**：通过SR04超声波模块实现障碍物检测，当接近障碍物时自动减速
- **OLED显示**：实时显示姿态角度、编码器速度和障碍物距离等信息
- **失控保护**：当倾角过大时，自动切断电机，防止意外损坏

## 硬件组成

| 硬件模块         | 功能描述                           |
| :--------------- | :--------------------------------- |
| STM32F103C8T6    | 主控制芯片，负责所有算法和控制逻辑 |
| MPU6050          | 六轴加速度计和陀螺仪，用于姿态检测 |
| HC-SR04          | 超声波测距模块，用于障碍物检测     |
| 直流减速电机     | 驱动轮，带编码器反馈速度信息       |
| 0.96寸OLED显示屏 | 显示系统工作状态                   |
| 电机驱动模块     | 控制电机正反转                     |
| 锂电池           | 提供系统电源                       |

**硬件连接示意**：

- MPU6050通过IIC总线与STM32连接
- 电机通过驱动板与STM32的定时器PWM输出连接
- 编码器通过定时器编码器模式接口与STM32连接
- 超声波模块通过GPIO和定时器与STM32连接
- OLED显示屏通过IIC总线与STM32连接

## 软件设计

### 系统结构

本项目软件基于STM32 HAL库开发，主要包括以下模块：

- **姿态解算模块**：使用MPU6050 DMP算法获取Roll、Pitch、Yaw角度
- **平衡控制模块**：采用串级PID控制，包括直立环、速度环和转向环
- **电机驱动模块**：控制电机的PWM输出和方向
- **超声波测距模块**：通过测量超声波发射到接收的时间计算距离
- **OLED显示模块**：显示系统状态和传感器数据
- **编码器解码模块**：读取电机编码器信息，计算实际速度

  中断函数：

  1. 外部中断

  - void EXTI2_IRQHandler(void) // 对应 PA2，SR04 超声波 ECHO 上升/下降沿，EXTI2_IRQn 抢占优先级 0，子优先级 0
  - void EXTI9_5_IRQHandler(void) // 对应 PB5，MPU6050 DMP 中断，EXTI9_5_IRQn 抢占优先级 0，子优先级 0

  1. 串口中断

  - void USART3_IRQHandler(void) // 串口3，用于接收蓝牙模块数据，USART3_IRQn  0／0
  - 滴答定时器优先级15/0

### 控制算法

平衡控制采用三环PID控制算法：

1. **直立环(PD控制)**：稳定小车垂直角度
2. **速度环(PI控制)**：控制小车的前进后退速度
3. **转向环(PD控制)**：控制小车的左右转向

**PID控制流程图**：
请查看 [PID控制流程图](images/pid_diagram.txt) 了解具体控制流程

### 关键参数

```c
// 平衡参数
float Med_Angle=2.438; // 平衡时角度值偏移量（机械中值）
float Vertical_Kp=-252, Vertical_Kd=-3; // 直立环PD参数
float Velocity_Kp=-0.4, Velocity_Ki=-0.002; // 速度环PI参数
float Turn_Kp=10, Turn_Kd=0.1; // 转向环PD参数

// 平衡角度阈值，超过此角度认为失去平衡
#define BALANCE_ANGLE_THRESHOLD 40.0f

// 速度限制参数
#define SPEED_Y 15.0f  // 俯仰(前后)最大设定速度
#define SPEED_Z 150    // 偏航(左右)最大设定速度 
```

## 开发环境

- 开发工具：STM32CubeMX + Keil MDK ARM 5
- 编程语言：C语言
- 调试工具：ST-Link V2

## 文件结构

```
UpStanding_Car(520)/
├── Core/ - STM32 HAL核心代码
│   ├── Inc/ - 头文件
│   └── Src/ - 源文件
├── MyCode/ - 自定义驱动代码
│   ├── IIC.c/h - I2C通信协议实现
│   ├── mpu6050.c/h - MPU6050驱动
│   └── inv_mpu*.c/h - MPU6050 DMP姿态解算算法
├── Drivers/ - STM32 HAL驱动库
├── MDK-ARM/ - Keil工程文件
└── HARDWARE/ - 硬件设计文件
    └── 电路原理图和PCB图.eprj - PCB设计文件
```

## 使用方法

1. 安装电池并接通电源
2. 小车上电后，将会自动初始化并尝试保持平衡
3. 通过无线串口模块发送指令控制小车移动：
   - '1': 前进
   - '2': 后退
   - '3': 左转
   - '4': 右转
   - '0': 停止

## 注意事项

1. 首次使用时，请在平坦表面放置小车，确保其能够稳定平衡
2. 若小车无法平衡，可能需要调整 `Med_Angle`参数来适应机械结构
3. 不同的电池电量可能会影响平衡性能，请保持电池充足
4. 请避免在湿滑或不平坦的表面使用
5. 小车倒下时会自动切断电机电源，再次使用需要重新扶正重启

## 主要代码逻辑

### 平衡控制算法

```c
// 检查平衡车是否处于平衡状态
uint8_t Is_Balance(float angle)
{
    // 检查角度是否在平衡范围内
    if(fabs(angle - Med_Angle) > BALANCE_ANGLE_THRESHOLD)
    {
        return 0; // 失去平衡
    }
    return 1; // 平衡状态
}

//直立环PD控制器
int Vertical(float Med, float Angle, float gyro_Y)
{
    int temp;
    temp = Vertical_Kp * (Angle - Med) + Vertical_Kd * gyro_Y;
    return temp;
}

//速度环PI控制器
int Velocity(int Target, int encoder_L, int encoder_R)
{
    static int Err_LowOut_last, Encoder_S;
    static float a = 0.7;
    int Err, Err_LowOut, temp;
  
    //1、计算偏差值
    Err = (encoder_L + encoder_R) - Target;
    //2、低通滤波
    Err_LowOut = (1-a) * Err + a * Err_LowOut_last;
    Err_LowOut_last = Err_LowOut;
    //3、积分
    Encoder_S += Err_LowOut;
    //4、积分限幅
    Encoder_S = Encoder_S > 20000 ? 20000 : (Encoder_S < (-20000) ? (-20000) : Encoder_S);
  
    //5、速度环计算
    temp = Velocity_Kp * Err_LowOut + Velocity_Ki * Encoder_S;
    return temp;
}
```

### 主循环显示

```c
while (1)
{
    sprintf((char *)display_buf,"Encoder_L:%d   ",Encoder_Left);
    OLED_ShowString(0,0,display_buf,16);
    sprintf((char *)display_buf,"Encoder_R:%d   ",Encoder_Right);
    OLED_ShowString(0,2,display_buf,16);	
    sprintf((char *)display_buf,"roll:%.1f   ",roll); 
    OLED_ShowString(0,4,display_buf,16);
    GET_Distance();
    sprintf((char *)display_buf,"distance:%.1f  ",distance);
    OLED_ShowString(0,6,display_buf,12);
}
```

## 开发日志

- 完成硬件电路设计和PCB制作
- 完成基础驱动程序和MPU6050姿态解算
- 实现基础平衡控制算法
- 添加超声波避障功能
- 2025.4.22 系统整合和调试优化
- 添加蓝牙/WiFi控制功能，手机APP遥控界面

## 参考资料

1. STM32F103数据手册
2. MPU6050数据手册
3. PID控制理论与实践

## 许可证

本项目采用MIT许可证。详情请参阅LICENSE文件。
