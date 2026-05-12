# 基于 STM32 + FreeRTOS 的智能烟机控制系统

## 1. 项目简介

本项目是一个基于 **STM32F103RCT6 + FreeRTOS** 的智能烟机/风机控制系统，面向嵌入式实时控制场景设计。

系统以 STM32 为主控，结合温湿度传感器、烟雾传感器、电机驱动、编码器测速、LCD 显示、按键输入、RGB 状态灯、蜂鸣器报警、照明灯控制和自锁电源管理，实现烟机风机的手动控制、自动调速、烟雾报警、防回流控制和状态显示等功能。

项目重点体现以下能力：

- STM32 外设驱动开发
- FreeRTOS 多任务软件架构设计
- 电机 PWM 调速
- 编码器测速
- PID 闭环控制
- 多传感器采集与数据处理
- 状态机与事件驱动设计
- 自锁电源与软件关机控制
- PCB 硬件设计与模块调试

---

## 2. 硬件平台

### 2.1 主控芯片

| 模块 | 型号 / 说明 |
|---|---|
| MCU | STM32F103RCT6 |
| 内核 | ARM Cortex-M3 |
| 主频 | 72 MHz |
| 开发环境 | Keil MDK |
| 固件库 | STM32F10x Standard Peripheral Library |
| RTOS | FreeRTOS |

---

### 2.2 硬件功能模块

| 模块 | 说明 |
|---|---|
| 电源输入 | 12V DC 输入 |
| 自锁电源 | 支持按键开机、MCU 维持上电、软件关机 |
| 12V 转 5V | Buck 降压，为 MQ2、外设等供电 |
| 5V 转 3.3V | 为 STM32、SHT30、LCD 逻辑、按键等供电 |
| 温湿度传感器 | SHT30，I2C 通信 |
| 烟雾传感器 | MQ2，ADC 模拟采样 |
| 电机驱动 | TB6612FNG |
| 电机接口 | 支持 JGB37-520 编码电机 |
| 编码器接口 | A/B 相输入，使用定时器编码器模式 |
| 显示屏 | SPI LCD 显示屏 |
| 按键 | 模式、速度、灯光、升级/功能按键 |
| RGB 状态灯 | 显示系统运行状态 |
| 蜂鸣器 | 报警提示、按键提示 |
| 照明灯 | 12V 照明灯 MOS 控制 |
| 串口 | USART1 调试日志，USART2 预留 WiFi |
| SWD | 程序下载与调试 |
| Bootloader | 预留在线升级功能 |

---

## 3. 主要硬件引脚分配

### 3.1 LCD 显示屏

| LCD 信号 | STM32 引脚 | 说明 |
|---|---|---|
| LCD_CS | PA4 | 片选 |
| LCD_SCK | PA5 | SPI 时钟 |
| LCD_A0 / DC | PA6 | 数据/命令选择 |
| LCD_MOSI | PA7 | SPI 数据 |
| LCD_RST | PC4 | LCD 复位 |
| LCD_LED | 3.3V | 背光常亮 |

---

### 3.2 SHT30 温湿度传感器

| 信号 | STM32 引脚 | 说明 |
|---|---|---|
| SHT30_SCL | PC0 | I2C SCL |
| SHT30_SDA | PC1 | I2C SDA |
| VCC | 3.3V | 传感器供电 |
| GND | GND | 地 |

> SHT30 的 SCL/SDA 需要上拉电阻，通常使用 4.7kΩ 上拉到 3.3V。

---

### 3.3 MQ2 烟雾传感器

| 信号 | STM32 引脚 | 说明 |
|---|---|---|
| MQ_ADC | PA1 / ADC1_IN1 | 烟雾传感器模拟量输入 |
| MQ2_VCC | 5V | MQ2 加热供电 |
| GND | GND | 地 |

MQ2 输出经过分压后接入 STM32 ADC，避免 ADC 输入超过 3.3V。

---

### 3.4 TB6612 电机驱动

| 信号 | STM32 引脚 | 说明 |
|---|---|---|
| MOTOR_PWM | PA15 | 电机 PWM 输出 |
| MOTOR_IN1 | PC11 | 电机方向控制 1 |
| MOTOR_IN2 | PC10 | 电机方向控制 2 |
| MOTOR_STBY | PC12 | TB6612 使能 |
| MOTOR_M+ | TB6612 AO1/AO2 | 电机输出 |
| MOTOR_M- | TB6612 AO1/AO2 | 电机输出 |

> PA15 默认复用为 JTAG 引脚，如用于 PWM，需要关闭 JTAG 并保留 SWD，同时配置 TIM2 重映射。

---

### 3.5 编码器接口

| 信号 | STM32 引脚 | 说明 |
|---|---|---|
| ENCODER_A | PB6 / TIM4_CH1 | 编码器 A 相 |
| ENCODER_B | PB7 / TIM4_CH2 | 编码器 B 相 |
| ENC_VCC | 3.3V | 编码器供电 |
| ENC_GND | GND | 编码器地 |

编码器 A/B 相建议增加：

- 1kΩ 串联电阻
- 10kΩ 上拉到 3.3V
- 预留小电容滤波位

---

### 3.6 按键模块

| 按键 | STM32 引脚 | 功能 |
|---|---|---|
| KEY_MODE | PB2 | 模式切换 |
| KEY_SPEED | PB11 | 速度/档位切换 |
| KEY_LIGHT | PB1 | 照明灯开关 |
| KEY_UPGRADE | PB0 | 升级/功能按键 |
| KEY_POWER | PB9 | 电源按键 |

按键硬件采用 10kΩ 上拉，按下接地，软件中为低电平有效。

---

### 3.7 RGB 状态灯

| 信号 | STM32 引脚 | 说明 |
|---|---|---|
| LED_R_CTRL | PC7 | 红色控制 |
| LED_G_CTRL | PC9 | 绿色控制 |
| LED_B_CTRL | PC8 | 蓝色控制 |

RGB LED 为共阳极结构：


GPIO = 0 → LED 点亮
GPIO = 1 → LED 熄灭
### 3.8 照明灯控制
信号	STM32 引脚	说明
LIGHT_CTRL	PA8	MOS 管控制信号
LIGHT+	12V_SYS	照明灯正极
LIGHT-	MOS Drain	照明灯负极

照明灯采用 N-MOS 低边开关控制：

LIGHT_CTRL = 1 → 照明灯亮
LIGHT_CTRL = 0 → 照明灯灭

---

### 3.9 蜂鸣器
信号	STM32 引脚	说明
BUZZER_CTRL	PD2	蜂鸣器控制

蜂鸣器用于：

按键反馈
烟雾报警
故障提示
系统状态提示

---

### 4. 软件架构

本项目采用分层软件架构，整体分为：

Project
├── Application
├── Service
├── Driver
├── BSP
├── Common
├── Bootloader
├── FreeRTOS
├── Start
└── Library

各层职责如下：

层级	职责
Application	FreeRTOS 任务层，负责系统任务调度
Service	业务服务层，负责控制策略、状态机、数据处理
Driver	设备驱动层，负责具体硬件模块驱动
BSP	板级支持包，负责 STM32 外设初始化
Common	公共数据结构、消息定义、系统状态
Bootloader	固件升级功能预留
FreeRTOS	实时操作系统
Start	启动文件
Library	STM32 标准外设库

---

### 5. 工程目录说明
5.1 User
User
├── main.c
├── stm32f10x_conf.h
├── stm32f10x_it.c
└── stm32f10x_it.h
文件	说明
main.c	系统入口，初始化基础模块并启动应用
stm32f10x_conf.h	标准外设库配置文件
stm32f10x_it.c	中断服务函数
stm32f10x_it.h	中断函数声明 

---

### 5.2 Application 层
Application
├── app_main.c
├── sensor_task.c
├── power_task.c
├── task_key.c
├── control_task.c
├── task_motor.c
├── task_speed.c
├── task_auto.c
├── task_anti_backflow.c
├── task_ui.c
└── task_log.c
文件	说明
app_main.c	应用层初始化，创建任务、队列、事件对象
sensor_task.c	周期采集 SHT30、MQ2 数据
power_task.c	电源保持、关机请求处理
task_key.c	按键扫描，识别短按/长按
control_task.c	系统模式切换和事件处理
task_motor.c	电机控制任务，输出 PWM
task_speed.c	编码器测速任务
task_auto.c	自动调速任务
task_anti_backflow.c	防回流控制任务
task_ui.c	LCD 显示刷新任务
task_log.c	串口日志输出任务

---

### 5.3 Service 层
Service
├── sensor_service.c
├── event_service.c
├── control_service.c
├── speed_service.c
├── auto_control_service.c
├── anti_backflow_service.c
├── motor_control_service.c
├── pid_service.c
├── ui_service.c
└── log_service.c
文件	说明
sensor_service.c	统一封装传感器数据读取
event_service.c	系统事件管理与分发
control_service.c	模式切换、系统控制逻辑
speed_service.c	编码器数据处理、转速计算
auto_control_service.c	根据传感器数据计算目标转速
anti_backflow_service.c	防回流状态机
motor_control_service.c	电机控制服务
pid_service.c	PID 控制算法
ui_service.c	LCD 界面数据组织
log_service.c	调试日志格式化输出

---

### 5.4 Driver 层
Driver
├── drv_console.c
├── drv_sensor.c
├── drv_actuator.c
├── drv_key.c
├── drv_motor_pwm.c
├── drv_encoder.c
└── drv_lcd.c
文件	说明
drv_console.c	串口控制台驱动
drv_sensor.c	传感器驱动统一接口
drv_actuator.c	执行器驱动统一接口
drv_key.c	按键驱动
drv_motor_pwm.c	电机 PWM 驱动
drv_encoder.c	编码器驱动
drv_lcd.c	LCD 显示驱动

Driver 层只负责硬件模块的基础访问，不直接处理系统业务逻辑。

---

### 5.5 BSP 层
BSP
├── bsp_adc.c
├── bsp_delay.c
├── bsp_gpio.c
├── bsp_spi.c
├── bsp_tim.c
├── bsp_usart.c
├── bsp_power.c
└── bsp_sys.c
文件	说明
bsp_adc.c	ADC 外设初始化和采样
bsp_delay.c	微秒/毫秒延时
bsp_gpio.c	GPIO 基础配置
bsp_spi.c	SPI 或模拟 SPI 接口
bsp_tim.c	定时器基础配置
bsp_usart.c	USART 串口初始化与发送
bsp_power.c	自锁电源控制
bsp_sys.c	系统时钟、NVIC 等基础配置

---

### 5.6 Common 层
Common
├── message_def.h
├── system_state.h
└── board_pins.h
文件	说明
message_def.h	系统事件、消息类型定义
system_state.h	系统运行状态数据结构
board_pins.h	硬件引脚映射表

---

### 6. FreeRTOS 任务设计
### 6.1 任务划分
任务	周期	作用
sensor_task	500 ms	采集温湿度和烟雾数据
task_key	10 ms	按键扫描、消抖、短按长按识别
control_task	事件驱动	处理系统模式和控制事件
task_auto	100 ms	自动模式下计算目标转速
task_motor	50 ms	电机 PWM / PID 控制
task_speed	50 ms	编码器测速
task_anti_backflow	100 ms	防回流状态机
task_ui	200~500 ms	LCD 显示刷新
task_log	500~1000 ms	串口调试日志输出
power_task	100 ms	电源保持和关机处理

---

### 6.2 任务通信方式

系统任务之间主要通过以下方式通信：

FreeRTOS Queue
系统事件 SystemEvent_t
全局系统状态 SystemState_t
必要时使用 Mutex 保护共享状态

典型事件流：

按键扫描任务
    ↓
发送按键事件
    ↓
control_task 处理事件
    ↓
更新系统模式 / 风机档位 / 照明状态
    ↓
motor_task / ui_task / log_task 根据系统状态执行动作

---

### 7. 系统状态设计

系统状态集中定义在 Common/system_state.h 中，用于保存当前运行状态。

主要包含：

当前系统模式
风机档位
温湿度数据
MQ2 烟雾浓度数据
当前转速
目标转速
PWM 占空比
照明灯状态
蜂鸣器状态
RGB 状态灯状态
报警状态
电源状态

---

### 8. 系统工作模式
模式	说明
OFF	待机模式，风机关闭
MANUAL	手动模式，通过按键切换风机档位
AUTO	自动模式，根据 MQ2 / SHT30 数据自动调速
ALARM	报警模式，烟雾浓度过高时强制高速运行
ANTI_BACKFLOW	防回流模式，根据烟雾变化趋势执行强排
UPGRADE	升级模式，预留 Bootloader 功能
ERROR	故障模式，如电机异常、传感器异常

---

### 9. 控制逻辑
### 9.1 手动模式

手动模式下，通过 KEY_SPEED 切换风机档位：

档位	PWM / 目标转速
LOW	低速
MID	中速
HIGH	高速

---

### 9.2 自动模式

自动模式下，系统根据 MQ2 烟雾浓度、温度、湿度计算目标转速。

基础控制策略：

烟雾浓度低 → 低速运行
烟雾浓度中等 → 中速运行
烟雾浓度高 → 高速运行
烟雾浓度过高 → 进入报警模式

后续可扩展为多传感器融合算法：

F = 0.6 * gas_factor + 0.2 * temperature_factor + 0.2 * humidity_factor
target_rpm = rpm_min + F * (rpm_max - rpm_min)

---

### 9.3 电机闭环控制

编码器用于采集电机转速，PID 控制器根据目标转速和当前转速计算 PWM 占空比。

控制流程：

目标转速 target_rpm
        ↓
编码器测速 current_rpm
        ↓
PID 控制器
        ↓
PWM duty
        ↓
TB6612
        ↓
电机转速变化

---

### 9.4 防回流控制

防回流功能基于烟雾浓度变化趋势和阈值状态机实现。

基本思路：

烟雾浓度快速上升
        ↓
进入防回流状态
        ↓
风机强制高速运行
        ↓
烟雾浓度稳定下降并持续一段时间
        ↓
退出防回流状态

防止烟雾浓度在阈值附近波动导致风机频繁启停。

---

### 10. 日志输出

系统通过 USART1 输出调试信息。

典型日志格式：

[SENSOR] Gas=65% Temp=28C Humi=74%
[MOTOR] rpm=360 target=220 duty=650
[MODE] AUTO
[KEY] MODE_SHORT
[ALARM] GAS_HIGH

---

### 11. 当前项目进度

已完成验证：

12V 自锁电源电路
USART1 串口打印
SHT30 / 温湿度模块替换方案
MQ2 ADC 采样
RGB 状态灯控制
按键输入检测
照明灯 MOS 控制逻辑
TB6612 电机驱动基础测试
编码器测速问题定位
PCB V2 硬件优化设计

待完善：

FreeRTOS 任务完整整合
编码器硬件滤波与上拉优化
PID 参数整定
LCD 页面显示
防回流状态机
Bootloader 在线升级
长时间运行稳定性测试

---

### 12. 编译与下载
### 12.1 开发环境
项目	说明
IDE	Keil MDK
Compiler	ARMCC V5
MCU	STM32F103RCT6
Debugger	ST-Link
RTOS	FreeRTOS

---

### 12.2 下载方式

使用 SWD 下载：

SWD 信号	STM32 引脚
SWDIO	PA13
SWCLK	PA14
NRST	NRST
VCC	3.3V
GND	GND

---

### 13. 设计注意事项
### 13.1 电机干扰

电机线和编码器线应尽量分开走线，避免电机 PWM 和有刷电机噪声干扰编码器输入。

建议：

编码器 A/B 增加 10kΩ 上拉
编码器 A/B 增加 1kΩ 串联电阻
预留小电容滤波
电机两端预留 100nF 抑制电容
TB6612 VM 附近放置大电解电容

---

### 13.2 MQ2 发热

MQ2 内部有加热丝，工作时发热属于正常现象。

注意：

MQ2 加热电源使用 5V
MQ_ADC 必须经过分压后再接 STM32
ADC 输入电压不得超过 3.3V
MQ2 尽量远离温湿度传感器，避免影响温度测量
13.3 SHT30 布局

SHT30 应尽量远离发热器件：

MQ2
MP1584 Buck
AMS1117
TB6612
MOS 管
大电流走线

建议放在板边，保证空气流通。

---

### 13.4 供电稳定性

电机启动和 PWM 调速会造成电源波动，建议：

12V_SYS 加大电解电容
5V Buck 输入输出电容靠近芯片
3.3V 电源增加 10uF + 100nF
MCU 每个 VDD 附近放置 100nF 去耦电容
14. 后续计划
完成 FreeRTOS 多任务整合
完成事件驱动控制框架
完成编码器测速稳定性优化
完成 PID 闭环调速
完成 LCD 状态界面
完成防回流动态阈值状态机
完成 Bootloader + CRC32 固件升级
完成项目测试数据记录
整理项目文档和简历描述

---

### 15. 项目特点

本项目不是简单的传感器采集或普通 IoT 控制，而是围绕风机控制系统构建了完整的嵌入式实时控制框架，涵盖硬件设计、驱动开发、任务调度、传感器数据处理、电机闭环控制和状态机设计，适合作为嵌入式软件/嵌入式控制方向的综合项目。



