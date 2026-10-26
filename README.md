# STM32 Smart Cleaning Car

基于双 STM32F103C8T6 和 FreeRTOS 的智能清扫车控制系统。项目由主控板与采集从板组成，通过 CAN 总线交换轮速、电量和状态信息，支持蓝牙手动控制、超声波避障、轨迹录制与回放、弓字形覆盖清扫及里程计返航。

## 硬件与演示资料

- [电路原理图 PDF](docs/hardware/cleaning-car-schematic.pdf)
- [电路原理图源文件](docs/hardware/cleaning-car-schematic.epro2)
- 实物演示视频和手机蓝牙 APP：见本仓库的 [Releases](https://github.com/Nyxarian/stm32-smart-cleaning-car/releases)

视频和 APK 作为 Release 附件提供，源码仓库保持轻量，便于克隆和阅读。

> 本仓库来自毕业设计样机，重点展示嵌入式系统拆分、外设驱动、实时任务调度、通信协议与运动控制实践。它是教学/原型项目，不是量产清洁机器人固件。

## 系统架构

| 节点 | 主要职责 | 主要外设 |
| --- | --- | --- |
| `Car_1` 主控 | FreeRTOS 调度、运动模式、航向控制、避障、轨迹录放、路径规划、人机交互 | CAN、UART、I2C、TIM/PWM、GPIO、内部 Flash |
| `Car_2` 从控 | 编码器测速、电池 ADC 采样、状态上报、LED 指示 | CAN、TIM Encoder、ADC、GPIO |

主控通过 CAN 标准帧接收双轮编码器速度和电量信息。速度帧超过 200 ms 未到达时，软件看门狗停止底盘，降低通信中断后继续运行的风险。

## 已实现功能

- FreeRTOS 多任务调度：显示、超声波、MPU6050、蓝牙、电机模式、控制更新和 CAN 通信分任务运行。
- 蓝牙串口协议：以 `[` 和 `]` 组帧，通过中断接收与 FreeRTOS 队列处理分包、粘包和缓冲区边界。
- 四种运行模式：手动控制、超声波避障、轨迹录制/回放、弓字形覆盖清扫。
- 避障有限状态机：停车后由舵机带动超声波查看左右空间，再选择转向或掉头。
- 航向保持：MPU6050 Z 轴角速度积分得到 yaw，直行时采用航向误差比例补偿进行左右轮差速修正。
- 航迹推算：融合编码器轮速与 yaw，将增量里程投影到二维坐标，用于弓字形路径和原点返航。
- 轨迹录放：记录按键、时间戳、速度和实际转角并保存到片内 Flash；回放转弯使用陀螺仪反馈的角度控制，减少纯时间回放的转角累计误差。
- 配置持久化：速度和清扫区域长宽写入 STM32 内部 Flash，带魔数和范围校验。
- 电量采集：从控通过 ADC 采样并进行一阶 IIR 滤波，再通过 CAN 上报主控。
- OLED 状态显示、清扫电机/继电器控制和 UART 错误恢复。

## 控制实现说明

源码保留了位置式速度 PID 的计算函数，但当前样机的实际输出路径没有启用编码器速度闭环：控制任务将目标值作为 PWM 基准，并叠加基于 yaw 的航向比例补偿。因此本项目当前应准确描述为“开环 PWM 调速 + 航向闭环修正”，不应描述为已投入运行的双轮速度 PID 闭环。

MPU6050 当前只使用 Z 轴角速度积分和静止死区，没有实现卡尔曼滤波或加速度计融合。里程计属于航迹推算，会受到轮胎打滑、轮径误差和陀螺仪漂移影响。

## 软件结构

```text
Car_1/
  Core/                 CubeMX 生成的主控启动和外设初始化代码
  Drivers/              STM32 HAL 与 CMSIS
  MDK-ARM/
    application/        FreeRTOS 任务、共享类型和资源
    common/             模式、控制、协议、里程计与业务状态机
    interface/          电机、CAN、MPU6050、超声波、Flash 等驱动封装
    FreeRTOS/            FreeRTOS 内核及 Cortex-M3 移植
    Car_1.uvprojx       Keil MDK 工程
Car_2/
  Core/                 从控启动和外设初始化代码
  Drivers/              STM32 HAL 与 CMSIS
  MDK-ARM/
    application/        从控 FreeRTOS 任务
    common/             CAN 应用层
    interface/          编码器、ADC 与 CAN 驱动封装
    FreeRTOS/            FreeRTOS 内核及 Cortex-M3 移植
    Car_2.uvprojx       Keil MDK 工程
```

## 主要协议

| CAN ID | 方向 | 内容 |
| --- | --- | --- |
| `0x201` | 从控 -> 主控 | 左右轮编码器速度，4 字节 |
| `0x202` | 从控 -> 主控 | 电量百分比、原始 ADC 和滤波 ADC，5 字节 |
| `0x101` | 主控 -> 从控 | 三路 LED 状态，3 字节 |

蓝牙命令示例：

```text
[key,qianjin,down]
[mode,bizhang]
[speed,50]
[bow,150,100]
[cmd,guihua_start]
```

## 开发与构建

1. 使用 STM32CubeMX 打开 `Car_1/Car_1.ioc` 或 `Car_2/Car_2.ioc` 查看外设配置。
2. 使用 Keil MDK-ARM 打开对应的 `MDK-ARM/*.uvprojx`。
3. 分别编译主控和从控工程，通过 ST-Link/SWD 下载到各自的 STM32F103C8T6。
4. 确保两块控制板共地，并通过 CAN 收发器连接 CAN_H/CAN_L；当前配置波特率为 500 kbit/s。

硬件引脚、电机方向、编码器极性、电池 ADC 满空量程、轮径和脉冲数均与样机相关，移植到其他底盘前需要重新标定。

## 已知限制与后续改进

- 启用并重新整定编码器速度 PID，形成真实的双轮速度闭环。
- 将阻塞式角度转向和清扫继电器时序改为非阻塞状态机或软件定时器。
- 为 MPU6050 增加启动零偏标定、在线自检和姿态融合。
- 为 CAN 增加更严格的过滤器、发送失败统计和总线恢复策略。
- 为 Flash 数据增加版本号、长度与 CRC，避免掉电写入导致数据损坏。
- 完成低电量返航状态，而不是将当前的里程计返航表述为自动充电。
- 补充可重复的台架数据，例如控制周期抖动、CAN 丢包恢复、路径误差和续航测试。

## 工具与第三方代码

项目使用 STM32CubeMX/STM32 HAL、CMSIS、FreeRTOS 和 Keil MDK。`Core` 与部分外设初始化代码由 CubeMX 生成，HAL、CMSIS 和 FreeRTOS 保留其原有版权与许可。项目应用逻辑位于 `application`、`common` 和 `interface` 目录。
