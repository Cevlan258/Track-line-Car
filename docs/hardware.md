# 硬件与接口

[返回主页](../README.md) · [构建与部署](getting-started.md)

此表依据仓库中的外设初始化与应用配置整理。它描述当前软件要求；历史采购清单不等同于当前整车实装清单。

| 模块 | 当前软件配置 | 作用 |
| --- | --- | --- |
| STM32F103RCT6 | Cortex-M3；工程目标 `CAR_FIRST` | 实时控制、外设驱动与保护 |
| MaixCAM-Pro | MaixPy；320 × 240 图像；UART1 | 视觉与路线决策 |
| 雷达 | LD2450 格式，30 字节帧，最多 3 个目标 | 目标坐标和障碍侧别信息 |
| LoRa | EBYTE-EWT22A-900BWL22S 测试套件 / EWM22A-900BWL22S 模组 | 检查点数据上报 |
| 编码器、电机与舵机 | 双轮反馈、内轮减速、舵机转向 | 底盘运动执行 |
| SSD1309 OLED | SPI 与独立控制引脚 | 状态、时间、电池及雷达显示 |

## 串口连接

下列串口均配置为 **115200、8 数据位、无校验、1 停止位**。设备间需共地，TX 与 RX 交叉连接。

| 外部端 | STM32 端 | 说明 |
| --- | --- | --- |
| MaixCAM A19 / UART1 TX | PB11 / USART3 RX | MaixCAM → STM32 命令 |
| MaixCAM A18 / UART1 RX | PB10 / USART3 TX | STM32 → MaixCAM 遥测 |
| 雷达 TX | PC11 / UART4 RX | 当前雷达接收入口 |
| 雷达 RX（如需） | PC10 / UART4 TX | 固件当前不依赖配置命令发送 |
| LoRa RX | PA2 / USART2 TX | 配置与检查点报文 |
| LoRa TX | PA3 / USART2 RX | 串口预留接收 |
| LoRa AUX | PC2 | 模块就绪/忙状态 |

MaixCAM 串口使用 3.3 V 逻辑电平和独立稳定供电；不要从信号针脚为模块供电。其他电源连接按实际模块与底板规格确认，仓库没有完整原理图。

完整引脚配置以 [CubeMX 工程](../firmware/stm32/CAR_FIRST.ioc)、[UART 初始化](../firmware/stm32/Core/Src/usart.c)和 [GPIO 定义](../firmware/stm32/Core/Inc/main.h)为准。

## 雷达与 LoRa 的版本说明

早期需求中出现的 **HLK-LD2410S** 属于历史记录。当前 `radar.c` 解析的是 **LD2450 协议**：`AA FF 03 00` 帧头、3 组目标和 `55 CC` 帧尾；请使用输出兼容协议的设备，不能直接用旧需求中的型号替代。

当前 LoRa 配置以 [app_config.h](../firmware/stm32/App/Inc/app_config.h) 为准：team22 / `CIRCUIT_VOYAGE`、900MHz 频段、Fixed Mode、信道 10、本机和目标地址 `0x0001`。发送报文含目标地址和信道三字节前缀。个别历史提交标题中的 transparent mode 不是当前配置依据。

[历史 BOM](archive/巡线小车_BOM.xlsx)保留用于追溯选型，不保证与当前固件和实际车辆完全一致。
