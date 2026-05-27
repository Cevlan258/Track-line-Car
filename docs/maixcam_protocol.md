# MaixCAM-Pro 主控协议

本协议用于替代旧 OpenMV 感知帧。MaixCAM-Pro 负责视觉、路线推进、路口选择、终点判断和避障决策；STM32 负责电机闭环、舵机、电池检测、LoRa、RTC、雷达原始解析和安全停车。

## 硬件连接

- MaixCAM-Pro `A19/UART1_TX` -> STM32 `PB11/USART3_RX`
- MaixCAM-Pro `A18/UART1_RX` <- STM32 `PB10/USART3_TX`
- MaixCAM-Pro `GND` <-> STM32 `GND`
- 串口参数：`115200 8N1`
- MaixCAM-Pro 使用独立稳定供电；串口按 3.3V 逻辑电平处理。

优先使用 MaixCAM-Pro `UART1`。官方文档说明 `UART0` 会输出启动日志，且 `UART0_TX` 也是启动模式检测脚，接 MCU 时更容易引入启动风险。

## 通用帧格式

所有多字节整数为 little-endian。

| Byte | 字段 | 说明 |
| --- | --- | --- |
| 0 | `0xA6` | 帧头 |
| 1 | `0x6A` | 帧头 |
| 2 | `LEN` | 从 `TYPE` 到最后一个载荷字节的长度 |
| 3 | `TYPE` | `0x01` 命令帧，`0x81` 遥测帧 |
| ... | Payload | 固定长度载荷 |
| N-2 | `CRC_L` | CRC16-CCITT low byte |
| N-1 | `CRC_H` | CRC16-CCITT high byte |

CRC 参数：

- 初值：`0xFFFF`
- 多项式：`0x1021`
- 输入范围：从 `LEN` 字节开始，到 payload 最后一个字节结束。

## MaixCAM -> STM32 命令帧

`TYPE = 0x01`，`LEN = 11`，总长度 16 字节，周期建议 20-50Hz。

| Byte | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 4 | `SEQ` | u8 | MaixCAM 递增帧序号 |
| 5 | `MODE` | u8 | `0` idle/stop，`1` run，`2` finish，`3` fault |
| 6 | `FLAGS` | u8 | bit0 line_valid，bit1 finish，bit2 avoiding |
| 7 | `ROUTE_STEP` | u8 | MaixCAM 当前路线阶段 |
| 8 | `CHECKPOINT_REQUEST` | u8 | `0` 无请求，`1` 发送 2.1，`2` 发送 2.2 |
| 9 | `CONFIDENCE` | u8 | MaixCAM 当前决策置信度，`0..100` |
| 10-11 | `VX_MM_S` | i16 | 目标前进速度，STM32 二次限幅 |
| 12-13 | `YAW` | i16 | 目标转向量，STM32 二次限幅 |

STM32 安全策略：

- 超过 `APP_MAIX_LINK_TIMEOUT_MS` 未收到有效命令，进入 `APP_STATE_FAULT` 并停车。
- 速度限幅：`APP_MAIX_MAX_SPEED_MM_S`
- 转向限幅：`APP_MAIX_MAX_YAW`
- `MODE_FINISH` 或 `FLAGS bit1` 会触发终点停车。

## STM32 -> MaixCAM 遥测帧

`TYPE = 0x81`，`LEN = 39`，总长度 44 字节，周期由 `APP_MAIX_TELEMETRY_PERIOD_MS` 控制。

| Byte | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 4 | `SEQ` | u8 | STM32 递增帧序号 |
| 5 | `STATUS_FLAGS` | u8 | bit0 started，bit1 moving，bit2 stm32_fault |
| 6 | `LORA_STATUS` | u8 | `0` idle，`1` sending，`2` sent，`3` error |
| 7 | `BATTERY_PERCENT` | u8 | 电池百分比 |
| 8-9 | `BATTERY_X100` | u16 | 电池电压 * 100 |
| 10-13 | `DISTANCE_MM` | i32 | 编码器里程 |
| 14 | `RADAR_COUNT` | u8 | 有效雷达目标数，最大 3 |
| 15-41 | `RADAR_TARGETS` | 3 * 9 bytes | 雷达目标数组 |

每个雷达目标 9 字节：

| Offset | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| +0 | `VALID` | u8 | 目标有效 |
| +1..2 | `X_MM` | i16 | 横向坐标 |
| +3..4 | `Y_MM` | i16 | 前向坐标 |
| +5..6 | `SPEED_CM_S` | i16 | 速度 |
| +7..8 | `DISTANCE_MM` | u16 | 距离 |

## 软件职责

MaixCAM-Pro：

- 使用 MaixPy `camera.Camera(...)` 取图。
- 使用 `find_blobs` / `get_regression` 类图像算法做多 ROI 巡线。
- 推进路线表 `1.1 -> 1.2 -> 1.3 -> 1.4 -> 1.5`。
- 根据 STM32 遥测中的雷达目标做门检测、2.1/2.2 LoRa 请求和避障侧选择。
- 通过命令帧直接下发 `vx_mm_s` 和 `yaw`。

STM32：

- 解析 MaixCAM 命令帧。
- 发送编码器、电池、雷达、LoRa 状态遥测。
- 保留底盘 20ms 闭环和安全限幅。
- 按 MaixCAM 的 `CHECKPOINT_REQUEST` 调用现有 LoRa 发送逻辑。

## 参考

- MaixCAM-Pro 官方硬件说明：`https://wiki.sipeed.com/hardware/zh/maixcam/maixcam_pro.html`
- MaixPy UART 官方文档：`https://wiki.sipeed.com/maixpy/doc/en/peripheral/uart.html`
- MaixPy Line Tracking 官方文档：`https://wiki.sipeed.com/maixpy/doc/en/vision/line_tracking.html`
- MaixCAM 应用开机自启官方文档：`https://wiki.sipeed.com/maixpy/doc/en/basic/auto_start.html`
