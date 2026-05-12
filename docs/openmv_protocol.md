# OpenMV 巡线协议

## 硬件连接

- OpenMV `P4/TX` -> STM32 `PB11/USART3_RX`
- OpenMV `P5/RX` -> STM32 `PB10/USART3_TX`
- OpenMV `GND` -> STM32 `GND`
- OpenMV 使用独立稳定 5V 供电

串口参数：`115200 8N1`。

## OpenMV -> STM32

固定 14 字节二进制帧：

| Byte | 字段 | 说明 |
| --- | --- | --- |
| 0 | `0xAA` | 帧头 |
| 1 | `0x55` | 帧头 |
| 2 | `LEN` | 固定为 `10` |
| 3 | `SEQ` | 帧序号 |
| 4 | `FLAGS` | bit0 线有效，bit1 Start，bit2 Finish，bit3 Marker |
| 5 | `OFFSET_L` | 偏移低字节，int16 little-endian |
| 6 | `OFFSET_H` | 偏移高字节，范围建议 `-64..63` |
| 7 | `LEFT` | 归一化左边界，`0..127` |
| 8 | `RIGHT` | 归一化右边界，`0..127` |
| 9 | `SEG_COUNT` | 近端 ROI 线段数量 |
| 10 | `SELECTED` | OpenMV 当前选择的线段索引 |
| 11 | `ROAD_TYPE` | 路况类型 |
| 12 | `CONF` | 置信度，`0..100` |
| 13 | `XOR` | byte 2 到 byte 12 的异或 |

`ROAD_TYPE`：

- `0` unknown
- `1` straight
- `2` left_curve
- `3` right_curve
- `4` fork
- `5` cross
- `6` t_left
- `7` t_right
- `8` finish

## STM32 -> OpenMV

STM32 在路线状态机需要选择分支时发送 4 字节偏好命令：

| Byte | 字段 | 说明 |
| --- | --- | --- |
| 0 | `0xA5` | 帧头 |
| 1 | `0x5A` | 帧头 |
| 2 | `PREF` | `0` 最近线，`1` 最左线，`2` 最右线 |
| 3 | `XOR` | byte 0 到 byte 2 的异或 |

OpenMV 根据该偏好在多线段场景中选择输出的线中心。

## 当前路线表

STM32 按 `1.1 -> 1.2 -> 1.3 -> 1.4 -> 1.5` 执行：

1. 第一次路口选择最左线，进入 1.1 U 形长回路。
2. 第二次路口选择最右线，离开 1.1 回到主线。
3. 第三次路口选择最近线，通过 1.3 方框区。
4. 第四次路口选择最近线，通过 1.4 圆/矩形区。
5. 第五次路口进入低速接近终点，检测到红色 Finish 后停车。
