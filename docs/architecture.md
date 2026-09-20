# 系统架构与实现导览

[返回主页](../README.md)

## 职责划分

MaixCAM-Pro 负责图像采集、黑线路径提取、区域推进、检查点识别、障碍侧别决策和目标运动量生成。STM32F103RCT6 负责外设、实时执行、遥测和停车保护。两者通过 UART 命令/遥测链路协作，协议详见 [MaixCAM 串口协议](maixcam_protocol.md)。

| 子系统 | 入口 | 职责 |
| --- | --- | --- |
| 视觉和路线 | [main.py](../vision/maixcam/main.py) | 扫描带提取、路径连接、评分、转向、拱门和终点识别 |
| 应用状态 | [app_state.c](../firmware/stm32/App/Src/app_state.c) | 启动、运行、LoRa、终点和故障处理 |
| 通信 | [maix_link.c](../firmware/stm32/App/Src/maix_link.c) | 中断接收、帧同步、命令发布、遥测发送与 UART 错误恢复 |
| 协议 | [maix_link_protocol.c](../firmware/stm32/App/Src/maix_link_protocol.c) | 固定长度帧和 CRC16，可在宿主环境测试 |
| 底盘 | [ax_kinematics.c](../firmware/stm32/App/Src/ax_kinematics.c) | 编码器采样、内轮减速、电机闭环和舵机输出 |
| 传感器与任务通信 | [radar.c](../firmware/stm32/App/Src/radar.c)、[lora.c](../firmware/stm32/App/Src/lora.c) | 雷达目标解析、LoRa 参数配置和检查点报文 |

## 每帧决策流程

1. 从近到远的多个水平扫描带提取黑线，结合动态 LAB 阈值和旁侧白底约束过滤候选。
2. `PathGraph` 连接候选，提取偏移、预瞄偏移、曲率、分支、死路与闭环特征。
3. `ZonePlanner` 使用编码器里程辅助定位区域；`PathScorer` 按连续性和固定左版地图规则选路。
4. `SteeringPid` 根据起步、普通和复杂区域使用不同参数，结合偏移与预瞄误差生成转向量；`CommandPlanner` 产生速度和转向命令。
5. `VisionGateDetector` 在里程窗口内识别两侧立柱围成的走廊，产生 2.1 / 2.2 检查点请求。雷达不承担拱门识别。
6. `RadarObstaclePlanner` 使用遥测中的雷达目标判断障碍侧别；终点识别触发停车命令。

固定左版规则用于路径选择，车辆仍跟踪黑线中心。里程区域及阈值是赛道相关配置，需要随实际赛道和机械参数标定。

## 实时执行与故障保护

FreeRTOS 创建底盘任务、应用任务及默认任务。底盘任务通过 `vTaskDelayUntil` 按 20 ms 配置周期采样编码器并执行运动控制；应用任务处理命令、任务状态和显示。

电机控制使用现有增量式反馈实现，转向由舵机承担，并按转向量降低内侧轮目标速度。这里的 20 ms 是调度配置，不代表已经测量过调度抖动。

STM32 对有效命令进行速度和转向限幅，命令超时阈值为 150 ms。终点命令或终点标志触发停车；链路超时显示 `F LINK`，MaixCAM 主动故障显示 `F CMD`。USART3 错误回调会丢弃半帧并重启接收。

## 维护边界

- 当前运行入口是 MaixCAM 方案；[旧 OpenMV 代码](../legacy/openmv/README.md)仅作历史参考。
- 固件内部保持 CubeMX 常规结构；不要在仓库根目录生成新的 `Core/` 或 `Drivers/`。
- `App/Inc/ax_ccd.h` 仍提供 OLED 旧显示接口所需类型，不表示旧巡线逻辑参与构建。
- 修改协议需同时更新 C 编解码、MaixCAM 编解码、协议文档和宿主测试；本次目录迁移没有修改协议。
