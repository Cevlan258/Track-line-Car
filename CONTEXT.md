# 项目上下文

更新时间：2026-06-02

## 项目定位

本项目是 STM32F103RCT6 智能巡线小车固件，当前真实项目根目录是 `CAR_FIRST/`。上一级 `car_initial/` 只是工作区容器，里面可能还有独立 `.git` 状态；构建、测试、提交和推送都应在 `CAR_FIRST/` 内执行。

比赛任务包括：

- 黑线巡线：沿 3 cm 黑线行驶，经过直角弯、U 形段、S 弯、矩形/圆形复杂区和终点。
- 无线通信：在 2.1、2.2 标记点通过 LoRa 发送队号、队名、时间和耗时。
- 拱门通信：使用 MaixCAM 几何视觉识别 2.1/2.2 拱门，通过 LoRa 发送队号、队名、时间和耗时。
- 障碍检测：使用 24 GHz 雷达检测任务 3 障碍箱金属板位置，选择空侧绕行。

当前正式比赛策略已确认走左版地图。这里的“左版”不是贴黑线左边缘行驶，而是固定左侧赛道拓扑；小车仍跟踪黑线中心，但在多候选、分叉、镜像区和圆/矩形复杂区优先选择符合左版地图的路径。

## 当前架构

### STM32 侧

- `Core/Src/main.c`：HAL 初始化入口，初始化外设后进入 FreeRTOS。
- `Core/Src/freertos.c`：创建 `Robot_Task` 和 `AppState_Task`。
- `App/Src/app_state.c`：主状态机，负责启动、运行、LoRa、终点、故障状态；消费 MaixCAM 命令并写入底盘目标速度。
- `App/Src/maix_link.c`、`App/Src/maix_link_protocol.c`：MaixCAM 串口协议收发，使用 `USART3`。
- `App/Src/radar.c`：雷达帧解析，向 MaixCAM 遥测提供目标列表。
- `App/Src/lora.c`：LoRa 检查点发送。
- `App/Src/ax_robot.c`、`App/Src/ax_kinematics.c`、`App/Src/ax_motor.c`、`App/Src/ax_servo.c`：底盘、电机、舵机闭环执行。
- `App/Src/ax_function.c`、`openmv/main.py`：旧 OpenMV/CCD 巡线路径仍保留，但当前主控巡线逻辑已迁移到 MaixCAM，不应优先修改旧路径。

### MaixCAM 侧

- `maixcam/main.py`：当前主要巡线、路线推进、视觉拱门识别、雷达障碍处理、终点判断和运动命令生成逻辑。
- `docs/maixcam_protocol.md`：MaixCAM 与 STM32 的串口协议说明。
- `tests/maixcam_standalone_test.py`：MaixCAM 策略的 host 侧静态/行为测试。

MaixCAM -> STM32 命令帧包含 `MODE`、`FLAGS`、`ROUTE_STEP`、`CHECKPOINT_REQUEST`、`CONFIDENCE`、`VX_MM_S`、`YAW`。STM32 -> MaixCAM 遥测帧包含启动状态、LoRa 状态、电池、编码器里程和雷达目标。拱门事件由 MaixCAM 图像识别产生，雷达目标只用于任务 3 障碍判断。

## 巡线策略摘要

`maixcam/main.py` 每帧执行以下流程：

1. 多个水平扫描带提取黑线候选。
2. 使用动态 LAB 阈值适应环境亮度。
3. 要求黑线旁边存在白底，减少阴影或黑色障碍误检。
4. 把扫描带线段连成候选路径。
5. 计算路径特征：偏移、预瞄偏移、曲率、分支数、死路、闭环分数。
6. 按当前区域和左版地图分段偏好给候选路径打分。
7. 生成速度和转向命令发给 STM32。
8. 在 2.1/2.2 的里程辅助窗口内用几何门框识别触发 LoRa 请求。
9. 在任务 3 障碍区域内用雷达目标判断金属板侧别，选择空侧绕行。

当前区域按编码器里程粗分：

- `BOOT`：0-450 mm，低速起步。
- `DEAD`：450-2500 mm，强惩罚短死路和断头支线。
- `MIRROR`：2500-4700 mm，固定左版地图，不再动态猜左右镜像。
- `S`：4700-6100 mm，按连续路径和预瞄偏差通过 S 弯。
- `RECT`：6100-8000 mm，允许直角，优先远端可延续且符合左版出口的路径。
- `CIRCLE`：8000-10300 mm，惩罚闭环圆周，使用短时切线锁避免被圆吸住。
- `FINISH`：10300 mm 后，低速寻找红色终点区，识别后发送停车。

左版地图核心常量：

- `TRACK_SIGN_LEFT = -1`
- `TRACK_FIXED_MAP_SIGN = TRACK_SIGN_LEFT`

含义：图像坐标中 `x < CENTER_X` 的路径候选优先作为左版地图出口。该约束只参与路径选择，不改变“跟踪黑线中心”的基本控制目标。

## 目录结构

```text
CAR_FIRST/
  App/
    Inc/                    应用层头文件
    Src/                    应用层源文件：状态机、Maix 链路、雷达、LoRa、底盘控制
  Core/
    Inc/                    CubeMX 生成的核心头文件
    Src/                    HAL 初始化、FreeRTOS 任务、外设初始化
  Drivers/                  STM32 HAL/CMSIS
  Middlewares/              FreeRTOS
  cmake/                    CMake 工具链和 CubeMX 子工程配置
  docs/
    maixcam_protocol.md     当前 MaixCAM 主控协议
    openmv_protocol.md      旧 OpenMV 协议参考
  maixcam/
    main.py                 当前视觉和路线主控脚本
  openmv/
    main.py                 旧 OpenMV 脚本，保留参考
  tests/
    maix_link_protocol_test.c
    maixcam_standalone_test.py
  CMakeLists.txt            顶层固件构建入口
  CMakePresets.json
  CAR_FIRST.ioc             CubeMX 工程
  CONTEXT.md                本文件
```

## 构建和测试

推荐使用 STM32CubeIDE 插件安装的工具链，不要使用不完整的 Xilinx ARM GCC 路径。此前验证可用的工具链路径为：

- GCC：`C:\Users\ASUS\AppData\Local\stm32cube\bundles\gnu-tools-for-stm32\14.3.1+st.2\bin`
- CMake：`C:\Users\ASUS\AppData\Local\stm32cube\bundles\cmake\4.2.3+st.1\bin\cmake.exe`
- Ninja：`C:\Users\ASUS\AppData\Local\stm32cube\bundles\ninja\1.13.2+st.1\bin\ninja.exe`

Host 侧 MaixCAM 测试：

```powershell
& 'C:\Users\ASUS\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -B -m unittest tests.maixcam_standalone_test
& 'C:\Users\ASUS\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -B -m py_compile maixcam\main.py
```

协议 C 测试可参考：

```powershell
# 具体编译命令按当前工具链环境调整
tests\maix_link_protocol_test.c
```

固件构建建议使用干净构建目录，例如 `build/stm32cubeide-Debug`。如果遇到 `cannot find -lg_nano` 或 `cannot find -lc_nano`，通常是工具链路径错了，应切换到 STM32CubeIDE bundle 工具链并重新配置干净构建目录。

## 当前开发状态

当前工作分支：`codex/maixcam-gate-radar-adapt`

LoRa 配置已按 2026-06-08 场次 team22 左赛道要求更新：

- 队伍：team22，队名 `CIRCUIT_VOYAGE`。
- 左赛道参数：信道 `10`，目标地址 `0x0001`。
- 模块参数：900 MHz 频段，定点模式，空中速率 2.4 kbps，网络 ID `0x00`，包长 240，中继/密钥关闭。
- `LoRa_Init()` 会通过 `USART2` 发送 AT 指令写入参数，并切回 `AT+HMODE=1`。
- `LoRa_SendCheckpoint()` 在每帧前添加定点模式前缀 `00 01 0A`，随后发送队号、队名、时间、检查点和起点耗时文本。

当前未提交改动包括：

- `App/Inc/app_config.h`、`App/Src/lora.c`：team22 左赛道 LoRa 参数和定点模式发送前缀。
- `CONTEXT.md`：记录 LoRa 配置更新。
- `maixcam/main.py`：动态阈值、白底侧向过滤、左版地图评分、视觉拱门识别、雷达障碍避让、保守限速和丢线左搜。
- `tests/maixcam_standalone_test.py`：MaixCAM 策略 host 侧测试，覆盖视觉拱门和雷达障碍解耦。
- `docs/maixcam_protocol.md`：补充正式比赛固定左版地图、视觉拱门和雷达障碍职责说明。

最近验证结果：

- `python -B -m unittest tests.maixcam_standalone_test`：11 tests OK
- `python -B -m py_compile maixcam/main.py`：通过

继续开发前建议先执行：

```powershell
git status -sb
git branch --show-current
```

确认在 `CAR_FIRST/` 内，并确认当前分支和未提交改动符合预期。

## 跨设备继续开发注意事项

- 打开 VS Code 时优先打开 `CAR_FIRST/`，不要只打开上一级 `car_initial/`，否则可能出现多个 CMake 项目混淆。
- 提交和推送应在 `CAR_FIRST/` 内执行；上一级目录也可能有 `.git`，不要误操作。
- 中文文件请用 UTF-8 读取/编辑，PowerShell 中建议使用 `Get-Content -Encoding UTF8`。
- MaixCAM 当前连接使用 `UART1`，STM32 侧是 `USART3`：MaixCAM `A19/TX` -> STM32 `PB11/RX`，MaixCAM `A18/RX` <- STM32 `PB10/TX`。
- STM32 安全职责不应被削弱：MaixCAM 可下发速度/转向，但 STM32 仍要保持超时停车、速度限幅、转向限幅和终点停车。
- 雷达不再作为 2.1/2.2 拱门识别依据；如果 LoRa 触发异常，优先调 `VisionGateDetector` 的 ROI、几何尺寸和里程辅助窗口。
- 修改巡线策略时优先补充 `tests/maixcam_standalone_test.py`，再改 `maixcam/main.py`。
