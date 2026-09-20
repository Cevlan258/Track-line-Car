# 项目上下文

更新时间：2026-09-21

## 项目定位与展示依据

STM32F103RCT6 + MaixCAM-Pro 智能巡线小车，当前主方案为 MaixCAM 视觉决策、STM32/FreeRTOS 底盘执行。作者确认整体项目独立完成并已跑通完整赛道；没有提供计时、名次或成功率数据，不补写此类结论。

仓库：[Cevlan258/Track-line-Car](https://github.com/Cevlan258/Track-line-Car)，默认分支为 `main`。本地原始检出位于 `E:/STM32Project/car_initial/CAR_FIRST`；上一级只是工作区容器，可能有独立 Git 状态。Git 操作在本仓库根目录执行，**固件构建和 IDE 工作目录改为 `firmware/stm32/`**，宿主测试从仓库根目录执行。

## 当前结构与入口

| 位置 | 用途 |
| --- | --- |
| `README.md` / `README.en.md` | 中英文项目主页 |
| `firmware/stm32/` | 完整 CubeMX/CMake 固件工程，保持内部 App/Core/Drivers/Middlewares 布局 |
| `vision/maixcam/main.py` | 当前视觉、路线、拱门、雷达障碍决策及运动命令生成 |
| `legacy/openmv/` | 旧 OpenMV 脚本、协议和未参与构建的 C 实现 |
| `docs/` | 架构、硬件、部署、协议、验证记录、简历描述 |
| `docs/archive/` | 恢复归档的原始需求、BOM 和简要时间线 |
| `tests/` | Python 回归和宿主 C 协议测试工程 |

固件内部：

- `Core/Src/main.c`：HAL 初始化；`Core/Src/freertos.c`：创建底盘和应用任务。
- `App/Src/app_state.c`：启动、运行、LoRa、终点、故障状态；消费 MaixCAM 命令并更新底盘目标。
- `App/Src/maix_link.c` 和 `maix_link_protocol.c`：USART3 链路与协议。UART 错误后丢弃半帧、清除 ORE 并重新开启中断接收。
- `App/Src/radar.c`：LD2450 协议目标解析；`App/Src/lora.c`：检查点发送。
- `App/Src/ax_robot.c`、`ax_kinematics.c`、`ax_speed.c`、`ax_motor.c`、`ax_servo.c`：周期任务、编码器反馈、电机与舵机控制、内轮减速。
- `App/Inc/ax_ccd.h` 仍为 OLED 旧接口提供类型；对应旧 C 实现已移入历史目录，不参与当前固件构建。

迁移只调整位置、测试入口和文档，不改变算法、任务配置、引脚、控制参数或协议。构建目标仍为 `CAR_FIRST`，不要随展示标题重命名它。

## 策略与控制决策

当前正式策略固定左版地图，跟踪黑线中心；左版约束参与分叉、镜像和圆/矩形区域的路径评分，不是贴左边缘行驶。

MaixCAM 依次执行多扫描带提取、动态 LAB 阈值、白底过滤、候选连接和特征计算，再按区域评分；转向 PID 融合偏移与预瞄误差，按起步、普通和复杂区域切换参数。

当前里程区域配置：

| 区域 | 里程范围 | 主要策略 |
| --- | --- | --- |
| BOOT | 0–700 mm | 低速起步 |
| DEAD | 700–2500 mm | 惩罚短死路与断头支线 |
| MIRROR | 2500–8200 mm | 固定左版路径选择 |
| S | 8200–13200 mm | 连续路径与预瞄 |
| RECT | 13200–16500 mm | 直角和远端可延续出口 |
| CIRCLE | 16500–20500 mm | 闭环惩罚与短时切线锁 |
| FINISH | 20500 mm 后 | 低速搜索红色终点 |

`TRACK_SIGN_LEFT = -1`，`TRACK_FIXED_MAP_SIGN = TRACK_SIGN_LEFT`。以上为代码配置，需随实际赛道和编码器标定调整。

2.1/2.2 检查点使用视觉立柱走廊和里程窗口触发，横梁不是必要条件；雷达只参与障碍区域判断，不作为拱门计数依据。

STM32 保持 20 ms 配置周期的底盘控制、150 ms 命令超时阈值、速度/转向限幅及终点停车。故障显示区分链路超时 `F LINK` 与主动故障 `F CMD`。这些参数不是测得的最坏响应时间。

## 硬件与通信事实

- MaixCAM UART1：A19/TX → STM32 PB11/RX；A18/RX ← PB10/TX，115200 8N1，共地。
- STM32 UART4 接收雷达，代码解析 LD2450 30 字节帧。原始需求中的 HLK-LD2410S 为历史记录。
- LoRa 型号：EBYTE-EWT22A-900BWL22S 测试套件，板载 EWM22A-900BWL22S，900MHz 频段。
- LoRa 参数：team22 / `CIRCUIT_VOYAGE`，信道 10，本机及目标地址 `0x0001`，Fixed Mode、空中速率 2.4 kbps、网络 ID 0、包长 240，中继及密钥关闭。
- 初始化经 USART2 发送 `AT+HMODE=0`、`AT+TRANS=1`、地址和信道参数，最后 `AT+HMODE=1`；检查点报文添加目标地址和信道三字节前缀。
- 命令携带模式、标志、阶段、检查点请求、置信度、速度与转向；遥测携带启动/故障、LoRa、电池、编码器里程和雷达目标。帧格式参见 [协议文档](docs/maixcam_protocol.md)。

## 构建与验证

固件目录中使用 `cmake --preset Debug` / `cmake --build --preset Debug`，Release 同理。工具链需含 newlib-nano；不再使用文档中旧电脑的绝对路径。

仓库根目录执行：

```sh
python -B -m unittest discover -s tests -p "*_test.py"
cmake -S tests -B build/host
cmake --build build/host --config Debug
ctest --test-dir build/host -C Debug --output-on-failure
```

2026-09-21 迁移后验证：39 项 Python 测试通过；MSVC 宿主 C 协议测试通过；ARM GCC 14.3.1 的 Debug / Release 全新构建成功。Flash/RAM 数值与具体环境见 [验证记录](docs/validation.md)。

## 本次整理与同步

交付状态：目录与文档重构提交 `6e24942` 已同步 GitHub `main`；远端中英文 README 与本地内容一致，主页 HTML 已返回正确的标题、表格和 Mermaid 代码块，About 简介及 9 个技术主题已更新。原始本地检出已合入并再次通过 Python、宿主 C、Debug/Release 构建验证。

- 以远端 `99f4be2` 为基础，保留此前 5 个删除提交的历史；在固件目录恢复仍被构建引用的启动汇编和链接脚本，并归档被删除的资料。
- 原 `.clangd` 本地格式修改已随文件迁移保留；旧 `CONTEXT.md` 中过期路径、同步状态和工具绝对路径已被本文件更新。
- 原始本地文件另存于原检出 `.git/showcase-backup-20260921/`，包括旧构建目录和原 MaixCAM 目录；原 `.clangd` / `CONTEXT.md` 修改也保留了一份 Git stash，不提交远端。
- Python 缓存、应用 ZIP 和构建产物不再跟踪；保留第三方许可证，没有为自有代码新增许可证。
- 用户已授权验证后以普通提交同步 `main`，保留仓库名称与 URL，补充 GitHub 简介和主题；不使用强制推送。

## 未验证事项与后续维护

本次没有重新烧录、MaixCAM 设备运行、实车复测或 CubeMX GUI 再生成。未锁定 MaixPy 系统版本，未提供当前完整原理图、实车录像或量化成绩。

修改视觉策略时优先更新 MaixCAM 宿主行为测试；修改协议需同步两端及文档。保持 STM32 独立故障保护。资料与实车成果补充后同步两种语言的 README 和简历描述。
