# 构建、测试与部署

[返回主页](../README.md) · [硬件连接](hardware.md) · [验证记录](validation.md)

## 1. 准备环境

- Git、Python 3：宿主测试只使用标准库，不需要在电脑安装 MaixPy。
- CMake 3.22+、Ninja、ARM GNU 工具链：必须包含 `arm-none-eabi-gcc`、`arm-none-eabi-g++` 和 newlib-nano；STM32Cube 工具包可提供这些工具。
- 宿主 C 测试使用电脑本机编译器，例如 Windows 的 Visual Studio Build Tools（安装 C++ 桌面开发组件）或 Linux 的 GCC；不要使用 ARM 裸机编译器运行宿主测试。
- 设备部署使用 STM32CubeProgrammer / ST-Link，以及 MaixCAM-Pro 上的 MaixPy 和电脑上的 MaixVision。

本次验证环境为 Windows、Python 3.14、ARM GCC 14.3.1、CMake 4.2.3、Ninja 1.13.2，宿主编译器为 MSVC 14.50。MaixPy 设备系统版本未在仓库中锁定，复现时应记录所用版本。

下载工具可从 [STM32Cube 开发工具](https://www.st.com/en/development-tools/stm32cubeide.html)、[CMake](https://cmake.org/download/)及 [MaixVision 官方使用说明](https://wiki.sipeed.com/maixpy/doc/en/basic/maixvision.html)进入。

## 2. 获取项目并验证工具

```sh
git clone https://github.com/Cevlan258/Track-line-Car.git
cd Track-line-Car
python --version
cmake --version
ninja --version
arm-none-eabi-gcc --version
```

工具未被找到时，将安装目录下的 `bin` 加入当前终端的 `PATH`。不要将某台电脑的绝对路径写入共享配置。Windows 若安装了 STM32Cube bundles，可在 PowerShell 中按实际安装版本设置，例如：

```powershell
$bundleRoot = Join-Path $env:LOCALAPPDATA 'stm32cube/bundles'
$toolDirs = @(
    "$bundleRoot/gnu-tools-for-stm32/14.3.1+st.2/bin"
    "$bundleRoot/cmake/4.2.3+st.1/bin"
    "$bundleRoot/ninja/1.13.2+st.1/bin"
)
$env:PATH = ($toolDirs -join ';') + ';' + $env:PATH
```

版本目录必须存在；安装其他版本时替换对应目录名。`cannot find -lc_nano` 或 `-lg_nano` 通常说明所选工具链不完整或错误。

## 3. 构建 STM32 固件

从仓库根目录执行：

```sh
cd firmware/stm32
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

| 配置 | 输出 |
| --- | --- |
| Debug | `firmware/stm32/build/Debug/CAR_FIRST.elf` |
| Release | `firmware/stm32/build/Release/CAR_FIRST.elf` |

CMake 目标名仍为 `CAR_FIRST`。启动汇编与链接脚本位于固件目录，均为必需源文件，不能当作构建产物删除。修改工具链或从旧目录迁移后，应使用新的构建目录，避免复用包含旧绝对路径的缓存。

在 STM32CubeIDE / VS Code 中打开 **`firmware/stm32/`**，其 `.vscode`、`.settings` 和 `.clangd` 路径相对于该目录。VS Code 的现有配置依赖 STM32Cube 扩展提供的 `cube-cmake` 等命令；命令行构建不依赖这些扩展。

CubeMX 使用同目录的 `CAR_FIRST.ioc`，生成位置保持在固件目录。修改配置后需检查生成差异，尤其顶层 CMake 的 `App/Src` 源文件列表；本次未执行 GUI 再生成验证。

## 4. 运行宿主测试

以下命令均从仓库根目录执行。

Python 测试自动发现全部 `*_test.py`：

```sh
python -B -m unittest discover -s tests -p "*_test.py"
```

C 协议测试独立于 ARM 固件工程，CMake 自动选择宿主编译器：

```sh
cmake -S tests -B build/host
cmake --build build/host --config Debug
ctest --test-dir build/host -C Debug --output-on-failure
```

Windows 验证使用 Visual Studio 2026 生成器；如需显式指定，在第一条命令后加 `-G "Visual Studio 18 2026" -A x64`。Linux 可使用系统 GCC。不同生成器请使用不同构建目录。

C 测试覆盖命令编码/解码、CRC 错误拒绝和遥测布局。Python 测试含 MaixCAM 策略行为测试和固件源码静态检查，不能验证真实相机效果、调度时序或电气连接。

## 5. 部署与整车检查

1. 按[硬件连接](hardware.md)接线并核对供电；通过 ST-Link 与 STM32CubeProgrammer 将所选配置的 `CAR_FIRST.elf` 下载到 STM32。也可使用固件目录中的现有调试配置。
2. 在 MaixVision 连接 MaixCAM-Pro，打开 `vision/maixcam/`，运行项目并检查图像与串口状态。实际程序在设备上运行，电脑端测试不执行真实 `maix` 外设 API。
3. 使用目录中的 `app.yaml` 打包和安装应用；包文件写入 `dist/`，不提交版本库。运行和安装步骤以 [MaixVision 文档](https://wiki.sipeed.com/maixpy/doc/en/basic/maixvision.html)和[应用开发文档](https://wiki.sipeed.com/maixpy/doc/en/basic/app.html)为准。
4. 首次联调先架空车轮，检查电机方向、编码器符号、舵机中位、串口遥测和断开命令链路后的停车，再进行低速赛道调试。
5. 按实际摄像头位置、赛道长度和机械结构校准视觉阈值、里程区域及转向参数。代码中的队号、队名、LoRa 信道和地址属于原赛道配置，复用时应同步修改并测试。

本轮仅验证软件测试和固件构建，没有执行上述设备下载、安装或实车操作。
