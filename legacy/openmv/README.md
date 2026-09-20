# OpenMV 历史方案

[返回主页](../../README.md) · [当前系统架构](../../docs/architecture.md)

这里保存早期 OpenMV 视觉脚本、协议及未参与当前固件构建的旧应用实现，供研究项目演进使用，不是当前可直接构建的第二套整车方案。

- `main.py`：历史 OpenMV 脚本。
- [protocol.md](protocol.md)：历史感知帧协议。
- `stm32/Src/ax_ccd.c`：旧感知帧接收与兼容处理。
- `stm32/Src/ax_function.c`、`stm32/Inc/ax_function.h`：旧巡线控制。

当前入口为 `vision/maixcam/main.py`。旧源码依赖当前固件中的若干驱动和类型，没有单独维护的构建入口。

`firmware/stm32/App/Inc/ax_ccd.h` 仍保留在活动固件中，因为 OLED 旧显示接口使用其中的类型；本次保留这些接口，避免目录整理改变行为。底盘中相关旧变量也未作无关清理。
