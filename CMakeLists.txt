cmake_minimum_required(VERSION 3.22)

#
# 此文件只生成一次，
# 即使多次运行转换器也不会重新生成。
#
# 用户可按需修改此文件
#

# 设置编译器配置
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# 定义构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# 设置项目名称
set(CMAKE_PROJECT_NAME CAR_FIRST)

# 启用编译命令导出，便于 clangd 等工具索引
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# 核心项目设置
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# 启用 CMake 对 ASM 和 C 语言的支持
enable_language(C ASM)

# 创建可执行目标
add_executable(${CMAKE_PROJECT_NAME})

# 添加 STM32CubeMX 生成的源码
add_subdirectory(cmake/stm32cubemx)

# 设置链接目录
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # 添加用户定义的库搜索路径
)

# 添加可执行目标源码
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    App/Src/app_start.c
    App/Src/app_state.c
    App/Src/app_time.c
    App/Src/app_uart.c
    App/Src/display_ssd1309.c
    App/Src/gate_detector.c
    App/Src/lora.c
    App/Src/maix_link.c
    App/Src/maix_link_protocol.c
    App/Src/radar.c
    App/Src/ax_beep.c
    App/Src/ax_delay.c
    App/Src/ax_encoder.c
    App/Src/ax_key.c
    App/Src/ax_kinematics.c
    App/Src/ax_led.c
    App/Src/ax_motor.c
    App/Src/ax_robot.c
    App/Src/ax_servo.c
    App/Src/ax_speed.c
    App/Src/ax_vin.c
)

# 添加包含路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App/Inc
)

# 添加项目符号（宏）
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # 添加用户定义的符号
)

# 移除使用 C++ 文件时错误的 libob.a 库依赖
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# 添加链接库
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # 添加用户定义的库
)
