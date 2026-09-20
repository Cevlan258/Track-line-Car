cmake_minimum_required(VERSION 3.22)
project(track_line_host_tests LANGUAGES C)

# 此工程使用宿主编译器，不加载 STM32 交叉编译工具链。
enable_testing()
add_executable(maix_link_protocol_test
    maix_link_protocol_test.c
    ../firmware/stm32/App/Src/maix_link_protocol.c
)
target_include_directories(maix_link_protocol_test PRIVATE
    ../firmware/stm32/App/Inc
)
set_target_properties(maix_link_protocol_test PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
# 保持断言启用，避免 Release 配置静默跳过协议检查。
if(MSVC)
    target_compile_options(maix_link_protocol_test PRIVATE /UNDEBUG /utf-8)
else()
    target_compile_options(maix_link_protocol_test PRIVATE -UNDEBUG)
endif()
add_test(NAME maix_link_protocol COMMAND maix_link_protocol_test)
