cmake_minimum_required(VERSION 3.22)

# 未显式传入 -DCMAKE_TOOLCHAIN_FILE 时（例如 CLion 使用了旧版 Debug 配置），
# 自动加载交叉编译工具链，避免用宿主机 clang 编译 ARM 目标。
if(NOT CMAKE_TOOLCHAIN_FILE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake")
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake")
endif()

project(c10b_motor_test C ASM)

if(NOT CMAKE_CROSSCOMPILING)
    message(FATAL_ERROR
        "当前使用的是宿主机编译器 ${CMAKE_C_COMPILER}，它不支持 -mcpu=cortex-m3。\n"
        "请在 CLion 的 Settings -> Build, Execution, Deployment -> CMake 中启用 “C10B Debug” 预设，\n"
        "或改用命令行：cmake --preset debug && cmake --build --preset debug")
endif()

set(CPU_FLAGS
    -mcpu=cortex-m3
    -mthumb
)

add_executable(c10b_motor_test.elf
    src/main.c
    src/startup_stm32f103xe.s
)

target_include_directories(c10b_motor_test.elf PRIVATE
    vendor/minimal-libc
    vendor/CMSIS/Core/Include
    vendor/CMSIS/Device/Include
)

target_compile_definitions(c10b_motor_test.elf PRIVATE
    STM32F103xE
)

target_compile_options(c10b_motor_test.elf PRIVATE
    ${CPU_FLAGS}
    -O0
    -g3
    -Wall
    -Wextra
    -ffunction-sections
    -fdata-sections
)

target_link_options(c10b_motor_test.elf PRIVATE
    ${CPU_FLAGS}
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F103RCTx_FLASH.ld
    -nostdlib
    -nostartfiles
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/c10b_motor_test.map
)

add_custom_command(TARGET c10b_motor_test.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:c10b_motor_test.elf>
            ${CMAKE_CURRENT_BINARY_DIR}/c10b_motor_test.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:c10b_motor_test.elf>
            ${CMAKE_CURRENT_BINARY_DIR}/c10b_motor_test.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:c10b_motor_test.elf>
    COMMENT "Creating HEX and BIN images"
)
