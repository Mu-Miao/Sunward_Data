set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# CLion 从 Dock 启动时不会继承 shell 的 PATH，因此显式探测常用安装位置。
set(ARM_EABI_HINTS
    /opt/homebrew/bin
    /usr/local/bin
    /usr/bin
)
file(GLOB ARM_EABI_EXTRA_HINTS
    "/opt/homebrew/Caskroom/arm-gcc/*/bin"
    "/usr/local/Caskroom/arm-gcc/*/bin"
    "$ENV{HOME}/Downloads/arm-gnu-toolchain-*/bin"
    "$ENV{HOME}/Applications/arm-gnu-toolchain-*/bin"
    "$ENV{HOME}/arm-gnu-toolchain-*/bin"
)
list(APPEND ARM_EABI_HINTS ${ARM_EABI_EXTRA_HINTS})

find_program(CMAKE_C_COMPILER NAMES arm-none-eabi-gcc HINTS ${ARM_EABI_HINTS} REQUIRED)
find_program(CMAKE_ASM_COMPILER NAMES arm-none-eabi-gcc HINTS ${ARM_EABI_HINTS} REQUIRED)
find_program(CMAKE_OBJCOPY NAMES arm-none-eabi-objcopy HINTS ${ARM_EABI_HINTS} REQUIRED)
find_program(CMAKE_SIZE NAMES arm-none-eabi-size HINTS ${ARM_EABI_HINTS} REQUIRED)
find_program(CMAKE_GDB NAMES arm-none-eabi-gdb HINTS ${ARM_EABI_HINTS})

get_filename_component(ARM_EABI_BIN "${CMAKE_C_COMPILER}" DIRECTORY)
list(PREPEND CMAKE_PROGRAM_PATH "${ARM_EABI_BIN}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
