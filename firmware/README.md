# C10B 最小电机测试

目标板：C10B（STM32F103RCT6）。

每次复位后的行为：

1. 两个电机保持停止 2 秒。
2. 两个电机以低速转动 2 秒。
3. 两个电机停止，直到下次复位。

## 安全操作

- 烧录时保持电机使能开关为 OFF。
- 测试时把两个驱动轮架空。
- 烧录完成后打开电机使能，再按一次 RESET 按键。
- 遇到异常立即关闭电机使能或断开电池。

## 构建

在 CLion 中打开本目录并选择 `C10B Debug` 预设，或者执行：

```sh
/Applications/CLion.app/Contents/bin/cmake/mac/aarch64/bin/cmake --preset debug
/Applications/CLion.app/Contents/bin/cmake/mac/aarch64/bin/cmake --build --preset debug
```

生成文件位于 `build/c10b_motor_test.hex` 和
`build/c10b_motor_test.bin`。

若 CLion 提示 `clang: error: unsupported argument 'cortex-m3' to option '-mcpu='`，
说明它仍在用旧的宿主机编译器缓存：按 `⇧⌘A` 执行 `Reset Cache and Reload Project`，
并在 `Settings -> Build, Execution, Deployment -> CMake` 中启用 `C10B Debug` 预设。
