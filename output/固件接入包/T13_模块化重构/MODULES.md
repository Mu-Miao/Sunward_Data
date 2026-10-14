# Sunward 模块说明

2026-10-11：以接入前实际 Sunward 为基线拆分原 `dual_pi_test.c`，保留当前调参值与任务行为。

## 文件职责

每个 `.c` 单独编译；没有通过 `#include "xxx.c"` 拼接实现。对应头文件定义模块接口。

| 文件 | 职责 | 修改此处的常见情况 |
| --- | --- | --- |
| `Core/Src/car_app.c` | 初始化、前台调度、统一停止入口、10ms采样与保护顺序 | 调整整车调度或停止原因 |
| `Core/Src/car_pi.c` | 不依赖HAL的PI内核，实际dt积分、条件积分抗饱和 | 修改控制算法 |
| `Core/Src/car_wheel.c` | 编码器换算、两轮PI调用、反向/超速/堵转检查 | 修改轮速控制或编码器处理 |
| `Core/Src/car_heading.c` | MPU零偏校准、角度积分、航向差速 | 修改校准与直线航向保持 |
| `Core/Src/car_line.c` | LF04 GPIO读取、物理左右排列、有效图样判定、巡线差速 | 修改巡线处理 |
| `Core/Src/car_button.c` | PA0消抖、长短按、选择/校准/启动/停止意图 | 修改按键操作 |
| `Core/Src/car_route.c` | 模式选择与启动门槛、定距、速度曲线、停稳、AB/BC/CD/DA换段 | 修改路线和终点逻辑 |
| `Core/Src/car_command.c` | USB/蓝牙分流、命令解析、STOP/接收故障优先 | 修改调试指令 |
| `Core/Src/car_power.c` | ADC采样、低压与ADC失败停车 | 修改电池监测 |
| `Core/Src/car_signal.c` | 完成声光与校准就绪LED | 修改灯或蜂鸣器 |
| `Core/Src/car_ui.c` | OLED文本快照与串口遥测 | 修改显示和诊断字段 |
| `Core/Inc/car_config.h` | 全部原有 `DPT_*` 参数 | 日常调参入口 |
| `Core/Inc/car_state.h` | 按模块分组的内部状态类型 | 模块需要新增持久状态 |
| `Core/Inc/car_internal.h` | 内部模块接口汇总 | 供各模块使用，不用于main的业务操作 |
| `Core/Inc/car_math.h` | 回绕计数、取整、角度归一化 | 共享数学工具 |
| `Core/Src/dual_pi_test.c` | 旧 `DualPITest_*` 接口的转发兼容层 | 兼容旧调用，不承载控制逻辑 |

已有 `lab_uart_async`、`mpu6050`、`soft_i2c`、`task_oled` 保持独立，仍负责底层硬件访问。GPIO/定时器/串口配置、CubeMX的ioc及中断归属保持原样。EIDE增加显式CarModules源文件组。

## main怎么调用

`main.c` 包含 `car_app.h`，CubeMX及电机初始化完成后调用：

```c
CarApp_Begin(&huart1, &htim8, &htim4, &hadc1, Lab_MotorRawAB);
CarApp_EnableBluetooth(&huart3);
CarApp_EnableHeading(lab_mpu_ready);
```

实际主程序保留对初始化返回值的检查；上面的短代码仅展示接口位置。

前台循环持续调用 `CarApp_Poll(lab_tick10_count)`。TIM2中断只递增原10ms计数；UART中断只处理原异步收发队列。所有控制模块在同一前台上下文运行。

`CarApp` 私有持有一个 `CarAppContext`，按 heading/button/line/route/wheel/command/power/signal/ui/runtime 分组。模块函数显式接收上下文，不再依赖散落的全局变量。`CarApp_TestContext()` 仅在主机测试宏 `CAR_APP_TEST` 下存在，正式固件不会导出测试访问口。

## 一次轮询的顺序

1. 读取使能和按键，执行OFF、超时、制动超时及声光到期处理。
2. 检查ADC/低压，再扫描两个命令口；STOP/接收故障先于运动请求。
3. 处理按键意图和校准/运动请求，不积压待启动命令。
4. 在新10ms节拍测轮速，检查节拍，采样MPU与LF04，读取传感器后再次核对使能。
5. 更新里程、路线换段、减速/起步目标和停稳状态。
6. 两轮各自通过控制与保护检查后，才一起提交电机命令。
7. 更新诊断、就绪LED与OLED；发送拥堵丢诊断帧，不阻塞控制。

统一停车由 `CarApp_Stop()` 处理：零输出、清PI、处理声光与路线状态。模块发现故障也通过此入口停止，避免某个子模块下一拍覆盖停车。

## 调参和验证

原先在 `dual_pi_test.h` 修改的 `DPT_*` 数值，现在统一改 `Core/Inc/car_config.h`。旧头文件通过包含新配置保持编译兼容。

本轮保持原参数，包含左右CPR/PI、轮周长、黑0白1、物理左→右O4/O3/O2/O1、半圆400×π、任务二弧段1256mm、巡线200/50mm/s与增益64等。45秒仍是任务二开发保护上限，实际30秒验收状态依据实测记录。

在工程根目录运行：

```sh
python3 tools/tests/run_tests.py
python3 tools/tests/run_tests.py --sanitize
```

需要主机C编译器；`--sanitize`需要支持ASan/UBSan的Clang或GCC。测试临时产物自动清理。模拟HAL只供测试，不能加入固件包含路径。EIDE仍扫描Core和Drivers，并显式引用CarModules；tools/tests不参与固件编译。

回归包含旧GO100/GO20/航向/手动、一米与半圆/整圈、三模式按键、无蓝牙操作、丢线、错航向、换段边界、停稳、STOP/OFF/两端RX/MPU/FAST/GAP/LOW/ADC/STALL/REV、时间回绕与显示长度。

本轮旧单文件与模块版本还执行了同场景的行为对照：电机写入、GPIO声光、串口发送、OLED文本与轮询后状态，共242744个事件及采样，FNV摘要相同。该比较是主机模拟证据，不能证明实物波形与所有运行工况完全相同。

重构前最新固件FLASH34988B/RAM5360B；暂存模块固件FLASH35036B/RAM5360B。完整ARM编译链接通过；新模块未产生警告，原main未使用诊断函数的警告仍存在。本轮不执行烧录、串口连接或电机运动；现场验收仍按项目实测记录管理。

## 构建与恢复

EIDE在虚拟源文件组 `CarModules` 显式引用全部11个新模块，Debug/Release同时排除这些文件的普通目录引用，避免重复编译。源文件仍存放在Core/Src。这能避免编辑器源目录缓存遗漏新文件；其余构建/烧录选项保持原样。Debug构建参数同步包含模块，接入验证使用独立参数快照，避免编辑器重生成过程覆盖验证清单。

旧接入包是历史版本，不能单独覆盖新 `dual_pi_test.c/.h` 来恢复整车代码；本轮提供完整接入前源码、Debug构建和文件清单备份，恢复时应一并还原main、配置、控制源码与构建参数，并移除这次新增文件。备份位置与生成固件哈希记录在车车目录的模块化接入记录中。
