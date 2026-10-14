# 实验3 定时器 PWM 与编码器详细操作手册

## 1 使用范围与已确认信息

本手册面向当前 C10B 小车控制板，按以下已确认或由课程材料给出的信息编写：

- MCU：STM32F103RCT6，LQFP64 封装。
- 外部晶振：8 MHz。
- 系统时钟：72 MHz，PLL 倍频系数为 9。
- APB1 总线时钟：36 MHz；由于 APB1 分频不为 1，APB1 定时器时钟为 72 MHz。
- 旧工程已验证 USART1 使用 PA9/PA10，SWD 使用 PA13/PA14。
- 旧工程已验证 TIM4 编码器使用 PB6/PB7，Encoder Mode 为 TI12，输入滤波值为 10。

以上 MCU、时钟和旧工程配置可在 `/Users/mzh/STM32Cube/2/2.ioc` 中核对。

实验材料存在两套定时器分配：简要指导书使用 TIM2 PWM、TIM3 编码器、TIM6 10 ms；报告骨架针对双电机 C10B 板卡使用 TIM3 四通道 PWM、TIM4/TIM8 编码器、TIM2 20 ms。本手册采用报告骨架中的双电机方案：

| 功能 | 定时器与通道 | 引脚 | 本手册用途 |
|---|---|---|---|
| 电机 A | TIM3 CH1 / CH2 | PA6 / PA7 | 两个方向的 PWM 输入 |
| 电机 B | TIM3 CH3 / CH4 | PB0 / PB1 | 两个方向的 PWM 输入 |
| 编码器 A | TIM8 CH1 / CH2 | PC6 / PC7 | AB 相计数 |
| 编码器 B | TIM4 CH1 / CH2 | PB6 / PB7 | AB 相计数 |
| 周期采样 | TIM2 Base | 无外部引脚 | 每 20 ms 计算一次速度 |
| 串口 | USART1 | PA9 / PA10 | 输出计数和速度 |

> 重要：定时器和引脚分配有材料依据，但哪一组对应左轮、哪一个 PWM 通道代表实际正转，必须上板低速点动确认，不能仅靠文档猜测。

## 2 实验最终要实现什么

完成后的数据流程如下：

```text
设定电机命令
    ↓
TIM3 输出 PWM
    ↓
电机驱动器控制左右电机
    ↓
TIM8 和 TIM4 统计编码器 AB 相脉冲
    ↓
TIM2 每 20 ms 触发一次中断
    ↓
读取增量计数并换算 rpm
    ↓
主循环通过 USART1 发送给串口助手或 VOFA+
```

建议先完成 PWM、编码器、周期测速三个基础任务。PID 是后续综合任务，基础测速稳定以前不要加入 PID。

## 3 必须先理解的四个量

定时器可以理解为一个按固定速度加一的计数器。

- PSC：预分频器，决定计数速度。
- CNT：当前计数值。
- ARR：自动重装载值，CNT 数到 ARR 后从 0 重新开始。
- CCR：比较值，决定 PWM 高电平持续到哪里。

PWM 频率公式：

```text
fPWM = fTIM / ((PSC + 1) × (ARR + 1))
```

占空比公式：

```text
duty = CCR / (ARR + 1) × 100%
```

本实验 TIM3 定时器时钟为 72 MHz，取：

```text
PSC = 0
ARR = 3599
```

则：

```text
fPWM = 72,000,000 / (1 × 3600) = 20,000 Hz = 20 kHz
```

常用占空比对应的 CCR 如下：

| 占空比 | CCR 计算 | CCR 设置值 |
|---:|---:|---:|
| 0% | 0 × 3600 | 0 |
| 10% | 0.1 × 3600 | 360 |
| 20% | 0.2 × 3600 | 720 |
| 30% | 0.3 × 3600 | 1080 |
| 40% | 0.4 × 3600 | 1440 |

## 4 实验安全准备

首次通电前必须完成以下检查：

1. 把小车可靠垫高，确保两个驱动轮都离开桌面。
2. 保证电机电源、主控板、编码器和下载器共地。
3. 确认电机驱动器的使能或待机信号已经有效。
4. 初始四路 PWM 的 Pulse 全部设置为 0。
5. 首次点动只使用 10%～15% 占空比，持续不超过 1 秒。
6. 每次改变方向前先停止，再等待至少 100 ms。
7. 不允许长时间堵转；电机不转但明显发热或发出沉闷声音时立即断电。
8. 手边准备主板电源开关或可立即拔掉的电机电源。

## 5 建立实验三 CubeMX 工程

建议不要直接覆盖以前的实验工程。

1. 打开 STM32CubeMX。
2. 选择 **File → New Project → MCU Selector**。
3. 搜索并选择 `STM32F103RCT6`。
4. 新工程可命名为 `Experiment3_PWM_Encoder`。
5. 保存到一个新的目录中。

也可以复制旧工程后另存为实验三工程，但生成代码前务必确认保留用户代码选项已经开启。

## 6 配置调试接口和外部时钟

### 6.1 SYS 调试接口

在 **Pinout & Configuration → System Core → SYS** 中设置：

```text
Debug = Serial Wire
```

设置后 PA13 为 SWDIO，PA14 为 SWCLK。不要把这两个引脚分配给普通 GPIO。

### 6.2 RCC 外部晶振

在 **System Core → RCC** 中设置：

```text
High Speed Clock (HSE) = Crystal/Ceramic Resonator
```

### 6.3 Clock Configuration

进入 **Clock Configuration** 页面，配置：

```text
HSE Input Frequency = 8 MHz
PLL Source          = HSE
PLL MUL             = ×9
SYSCLK              = 72 MHz
AHB Prescaler       = /1
APB1 Prescaler      = /2
APB2 Prescaler      = /1
```

最终应看到：

```text
SYSCLK = 72 MHz
HCLK   = 72 MHz
PCLK1  = 36 MHz
APB1 Timer Clock = 72 MHz
PCLK2  = 72 MHz
```

注意：TIM2、TIM3、TIM4 位于 APB1。虽然 PCLK1 显示 36 MHz，但由于 APB1 分频为 2，定时器实际时钟自动乘 2，仍为 72 MHz。

## 7 配置 TIM3 四通道 PWM

### 7.1 启用通道

进入 **Timers → TIM3**，设置：

```text
Clock Source = Internal Clock
Channel 1    = PWM Generation CH1
Channel 2    = PWM Generation CH2
Channel 3    = PWM Generation CH3
Channel 4    = PWM Generation CH4
```

CubeMX 应自动分配：

```text
PA6 = TIM3_CH1
PA7 = TIM3_CH2
PB0 = TIM3_CH3
PB1 = TIM3_CH4
```

如果 CubeMX 分配了其他引脚，先检查是否启用了重映射或存在引脚冲突。

### 7.2 Parameter Settings

设置 TIM3 基本参数：

```text
Prescaler                 = 0
Counter Mode              = Up
Counter Period            = 3599
Clock Division            = DIV1
Auto-reload preload       = Enable
```

四个通道均设置：

```text
PWM Mode                  = PWM mode 1
Pulse                     = 0
Output Compare Polarity   = High
Output Compare Fast Mode  = Disable
```

Pulse 必须从 0 开始，防止程序刚启动时电机突然转动。

## 8 配置 TIM4 编码器

进入 **Timers → TIM4**：

```text
Combined Channels = Encoder Mode
Encoder Mode      = TI12
```

确认引脚：

```text
PB6 = TIM4_CH1
PB7 = TIM4_CH2
```

参数建议：

```text
Prescaler          = 0
Counter Period     = 65535
Encoder Mode       = TI12
IC1 Polarity       = Rising Edge
IC1 Selection      = Direct TI
IC1 Prescaler      = DIV1
IC1 Filter         = 10
IC2 Polarity       = Rising Edge
IC2 Selection      = Direct TI
IC2 Prescaler      = DIV1
IC2 Filter         = 10
```

滤波值 10 来自已经使用过的旧 `.ioc`。如果高速时明显漏计数，可逐步降低滤波值；如果静止时计数乱跳，可检查接地和接线后再适当增加滤波。

## 9 配置 TIM8 编码器

进入 **Timers → TIM8**，使用与 TIM4 相同的编码器参数：

```text
Combined Channels = Encoder Mode
Encoder Mode      = TI12
Prescaler         = 0
Counter Period    = 65535
IC1/IC2 Polarity  = Rising Edge
IC1/IC2 Selection = Direct TI
IC1/IC2 Prescaler = DIV1
IC1/IC2 Filter    = 10
```

确认引脚：

```text
PC6 = TIM8_CH1
PC7 = TIM8_CH2
```

TIM8 是高级定时器，但在这里仅使用它的编码器接口，不需要配置 PWM 输出。

## 10 配置 TIM2 为 20 ms 周期定时器

进入 **Timers → TIM2**，设置：

```text
Clock Source          = Internal Clock
Prescaler             = 7199
Counter Mode          = Up
Counter Period        = 199
Clock Division        = DIV1
Auto-reload preload   = Disable
```

计算过程：

```text
计数频率 = 72 MHz / (7199 + 1) = 10 kHz
更新频率 = 10 kHz / (199 + 1) = 50 Hz
更新时间 = 1 / 50 Hz = 20 ms
```

打开 TIM2 的 **NVIC Settings**：

```text
TIM2 global interrupt = Enabled
Preemption Priority   = 1
Sub Priority          = 0
```

如果课程统一规定其他中断优先级，以教师要求为准。

## 11 配置 USART1

进入 **Connectivity → USART1**：

```text
Mode        = Asynchronous
Baud Rate   = 115200 Bits/s
Word Length = 8 Bits
Parity      = None
Stop Bits   = 1
Flow Control= None
```

确认引脚：

```text
PA9  = USART1_TX
PA10 = USART1_RX
```

## 12 生成工程

进入 **Project Manager**：

1. 设置工程名和保存目录。
2. Toolchain 选择实际使用的环境。使用 STM32CubeIDE 就选 STM32CubeIDE；使用 EIDE 导入时可按课程要求选择 Makefile。
3. 勾选或确认 **Keep User Code when re-generating**。
4. 点击 **Generate Code**。
5. 编译一次，确保尚未加入用户代码时工程可以通过编译。

如果出现引脚冲突，先回 CubeMX 解决，不要直接修改自动生成的初始化代码。

## 13 添加用户代码

以下代码按 STM32 HAL 编写。所有自定义内容都放在 `USER CODE BEGIN` 与 `USER CODE END` 之间，防止重新生成代码时被覆盖。

### 13.1 头文件

在 `main.c` 的 `USER CODE BEGIN Includes` 中加入：

```c
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
```

### 13.2 宏和全局变量

在 `USER CODE BEGIN PV` 中加入：

```c
#define PWM_ARR_VALUE       3599U
#define PWM_PERIOD_COUNTS   3600U

#define SAMPLE_PERIOD_MS    20L
#define COUNTS_PER_REV      1560L   /* 后续必须用实测值替换 */

volatile int16_t encoder_left_delta  = 0;
volatile int16_t encoder_right_delta = 0;

/* 单位是 0.1 rpm，例如 1234 表示 123.4 rpm */
volatile int32_t speed_left_rpm_x10  = 0;
volatile int32_t speed_right_rpm_x10 = 0;

volatile uint8_t speed_data_ready = 0;

/* 手动测试后按实际方向改成 1 或 -1 */
static int8_t encoder_left_sign  = 1;
static int8_t encoder_right_sign = 1;
```

这里暂定 TIM8 为左轮、TIM4 为右轮。若实测相反，只调整名称或映射，不需要重做 CubeMX 配置。

### 13.3 函数声明

在 `USER CODE BEGIN PFP` 中加入：

```c
static uint32_t CommandToCompare(int16_t command);
static void Motor_A_Set(int16_t command);
static void Motor_B_Set(int16_t command);
static void Motor_StopAll(void);
static int16_t Encoder_ReadAndClear(TIM_HandleTypeDef *htim);
static int32_t EncoderDeltaToRpmX10(int16_t delta);
```

### 13.4 PWM 与编码器函数

在 `USER CODE BEGIN 0` 中加入：

```c
/* command 使用 -1000 到 1000，绝对值代表 0.0% 到 100.0% */
static uint32_t CommandToCompare(int16_t command)
{
    int32_t magnitude = command;

    if (magnitude < 0)
        magnitude = -magnitude;

    if (magnitude > 1000)
        magnitude = 1000;

    uint32_t compare =
        (uint32_t)((magnitude * (int32_t)PWM_PERIOD_COUNTS) / 1000);

    if (compare > PWM_ARR_VALUE)
        compare = PWM_ARR_VALUE;

    return compare;
}

/*
 * 这是依据报告骨架写出的初始映射模板：
 * 正命令：CH1 输出 PWM，CH2 为 0
 * 负命令：CH1 为 0，CH2 输出 PWM
 * 实际正反方向必须架空车轮后验证。
 */
static void Motor_A_Set(int16_t command)
{
    uint32_t compare = CommandToCompare(command);

    if (command > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
    else if (command < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, compare);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
}

static void Motor_B_Set(int16_t command)
{
    uint32_t compare = CommandToCompare(command);

    if (command > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, compare);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
    else if (command < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, compare);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
}

static void Motor_StopAll(void)
{
    Motor_A_Set(0);
    Motor_B_Set(0);
}

static int16_t Encoder_ReadAndClear(TIM_HandleTypeDef *htim)
{
    int16_t delta = (int16_t)__HAL_TIM_GET_COUNTER(htim);
    __HAL_TIM_SET_COUNTER(htim, 0);
    return delta;
}

static int32_t EncoderDeltaToRpmX10(int16_t delta)
{
    /*
     * rpm × 10 = delta × 60 × 10 / (CPR × Ts秒)
     * Ts 使用毫秒后，系数变为 600000。
     * 使用 int64_t 防止乘法溢出。
     */
    return (int32_t)(((int64_t)delta * 600000LL) /
                     (COUNTS_PER_REV * SAMPLE_PERIOD_MS));
}
```

如果板卡的停止电平不是“两路 CCR 均为 0”，必须按实际电机驱动电路修改 `Motor_A_Set()` 和 `Motor_B_Set()`。首次测试时不要直接使用负命令反转。

### 13.5 启动外设

在 `main()` 中，所有 `MX_..._Init()` 调用结束之后的 `USER CODE BEGIN 2` 中加入：

```c
/* 先强制写入安全停止值 */
Motor_StopAll();

/* 启动 TIM3 四路 PWM */
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

/* 启动两个编码器 */
HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

__HAL_TIM_SET_COUNTER(&htim8, 0);
__HAL_TIM_SET_COUNTER(&htim4, 0);

/* 最后启动 20 ms 周期中断 */
HAL_TIM_Base_Start_IT(&htim2);
```

### 13.6 TIM2 周期中断回调

在 `USER CODE BEGIN 4` 中加入：

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        int16_t left_raw  = Encoder_ReadAndClear(&htim8);
        int16_t right_raw = Encoder_ReadAndClear(&htim4);

        encoder_left_delta =
            (int16_t)(encoder_left_sign * left_raw);
        encoder_right_delta =
            (int16_t)(encoder_right_sign * right_raw);

        speed_left_rpm_x10 =
            EncoderDeltaToRpmX10(encoder_left_delta);
        speed_right_rpm_x10 =
            EncoderDeltaToRpmX10(encoder_right_delta);

        speed_data_ready = 1;
    }
}
```

不要在这个回调里执行阻塞式 `printf()`。回调只负责读取、计算和置标志。

### 13.7 GCC 的 printf 串口重定向

仍在 `USER CODE BEGIN 4` 中加入：

```c
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len,
                      HAL_MAX_DELAY);
    return len;
}
```

如果工程已经有 `_write()`，不要重复定义，应继续使用原有重定向代码。

### 13.8 主循环输出

在 `while (1)` 的 `USER CODE BEGIN 3` 中加入：

```c
if (speed_data_ready)
{
    int32_t left_copy;
    int32_t right_copy;

    __disable_irq();
    left_copy = speed_left_rpm_x10;
    right_copy = speed_right_rpm_x10;
    speed_data_ready = 0;
    __enable_irq();

    /* 输出单位为 0.1 rpm，VOFA+ 中的数值除以 10 即实际 rpm */
    printf("%ld,%ld\r\n", (long)left_copy, (long)right_copy);
}
```

这里不用浮点 `printf`，可以避免未开启浮点格式化支持时没有输出的问题。

## 14 分阶段上板测试

### 阶段 A：只验证 PWM

1. 暂时不要连接或启动 TIM2 中断和编码器逻辑也可以。
2. 编译并下载程序。
3. 确认上电后两轮静止。
4. 如有示波器或逻辑分析仪，先测 PA6、PA7、PB0、PB1：
   - 停止时应为安全电平。
   - 设置 10% 时应看到约 20 kHz、10% 占空比波形。
5. 只测试电机 A：

```c
Motor_StopAll();
HAL_Delay(500);
Motor_A_Set(100);   /* 10% */
HAL_Delay(800);
Motor_A_Set(0);
```

6. 记录 Motor A 实际转向。
7. 再单独测试 Motor B。
8. 确认停止至少 100 ms 后，才可短时测试负命令。

命令换算：

```text
100  = 10%
200  = 20%
300  = 30%
400  = 40%
-100 = 反方向 10%
```

如果 10% 时电机不转，不一定是故障，可能尚未超过静摩擦形成的起转阈值。可短时升到 15% 或 20%，但先确认没有接线和使能问题。

### 阶段 B：手动验证编码器方向

1. 保持 PWM 为 0，电机不要通电转动。
2. 启动 TIM4、TIM8 编码器。
3. 暂时用调试器观察 `TIM4->CNT` 和 `TIM8->CNT`，或周期打印增量。
4. 手动向小车前进方向转动左轮：
   - 判断变化的是 TIM8 还是 TIM4。
   - 记录计数正负。
5. 对右轮重复测试。
6. 如果向前时计数为负，把对应的 `encoder_*_sign` 改成 `-1`。
7. 最终要求：小车向前时左右速度符号都为正。

### 阶段 C：测量每圈计数

1. 在轮胎和固定车架上分别做一个对齐标记。
2. 清零对应编码器计数器。
3. 沿固定方向缓慢、完整地转动一圈。
4. 记录计数绝对值。
5. 重复至少三次，取平均值。
6. 两个轮子分别测量。

理论参考值：

```text
13 编码器线数 × 30 减速比 × 4 倍频 = 1560 count/rev
```

但程序中的 `COUNTS_PER_REV` 最终应使用实测结果。如果左右轮结果差异很大，先排查漏计数、AB 相接线和输入滤波，不要直接取平均掩盖问题。

### 阶段 D：验证 20 ms 周期

1. 启动 TIM2 中断。
2. 电机静止时，观察左右 `encoder_delta` 应接近 0。
3. 手转车轮，观察每 20 ms 都有新的增量和转速数据。
4. 可在 TIM2 回调中短时翻转一个空闲 GPIO，用示波器验证周期；不要用中断内 `printf` 的时间间隔代替精确测量。

### 阶段 E：PWM 与测速联合测试

车轮保持架空，按以下顺序测试：

| PWM 命令 | 占空比 | 建议运行时间 | 应记录内容 |
|---:|---:|---:|---|
| 0 | 0% | 2 s | 是否完全停止、静止漂移 |
| 100 | 10% | 2 s | 是否起转、左右 rpm |
| 200 | 20% | 3 s | 左右 rpm、波动范围 |
| 300 | 30% | 3 s | 左右 rpm、波动范围 |
| 400 | 40% | 3 s | 左右 rpm、声音、发热 |

每个档位之间先停止约 1 秒。不要直接从正转切换到反转。

## 15 VOFA+ 基本使用

1. 用串口工具确认数据格式类似：

```text
1023,987
1031,995
1028,991
```

2. 在 VOFA+ 中选择正确串口。
3. 波特率设置为 115200。
4. 协议选择可接收逗号分隔文本数据的 FireWater。
5. 两列数据分别作为左右轮曲线。
6. 当前程序输出单位是 0.1 rpm，曲线值 1000 表示 100.0 rpm。

VOFA+ 适合看转速变化趋势，不适合直接测量真正的 20 kHz PWM 波形。PWM 频率和占空比应由示波器或逻辑分析仪验证。

## 16 结果是否正确的判断标准

### PWM 部分

- 四个 PWM 通道能够启动。
- 实测频率接近 20 kHz。
- 设置 10%、20%、30%、40% 时实测占空比相符。
- 停止状态安全且不会突然满占空比输出。
- 左右轮实际方向映射已经记录。

### 编码器部分

- 手动正转、反转时计数符号相反。
- 静止 10 秒时计数不会大量漂移。
- 完整转动一圈的计数重复性较好。
- 左右轮向前时，经软件修正后的计数符号相同。

### 周期测速部分

- TIM2 采样周期为约 20 ms。
- 电机恒速时速度曲线总体稳定。
- 增加占空比后平均转速总体上升。
- 同占空比左右轮速度允许存在一定差异，但差异应能解释和记录。

## 17 常见故障排查

| 现象 | 优先检查 | 处理方法 |
|---|---|---|
| 程序下载失败 | SYS Debug、SWD 接线、供电 | SYS 选 Serial Wire，检查 PA13/PA14/GND |
| 上电立即高速转动 | Pulse 初值、CCR、停止逻辑 | 四路 Pulse 改 0，上电先调用 Motor_StopAll |
| PWM 引脚没有波形 | TIM3 是否启动、引脚映射 | 检查四次 HAL_TIM_PWM_Start 返回值 |
| PWM 频率不是 20 kHz | 时钟、PSC、ARR | 确认 TIM3 时钟是 72 MHz、PSC=0、ARR=3599 |
| 电机完全不转 | 驱动器使能、供电、占空比 | 检查 STBY/EN、共地，短时提高到 20% |
| 电机方向相反 | 通道对应关系 | 记录实际映射后交换通道逻辑或命令符号 |
| 编码器一直为 0 | AB 接线、供电、引脚复用 | 检查 PB6/PB7、PC6/PC7 和共地 |
| 静止时计数乱跳 | 接地、线缆、输入滤波 | 先修复接线，再调整 IC Filter |
| 高速时计数偏低 | 滤波过大、波形质量差 | 降低 Filter，并用示波器看 AB 相 |
| 向前时一正一负 | 左右编码器方向不同 | 对相应 `encoder_*_sign` 设为 -1 |
| rpm 明显错误 | CPR 或采样周期错误 | 实测 count/rev，确认 TIM2 真的是 20 ms |
| 串口无输出 | 波特率、TX/RX、printf 重定向 | 检查 PA9/PA10、115200 和 `_write()` |
| 加 printf 后测速异常 | 在中断里阻塞输出 | 中断只置标志，主循环负责发送 |

## 18 实验报告需要保存的证据

实验过程中及时截图或录像，不要完成后再补：

1. CubeMX 芯片型号和 72 MHz 时钟树。
2. TIM3 CH1～CH4、PSC、ARR、Pulse 和引脚配置。
3. TIM4 与 TIM8 的 Encoder Mode、引脚和滤波配置。
4. TIM2 的 PSC、ARR 和 NVIC 配置。
5. 工程编译成功、下载成功或断点调试画面。
6. PWM 波形的频率和占空比测量。
7. 手动正反转时编码器计数符号。
8. 每个轮子转一整圈的计数值。
9. 不同占空比下左右轮 rpm 数据。
10. VOFA+ 左右轮速度曲线。
11. 遇到的问题、判断依据、修改方法和修改后的结果。

所有表格应填写实测值。不要把 1560 count/rev、20 kHz 或方向映射直接当成已经测得的结果。

## 19 推荐的实际执行顺序

严格按以下顺序推进，每一步成功后再进行下一步：

```text
确认型号和时钟
→ 配置并生成工程
→ 编译空工程
→ 验证安全停止
→ 验证单路 PWM
→ 验证两路电机方向
→ 手动验证两个编码器
→ 实测每圈计数
→ 开启 20 ms 中断
→ 计算并输出 rpm
→ 联合测试多个占空比
→ 使用 VOFA+ 画曲线
→ 整理报告和视频
→ 基础稳定后再考虑 PID
```

## 20 开始实验前的最终检查表

- [ ] 实物仍是同一块 C10B、STM32F103RCT6 板卡。
- [ ] 新工程没有覆盖以前的实验工程。
- [ ] SYS Debug 已选择 Serial Wire。
- [ ] SYSCLK 为 72 MHz，APB1 Timer Clock 为 72 MHz。
- [ ] TIM3 为四通道 20 kHz PWM，四路 Pulse 初值为 0。
- [ ] TIM4 为 PB6/PB7 编码器。
- [ ] TIM8 为 PC6/PC7 编码器。
- [ ] TIM2 为 20 ms 更新中断并已打开 NVIC。
- [ ] USART1 为 PA9/PA10、115200 8N1。
- [ ] 两侧车轮已经可靠架空。
- [ ] 首次测试命令不超过 10%～15%。
- [ ] 有随时切断电机电源的方法。

