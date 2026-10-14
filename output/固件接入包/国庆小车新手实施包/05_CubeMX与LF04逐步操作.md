# C10B：CubeMX配置与LF04逐步操作

> 2026-10-11最新：OLED方向已由用户确认正常，LF04已接线、黑白标定待测。已编译[LF04静态读数固件与操作说明](/Users/mzh/Desktop/车车/output/固件接入包/T08_LF04静态标定接入包/LF04静态标定操作说明.md)，保持OFF用OLED末行记录O1～O4。

> 2026-10-11最新：OLED默认四线SPI已按SSD1306试亮方案接入并编译，保留按键启动；静态亮屏尚待确认。使用[OLED试亮说明](/Users/mzh/Desktop/车车/output/固件接入包/T11_OLED试亮接入包/OLED试亮操作说明.md)与对应HEX。

> 2026-10-11最新工程状态：场地GO100已记录尺测102/101/102cm及正常停车声光；现已接入PA0独立校准/启动并完整编译，按键实物验证待烧录后进行。使用[最新按键启动操作说明](/Users/mzh/Desktop/车车/output/固件接入包/T10_按键独立启动接入包/按键启动操作说明.md)与对应HEX；OLED暂未接入。以下通用实验步骤与历史进度保留，不能代替最新实测记录。

更新日期：2026年10月7日。本版补充：代码、放置位置、原因、预期输出和逐项验收。适用于现有STM32F103RCT6 C10B学习联调，采用独立的新HAL工程；如果你已有老师/厂商工程，先另存副本，再按对应项目对照，不能同时启用两套控制任务。本指南已经核对本地厂商.ioc与LF04手册，尚未替你生成/烧录完整固件。

## 1 今天先做到哪一步

先做到“串口显示四路读数，移动黑线时逐路变化”，再加入电机和巡线控制。按顺序验收：工程能编译烧录 → 串口能打印 → LF04四路黑白可靠 → 编码器/电机方向可靠 → 速度闭环 → 圆弧纠偏 → 完整H题。

静态读数阶段保持电机输出停止、驱动轮架空。改接线先断开电池；使能开关OFF不能未经验证就当成物理断开电机电源。不启动雷达、蓝牙、PS2或完整厂商自动行驶框架。

### 1.1 本手册完成后能做到什么

第2—8节是一套连贯的静态调试实验：工程运行、串口有字、板上LED和蜂鸣器可控、四路LF04黑白可靠。第9节逐项验证PWM、编码器、10ms节拍和ADC；第10节先在电机不动时验证巡线转向请求。要真正运动，还需要把测速、速度PI、转向请求、使能与停车逻辑接成一个控制任务。完整H题的航向、路段切换、定距停车和四圈状态机继续看02手册；本篇不宣称已交付完整路线固件。

**使用顺序：一次只新增一个实验，编译、烧录、验收后再继续。不要把后面的所有代码一次粘完。** 各节明确写“新增”或“替换”，按这两个词操作。

### 1.2 先认识代码到底放在哪里

CubeMX中建议勾选 Project Manager → Code Generator → **Generate peripheral initialization as a pair of .c/.h files per peripheral**。这样GPIO、UART、TIM、ADC分别在gpio.c、usart.c、tim.c、adc.c中，本文代码位置最容易对应。不同版本目录可能是Core/Src与Core/Inc，也可能是Src与Inc；认文件名和下面的标记，不靠行号。

| 搜索的标记/文件 | 里面放什么 | 为什么 |
| --- | --- | --- |
| main.c：USER CODE BEGIN Includes | include头文件 | 告诉编译器函数与句柄的声明从哪里来 |
| main.c：USER CODE BEGIN PD | 宏、实验开关 | 集中放需要自己改的选项 |
| main.c：USER CODE BEGIN PV | 跨调用保存的变量 | 例如上次打印时间、编码器前一次计数 |
| main.c：USER CODE BEGIN 0 | 本文的辅助函数 | 在main前定义，main可以直接调用；按本文顺序放，不需要另加函数声明 |
| main.c：USER CODE BEGIN 2 | 外设初始化后的启动/一次性检查 | GPIO、UART等已经初始化，句柄才可使用 |
| main.c：USER CODE BEGIN 3 | while(1)内的轮询代码 | 会反复执行，适合静态读数和低频日志 |
| main.c：USER CODE BEGIN 4 | HAL回调函数 | TIM中断经HAL分发后调用；函数定义不能塞进main里面 |
| gpio.c、usart.c、tim.c、adc.c | CubeMX生成的配置 | 优先回.ioc改配置再Generate Code，不手改生成区当成长期修复 |

只复制代码块**内部**，不复制Markdown的三个反引号。不删除原有USER CODE标记，不把第二个main函数贴进去，不重复定义huart1、htim3等生成句柄。再生成前保存源码并留副本；Keep User Code只能保护指定用户区，不替你保护任意位置。

如果没有usart.h/tim.h/adc.h，而句柄就在main.c里，保留生成的句柄声明，省略不存在的头文件。本文USER CODE BEGIN 0在生成变量区之后，可以直接使用这些句柄。USART1在本文始终指huart1，不改成huart3。

### 1.3 先建立版本与实测记录

每次记录：工程路径、.ioc、编译输出文件、烧录时间、实物现象、串口日志。CubeMX只是生成初始化代码；编译器把代码变成固件；下载工具把固件写入主板。三件事分别验证，不能用“生成成功”代替“板子正在跑新程序”。

本文串口日志用英文和整数，避免终端中文编码及浮点printf配置成为第一阶段的额外问题。标为“预期输出”的数字是示例，不是你的小车实测成绩。

## 2 新建CubeMX工程

1. 打开CubeMX，选 ACCESS TO MCU SELECTOR / New Project。
2. 搜索STM32F103RCT6，选择LQFP64封装。核对板上主芯片，不能选择F407或F103C8。
3. Project Manager填写独立Project Name，例如c10b_line_lab，保存在新的目录。
4. Toolchain/IDE按实际工具选择。Mac已有ARM GCC可选Makefile；已有CubeIDE则选择相应工程。CubeMX只生成代码，不自动提供完整编译器与烧录器。
5. Code Generator勾选 Keep User Code when re-generating；自写代码放USER CODE区或独立文件。保存.ioc。

通过标准：项目目标确为STM32F103RCT6，保存位置不覆盖现有firmware。

**怎样核对：** .ioc的MCU型号应显示STM32F103RCT6；GCC工程通常使用STM32F103xE编译宏与对应启动文件（F103RCT6属于这一分组）。打开生成main.c，能找到HAL_Init、SystemClock_Config、MX_GPIO_Init等调用。此节不额外添加C代码：芯片、目录和工具链是工程配置，写一段GPIO代码无法修正选错芯片的问题。

## 3 配调试与72MHz时钟

Pinout & Configuration → System Core → SYS → Debug选Serial Wire。保留PA13/PA14 SWD，释放JTAG占用的PA15/PB3/PB4；不要选择Full JTAG。

System Core → RCC → High Speed Clock (HSE)选Crystal/Ceramic Resonator。Clock Configuration中按板上8MHz晶振设置：

| 项目 | 设置/核对值 |
| --- | --- |
| HSE Input | 8 MHz |
| PLL Source | HSE |
| PLL multiplier | ×9 |
| System Clock Mux | PLLCLK |
| AHB prescaler | /1，HCLK=72 MHz |
| APB1 prescaler | /2，PCLK1=36 MHz |
| APB2 prescaler | /1，PCLK2=72 MHz |
| APB1 timer clocks | 72 MHz |
| APB2 timer clocks | 72 MHz |

不要选择HSE Bypass代替板上的晶振。APB1分频为2时定时器得到2×PCLK1，所以TIM2/TIM3/TIM4仍是72MHz。此版本不启用USB外设；USART1通过板载USB转串口芯片通信，与MCU的USB外设不同。

通过标准：时钟树无红色冲突，SYSCLK/HCLK为72MHz，定时器时钟与上述值一致。

**生成后看哪里：** main.c的SystemClock_Config应有HSE_ON、PLL_SOURCE_HSE、PLL_MUL9、AHB_DIV1、APB1_DIV2、APB2_DIV1及FLASH_LATENCY_2对应配置。名称以生成HAL版本为准，不手抄一套SystemClock_Config覆盖生成函数。第4.3节的Lab_PrintBoot会打印运行时钟供你复核。HAL读取时钟寄存器并按工程HSE_VALUE计算频率，所以仍要核对板上晶振和HSE_VALUE=8000000，不能把打印值当成示波器测量。

## 4 GPIO、串口与四路巡线输入

在芯片图逐个点击引脚选择功能；在System Core → GPIO表内修改Pull、Output Level及User Label。第一版只需要基础GPIO、USART1和LF04输入，其他外设可逐步添加。

| 引脚 | CubeMX功能 | 细项 | 用途 |
| --- | --- | --- | --- |
| PA0 | GPIO_Input | No pull，与板上按键电路配合 | USER按键，按下高 |
| PC13 | GPIO_Input | No pull | 使能开关，ON/OFF实测记录 |
| PC2、PC3 | GPIO_Output | Push-pull、Low speed、初始High | LED，低电平亮 |
| PA15 | GPIO_Output | Push-pull、Low speed、初始Low | 蜂鸣器，高电平响 |
| PC8 | GPIO_Input | Pull-up；Label=LINE_O1 | LF04 O1 |
| PC4 | GPIO_Input | Pull-up；Label=LINE_O2 | LF04 O2 |
| PC9 | GPIO_Input | Pull-up；Label=LINE_O3 | LF04 O3 |
| PB8 | GPIO_Input | Pull-up；Label=LINE_O4 | LF04 O4 |
| PA6、PA7、PB0、PB1 | 第一阶段GPIO_Output | Push-pull、Low speed、初始High | AT8236四个输入置高，静态阶段保持制动命令；第9.1节再改成TIM3 PWM |
| PA9 | USART1_TX | 由USART1异步模式分配 | 日志输出 |
| PA10 | USART1_RX | 由USART1异步模式分配 | 串口接收，首版不解析控制命令 |

LF04是数字开关量，选择GPIO_Input，不是ADC，也不是GPIO_Output；首版不需要EXTI。四路Pull-up用于输入默认电平，不能替代正确供电与接线，也不能将5V信号强拉成安全的3.3V信号。

Connectivity → USART1 → Mode=Asynchronous；Parameter Settings设115200、8 Bits、None parity、1 Stop Bit、None hardware flow control。首版采用轮询发送，不需要UART DMA/接收中断。

如果改厂商雷达HAL工程：.ioc中PC4/PC8/PC9原为GPIO_Output，PB8已是Pull-up输入。四路全部改输入，并停用PS2初始化与调用。检查生成gpio.c中四路不再出现在输出初始化组；另外的PS2代码不能再次将它们改回输出。

Buzzer（蜂鸣器，PA15）：下面给出设置、测试代码与验收。

### 4.1 先核对CubeMX生成的GPIO代码

Generate Code后，打开gpio.c的MX_GPIO_Init。下面是**核对用的关键片段，不整段替换生成文件**；标签名可能使生成代码使用LINE_O1_Pin等宏，等价即可。

```c
/* 先设输出电平，再改成输出模式，减少切换时的错误电平。 */
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);

/* GPIO_InitStruct是MX_GPIO_Init里的局部变量。 */
GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_8 | GPIO_PIN_9;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

GPIO_InitStruct.Pin = GPIO_PIN_8;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
```

PC2/PC3/PA15及四个电机输入还应各有GPIO_MODE_OUTPUT_PP初始化；上面的WritePin只设置输出锁存值，**不会自己把引脚变成输出**。PC4/PC8/PC9/PB8不应出现在任何输出初始化组。检查stm32f1xx_hal_msp.c的HAL_MspInit有`__HAL_AFIO_REMAP_SWJ_NOJTAG();`，保留SWD并释放PA15。

**为什么：** LF04要往MCU输入信号；如果MCU也把这些脚设为输出，两边可能互相驱动。PC2/PC3写High关灯，PA15写Low关蜂鸣器。本文新增四个电机输入的静态High，是该板AT8236的制动命令；MCU复位到初始化之前仍可能有过渡电平，不能据此省略电机电源隔离与架空。

若是厂商工程，全文搜索`PS2_SetInit`、`PS2_Read`、`HAL_GPIO_Init`，以及PC4/PC8/PC9/PB8：停用后续重配巡线脚的代码。厂商TIM3初始化里可能直接启动PWM，静态实验也要停用它和原控制中断。最容易照做的方式仍是本文的独立新工程。

### 4.2 加入头文件和实验开关

**位置：main.c → USER CODE BEGIN Includes。新增：**

<!-- CODE: base_includes -->
```c
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"  /* 外设分文件时需要；已有则不重复加。 */
```

stdio.h提供snprintf，string.h提供strlen，usart.h声明huart1。这里只用snprintf把数据写入字符数组，随后明确发送到USART1，**不需要先做printf重定向或重写fputc**。

**位置：main.c → USER CODE BEGIN PD。新增：**

<!-- CODE: base_defines -->
```c
#define LAB_INDICATOR_SELFTEST 0  /* 初次保持0；想单独测试灯/蜂鸣器时改为1。 */
```

### 4.3 加入静态调试辅助函数

**位置：main.c → USER CODE BEGIN 0。按顺序新增整段：**

<!-- CODE: base_functions -->
```c
static void Lab_Write(const char *text)
{
    size_t n = strlen(text);
    if (n > 0U && n <= 65535U) {
        /* 只在主循环/初始化中使用；本实验不在中断里打印。 */
        if (HAL_UART_Transmit(&huart1, (uint8_t *)text,
                              (uint16_t)n, 50U) != HAL_OK) {
            /* UART自己失败时无法靠同一个UART报错，点亮PC2提示。 */
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
        }
    }
}

static void Lab_Quiet(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}

/* STM32F1每个GPIO的配置占CRL/CRH中的4位。pin_no是编号，不是位掩码。 */
static unsigned Lab_PinConfig(GPIO_TypeDef *port, unsigned pin_no)
{
    uint32_t cr = (pin_no < 8U) ? port->CRL : port->CRH;
    return (unsigned)((cr >> ((pin_no & 7U) * 4U)) & 0xFU);
}

static int Lab_LinePinOK(GPIO_TypeDef *port, unsigned pin_no)
{
    /* F1输入上下拉模式配置为8；ODR对应位为1表示上拉。 */
    return Lab_PinConfig(port, pin_no) == 8U &&
           (port->ODR & (1UL << pin_no)) != 0U;
}

static int Lab_LinePinsOK(void)
{
    return Lab_LinePinOK(GPIOC, 8U) && Lab_LinePinOK(GPIOC, 4U) &&
           Lab_LinePinOK(GPIOC, 9U) && Lab_LinePinOK(GPIOB, 8U);
}

static void Lab_PrintBoot(void)
{
    char msg[192];
    Lab_Write("\r\nC10B_LINE_LAB boot; motor test disabled\r\n");
    int n = snprintf(msg, sizeof msg,
        "SYS=%lu HCLK=%lu PCLK1=%lu PCLK2=%lu Hz\r\n",
        (unsigned long)HAL_RCC_GetSysClockFreq(),
        (unsigned long)HAL_RCC_GetHCLKFreq(),
        (unsigned long)HAL_RCC_GetPCLK1Freq(),
        (unsigned long)HAL_RCC_GetPCLK2Freq());
    if (n > 0 && n < (int)sizeof msg) Lab_Write(msg);

    n = snprintf(msg, sizeof msg,
        "LINE cfg: PC8=%X PC4=%X PC9=%X PB8=%X; pullup=%s\r\n",
        Lab_PinConfig(GPIOC, 8U), Lab_PinConfig(GPIOC, 4U),
        Lab_PinConfig(GPIOC, 9U), Lab_PinConfig(GPIOB, 8U),
        Lab_LinePinsOK() ? "PASS" : "FAIL");
    if (n > 0 && n < (int)sizeof msg) Lab_Write(msg);
    Lab_Write("Boot report done; check physical LEDs/beeper too.\r\n");
}

static void Lab_IndicatorSelfTest(void)
{
    if (LAB_INDICATOR_SELFTEST == 0) return;
    Lab_Write("Indicator test: PC2, PC3, then beeper\r\n");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_Delay(200U);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_Delay(200U);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_Delay(100U);
    Lab_Quiet();
    Lab_Write("Indicator test done; all off\r\n");
}
```

| 函数 | 做了什么 | 为什么要有它 |
| --- | --- | --- |
| Lab_Write | 将文字明确发送到huart1；失败时亮PC2 | 让“串口有字”有一条真实发送路径；排除printf被重定向到别的串口 |
| Lab_Quiet | 两灯灭、蜂鸣器关 | 开机状态有明确命令；它只管指示器，不替代电机停止逻辑 |
| Lab_PinConfig/LinePinsOK | 读取当前引脚配置及上拉状态 | 检查烧进板子的程序运行后是否仍把四路当输入；后续日志还会重复检查 |
| Lab_PrintBoot | 打印工程标识、时钟和四路配置 | 复位后能辨认新固件，并把时钟/GPIO验收变成可观察结果 |
| Lab_IndicatorSelfTest | 可选：两灯依次亮200ms，蜂鸣器响100ms，然后全关 | 区分“程序请求关灯”与“接线/引脚极性真实正确” |

此自检的HAL_Delay只用于**启动时、尚未运行控制任务**。以后做运动控制，不把它放进中断或控制任务。GPIO配置自检是F103专用，不能原样搬到F407。PASS说明读到的四路寄存器符合输入上拉配置，不代表供电、接线、高低电平或黑白识别已经正确。

### 4.4 在初始化后调用这些函数

**位置：main.c → USER CODE BEGIN 2，必须在MX_GPIO_Init和MX_USART1_UART_Init之后。新增：**

<!-- CODE: base_init -->
```c
Lab_Quiet();
Lab_PrintBoot();
Lab_IndicatorSelfTest();
```

这里只粘三行调用；MX_GPIO_Init与MX_USART1_UART_Init由CubeMX生成，不在用户区再重复调用。把Lab_Write放到UART初始化前，句柄尚未准备好；把Lab_Quiet放到GPIO初始化前，引脚模式还没生效。

### 4.5 加一个持续输出，避免错过开机文字

**位置：main.c → USER CODE BEGIN PV。新增：**

<!-- CODE: base_variables -->
```c
static uint32_t line_log_ms = 0U;
```

**位置：main.c → while(1)内的USER CODE BEGIN 3。新增：**

<!-- CODE: heartbeat_loop -->
```c
if ((uint32_t)(HAL_GetTick() - line_log_ms) >= 1000U) {
    line_log_ms = HAL_GetTick();
    Lab_Write("alive; USART1 115200 8N1\r\n");
}
```

打开串口后再按RESET，可看开机报告；即使错过报告，也应每秒看见一条alive。这证明主循环在持续运行。时间相减用uint32_t，HAL毫秒计数回绕后仍能比较短时间间隔。

### 4.6 逐条验收，不能只写“看起来正常”

先编译，再烧录，串口选择板载CH9102端口，设115200/8N1、无流控。复位后预期输出：

```text
C10B_LINE_LAB boot; motor test disabled
SYS=72000000 HCLK=72000000 PCLK1=36000000 PCLK2=72000000 Hz
LINE cfg: PC8=8 PC4=8 PC9=8 PB8=8; pullup=PASS
Boot report done; check physical LEDs/beeper too.
alive; USART1 115200 8N1
```

- **串口能看到文字：** 新固件标识和持续alive都能看到；乱码先查波特率、实际时钟、地线与端口。
- **四路没有输出/外设复用冲突：** .ioc与生成GPIO代码符合4.1节；运行日志pullup=PASS；原PS2初始化和轮询已停用；第6节运行中也持续PASS。此处未要求黑白已经识别正确。
- **LED/蜂鸣器初始安静：** SELFTEST=0时，两只用户LED灭、蜂鸣器无声；板上的电源灯、LF04自身指示灯不算这两只用户LED。若PC2亮且串口没字，可能是Lab_Write发送失败的提示。
- **证明指示器能被控制：** SELFTEST改成1，重新编译烧录；PC2灯亮→PC3灯亮→蜂鸣器短响→全部关闭。验收后恢复0，再编译烧录。
- **没有自动运动：** 四个电机输入仍处于第一阶段静态GPIO High；本阶段不调用PWM启动或厂商自动任务。

只看到模块LED，不等于串口验收通过；只看到寄存器PASS，也不等于线路和传感器验收通过。

## 5 LF04接线、固定与供电

改线先断电。LF04有VCC、GND、O1、O2、O3、O4共六根线。

| LF04信号 | 本地C10B例程对应引脚 |
| --- | --- |
| O1 | PC8 |
| O2 | PC4 |
| O3 | PC9 |
| O4 | PB8 |
| GND | C10B GND，共地 |
| VCC | 按模块允许电压和实际接口核对后供电 |

本地C10B原理图中CN5的电源脚是3.3V，不能把它标成5V。若使用其他电源点，先核对模块规格和输出高电平，不能把电池电压直接接LF04 VCC，也不能根据线颜色或插头观看方向猜VCC/GND。接线后测供电与白底输出高电平，再进行GPIO读数测试。

探头朝地面，四路横向排列，固定在车头；按“坐在车上朝前看”的左到右定义物理顺序。厂商建议探头距地面约3cm起调，测的是红外发射/接收器到地面，不是亚克力顶板到地面。把真实白底和约1.8cm宽的黑线放在下面。

记录安装高度、探头前伸量和各通道对应的物理位置。不能仅凭O1/O4标签假定左右，因为安装方向可能翻转。

## 6 加一段只打印原始读数的程序

先完成第4节。继续使用它的Lab_Write、Lab_LinePinsOK、line_log_ms等代码，不再创建同名变量或第二个日志任务。本阶段电机不运动。

### 6.1 添加读线日志函数

**位置：main.c → USER CODE BEGIN 0，接在第4节辅助函数后。新增：**

<!-- CODE: line_function -->
```c
static void Lab_PrintLine(void)
{
    unsigned o1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET;
    unsigned o2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET;
    unsigned o3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET;
    unsigned o4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET;
    unsigned key = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET;
    unsigned en = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET;
    char msg[128];
    int n = snprintf(msg, sizeof msg,
        "O1=%u O2=%u O3=%u O4=%u KEY=%u EN_RAW=%u CFG=%s\r\n",
        o1, o2, o3, o4, key, en,
        Lab_LinePinsOK() ? "PASS" : "FAIL");
    if (n > 0 && n < (int)sizeof msg) Lab_Write(msg);
}
```

**为什么：** HAL_GPIO_ReadPin读取实际输入电平；和GPIO_PIN_SET比较后统一得到0/1。snprintf限定数组大小，检查n再发送，避免截断字符串被当成完整日志。KEY和EN_RAW同时显示，方便确认按键及使能开关的实际极性；PC13的1不能未经测量就当ON。这段没有把按键或开关接到电机控制。

### 6.2 替换主循环中的alive实验

**位置：main.c → USER CODE BEGIN 3。将第4.5节的整个if替换为：**

<!-- CODE: line_loop -->
```c
if ((uint32_t)(HAL_GetTick() - line_log_ms) >= 100U) {
    line_log_ms = HAL_GetTick();
    Lab_PrintLine();
}
```

两段共用line_log_ms，所以要替换，不能把alive和读线两个if并排留着。现在每秒约10行，黑线移动时容易观察。50ms是UART发送的超时上限，正常一行发送只需数毫秒；仍然是阻塞发送，所以只用于静态诊断，不复制进10ms中断。以后开始闭环，要移走这段或改为主循环非阻塞日志。

### 6.3 你应该看到什么，以及怎么证明通过

Generate Code（若.ioc有改动）→ 编译 → 核对输出镜像 → 烧录 → 复位。终端DTR/RTS若使板卡不断复位，关闭自动控制再观察。只改C代码时可直接编译，无需每次重新生成工程。

```text
O1=1 O2=1 O3=1 O4=1 KEY=0 EN_RAW=1 CFG=PASS
O1=0 O2=1 O3=1 O4=1 KEY=0 EN_RAW=1 CFG=PASS
```

这是“全白→仅O1对应探头见黑”的预期例子，EN_RAW以你的实测为准。按住USER时KEY应为1，松开为0；切换使能时EN_RAW应改变，记录“ON=几、OFF=几”。按键抖动是机械现象，此处只验电平，不把该日志采样当作正式去抖启动逻辑。

**通过标准与证据：** 连续输出至少30秒；CFG始终PASS；逐个遮黑四路，只有对应O由1变0并能回到1；每路黑白重复至少10次。LF04未连接时，上拉也可能让四路全1，所以“全1稳定”不构成传感器验收。保存一段全白、各路见黑、按键和开关变化的日志。

若CFG从PASS变FAIL，优先查运行中谁重新初始化了GPIO；若CFG一直PASS、模块灯变化而O不变，优先查输出线和共地；若模块灯也不变，先查第5、7节。

## 7 四个电位器怎么调

厂商手册规定：U1→O1、U2→O2、U3→O3、U4→O4。顺时针增加识别距离，逆时针降低识别距离。实际调的是比较阈值，不是小车速度或转向PID。

以下只调一条通道，其他三条先不动：

1. 固定传感器在实际运行高度，电机停止；把对应探头放在白底上。
2. 再将相同高度下的黑线移到该探头下面，观察对应通道LED和串口位。
3. 用小螺丝刀小幅旋转对应电位器，直到“白底灯灭、原始读数1；黑线灯亮、原始读数0”。避免直接转很多圈或硬顶机械限位。
4. 如果白黑都不亮/都读1，可小幅顺时针调；如果白黑都亮/都读0，可小幅逆时针调。每次都重新比较白与黑，不只找单一亮灯点。
5. 在刚出现可靠区别的区间内留出一点余量，不让LED在静止时闪烁。黑白来回移动至少10次，读数都能重复才算通过。
6. 调下一条通道，直到四路全部通过；再静止放白底/黑线各约10秒观察是否抖动。
7. 换高度、光照或场地材料后复验；避免太阳直射，不在空中调好就直接上车。

| 地面 | 通道指示LED | GPIO原始值 |
| --- | --- | --- |
| 白底 | 灭 | 1 |
| 黑线 | 亮 | 0 |

若LED按预期变化，但串口相应位不变，优先查接线、GPIO模式和PS2代码；若LED本身都不区分白黑，优先查供电、阈值、高度、真实材料与光照。

## 8 找出真正的左到右顺序

不要立刻把O1/O2/O3/O4当作左外、左内、右内、右外。

1. 四个探头放白底，应全部读1。
2. 只把最左侧探头放到黑线，观察哪个O变0，记录为左外。
3. 对左内、右内、右外重复；探头太近时可用窄黑条做单通道台架标定，正式测试场地只保留原题黑弧。
4. 在软件中按物理左到右重排为L1/L2/R2/R1，再使用巡线真值表。

| 物理位置 | 对应O编号 | 对应GPIO | 白底/黑线原始值 |
| --- | --- | --- | --- |
| 左外L1 | 待填 | 待填 | 待测 |
| 左内L2 | 待填 | 待填 | 待测 |
| 右内R2 | 待填 | 待填 | 待测 |
| 右外R1 | 待填 | 待填 | 待测 |

注意：手册真值表采用0=黑、1=白；H题C核心HCInput.black_mask采用1=黑。接核心时，先按物理顺序重排，再对每路取black=(raw==0)，将物理左外放bit0、右外放bit3。两种编码不能直接混用。

### 8.1 把实测顺序写进程序，而不是只填表

先完成上表。下面默认四项全是0，表示**尚未标定**；不会假定O1在左侧。

**位置：main.c → USER CODE BEGIN 0，接在已有辅助函数后。新增：**

<!-- CODE: mapping_functions -->
```c
/* 依次为L1、L2、R2、R1对应的O编号，填1~4，且不能重复。 */
static const uint8_t lab_o_for_lr[4] = {0U, 0U, 0U, 0U};

static int Lab_ReadMapped(uint8_t lr_raw[4], uint8_t *black_mask)
{
    uint8_t o_raw[4] = {
        HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET,
        HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET,
        HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET,
        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET
    };
    unsigned used = 0U;
    *black_mask = 0U;
    if (!Lab_LinePinsOK()) return 0;
    for (unsigned i = 0U; i < 4U; ++i) {
        unsigned o = lab_o_for_lr[i];
        if (o < 1U || o > 4U || (used & (1U << (o - 1U))) != 0U) {
            return 0; /* 未填、越界、重复都拒绝使用。 */
        }
        used |= 1U << (o - 1U);
        lr_raw[i] = o_raw[o - 1U];
        if (lr_raw[i] == 0U) *black_mask |= (uint8_t)(1U << i);
    }
    return 1;
}

static void Lab_PrintMapped(void)
{
    uint8_t lr[4], mask;
    if (!Lab_ReadMapped(lr, &mask)) {
        Lab_Write("MAP/CFG not ready; fill L1,L2,R2,R1 O numbers\r\n");
        return;
    }
    char msg[96];
    int n = snprintf(msg, sizeof msg,
        "L1L2R2R1=%u%u%u%u black_mask=0x%X\r\n",
        (unsigned)lr[0], (unsigned)lr[1],
        (unsigned)lr[2], (unsigned)lr[3], (unsigned)mask);
    if (n > 0 && n < (int)sizeof msg) Lab_Write(msg);
}
```

**位置：USER CODE BEGIN 3的100ms日志if内。把Lab_PrintLine这一行替换为：**

<!-- CODE: mapping_call -->
```c
Lab_PrintMapped();
```

如果实测L1=O4、L2=O3、R2=O2、R1=O1，才将数组改成`{4U, 3U, 2U, 1U}`；这是示例，不能未经测量就照填。

**为什么：** 先重排，再取raw==0，最后把L1放bit0。这样原始真值表和HCInput.black_mask的语义明确分开。mask不是视觉上从左到右书写的二进制串：L1在最低位。

**通过标准：** 全白→原始1111、mask=0x0；仅L1见黑→0111、0x1；仅L2→1011、0x2；仅R2→1101、0x4；仅R1→1110、0x8；两内侧→1001、0x6。分别移动黑条核对，不能用电脑上假数据替代此项实测。

## 9 电机PWM、编码器与控制周期怎样配置

以下在静态LF04读数通过后再逐项加入；配置外设不等于已经有安全电机驱动、速度PI或完整路线程序。

### 9.1 TIM3电机PWM

Timers → TIM3；CH1/CH2/CH3/CH4均选PWM Generation。核对引脚PA6、PA7、PB0、PB1，不使用TIM3重映射。Parameter Settings：

| 项目 | 本地厂商HAL基线 |
| --- | --- |
| Clock Source | Internal Clock |
| Prescaler | 0 |
| Counter Period ARR | 7199 |
| Counter Mode | Up |
| Clock Division | DIV1 |
| Auto Reload Preload | Enable |
| 四通道Mode | PWM mode 1 |
| 四通道Polarity | High |
| 停止初值Pulse | 四通道7200，配合下述驱动语义 |

72MHz/(0+1)/(7199+1)=10kHz。厂商AT8236驱动采用一路常高、另一路PWM：逻辑零命令应让同一电机两输入都高，即制动。CCR=7200大于ARR，会在PWM1/高有效下输出常高；不能只改成7199就认为完全等价。

如果MX界面对Pulse范围有限制，生成后在启动PWM前用__HAL_TIM_SET_COMPARE逐通道写7200，再按已验证的驱动顺序启动；不要让Pulse=0的默认初值直接进入运行。点动阶段始终架空、短时、一次只测一只轮，先验证停止/前进极性和使能行为。

A通道PA6/PA7，B通道PB0/PB1；A/B不是固定左右，要根据实际接到哪只轮记录。现有8MHz点动工程同ARR约1.11kHz，不把它与新72MHz基线混用。

#### 9.1.1 代码放哪里：先只有制动，没有自动点动

先将第一阶段的PA6/PA7/PB0/PB1从GPIO_Output改为TIM3四通道PWM，再Generate Code。第一阶段High输出不能与PWM功能同时配置。

**位置：main.c → USER CODE BEGIN Includes。新增`#include "tim.h"`。** htim3由CubeMX声明并初始化，不在用户区再定义一个htim3。

**位置：main.c → USER CODE BEGIN 0。新增：**

<!-- CODE: motor_functions -->
```c
static int Lab_ClampMotor(int cmd)
{
    if (cmd > 7200) return 7200;
    if (cmd < -7200) return -7200;
    return cmd;
}

/* 这里是A/B电气命令，不是已标定的左/右车轮前进命令。 */
static void Lab_MotorRawAB(int a, int b)
{
    a = Lab_ClampMotor(a);
    b = Lab_ClampMotor(b);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
                          (uint32_t)(a >= 0 ? 7200 - a : 7200));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,
                          (uint32_t)(a >= 0 ? 7200 : 7200 + a));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,
                          (uint32_t)(b >= 0 ? 7200 - b : 7200));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,
                          (uint32_t)(b >= 0 ? 7200 : 7200 + b));
}

static void Lab_MotorStartBrake(void)
{
    Lab_MotorRawAB(0, 0); /* 四通道CCR先写7200。 */
    /* OC/ARR有预装载时，先触发更新，让停止值进活动寄存器。 */
    if (HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE) != HAL_OK) {
        Lab_Write("PWM preload update FAIL\r\n");
        Error_Handler();
    }
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK) {
        Lab_Write("PWM start FAIL; keep motor power isolated\r\n");
        Error_Handler();
    }
    Lab_Write("PWM started: BRAKE A/B; CCR1..4=7200\r\n");
}
```

**位置：main.c → USER CODE BEGIN 2，MX_TIM3_Init之后。新增：**

<!-- CODE: motor_init -->
```c
Lab_MotorStartBrake();
```

新工程不在MX_TIM3_Init内部启动PWM；若改厂商工程，先删除其用户区中原来的四次HAL_TIM_PWM_Start，统一在以上函数中启动。启动各通道、引脚切换与复位仍有过渡过程，首次验证时电机驱动电源隔离、轮子架空；不能只靠CCR文字保证启动瞬间无脉冲。

**为什么这样写：** PWM1高有效时CCR=7200>ARR=7199，通道常高；同一电机两输入高对应该板的制动。正命令让IN1出现低脉冲，负命令让IN2出现低脉冲，与本地bsp_motor.c的Set_Pwm语义一致。限幅使CCR始终位于0~7200；原始命令的正负只表示电气方向，是否是车轮前进要架空实测。

**通过标准：** 有BRAKE文字；有条件时测四输入均高，示波器再测点动时PWM约10kHz；使能与停止行为按实物记录。只有CCR打印正确，不能代替波形或轮子行为。

#### 9.1.2 只测一只轮，明确给出一次性点动代码

这一步必须在静态读线已通过、停止输出已验证后再做。PC13的ON/OFF原始值用第6节实测结果；OFF是否切断驱动电源另行验证。

**位置：USER CODE BEGIN PD。新增下列宏，默认仍不运动：**

<!-- CODE: motor_test_defines -->
```c
#define LAB_ENABLE_ON_RAW (-1) /* 未测填-1；实测ON=0则填0，ON=1则填1。 */
#define LAB_TEST_A_CMD     0   /* 架空验证时只改一项，例如+400或-400。 */
#define LAB_TEST_B_CMD     0   /* 与A不能同时非零。 */
```

**位置：USER CODE BEGIN 0，放在电机函数之后。新增：**

<!-- CODE: motor_test_function -->
```c
static void Lab_MotorTestOnce(void)
{
    int a = LAB_TEST_A_CMD;
    int b = LAB_TEST_B_CMD;
    if (a == 0 && b == 0) return;
    if ((a != 0 && b != 0) || LAB_ENABLE_ON_RAW < 0 ||
        LAB_ENABLE_ON_RAW > 1) {
        Lab_Write("Motor test rejected: mapping/settings not ready\r\n");
        return;
    }
    unsigned raw = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET;
    if (raw != (unsigned)LAB_ENABLE_ON_RAW) {
        Lab_Write("Motor test skipped: enable is OFF\r\n");
        return;
    }
    Lab_Write("One wheel pulse, at most 100ms\r\n");
    uint32_t start = HAL_GetTick();
    Lab_MotorRawAB(a, b);
    while ((uint32_t)(HAL_GetTick() - start) < 100U) {
        raw = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET;
        if (raw != (unsigned)LAB_ENABLE_ON_RAW) break;
    }
    Lab_MotorRawAB(0, 0);
    Lab_Write("Pulse ended; BRAKE\r\n");
}
```

**位置：USER CODE BEGIN 2，Lab_MotorStartBrake之后。新增`Lab_MotorTestOnce();`。** 它只在开机执行一次，不放while(1)里。每次RESET/上电也会重新执行一次，因此测试完立即将两项CMD恢复0、重新编译烧录。

起点400/7200约5.6%，不保证电机克服死区；不转先看使能、接线与波形，再小步提高命令，不能直接跳满输出。一次只测A或B，记录是哪只轮、正命令转向、前进需要的符号。不要在轮子还转着时直接换正负命令；先制动并停稳。

这里轮询PC13，在开关OFF时提前停止；它没有按键去抖、低压保护、硬件急停或故障恢复，不作为正式行驶驱动。若程序停在Error_Handler或MCU异常，GPIO开关本身也不能保证切断电机电源。

### 9.2 TIM4/TIM8编码器

Timers → TIM4/TIM8 → Combined Channels选Encoder Mode；Encoder Mode选TI1 and TI2。核对TIM4_CH1/2=PB6/PB7、TIM8_CH1/2=PC6/PC7。

| 项目 | 设置 |
| --- | --- |
| Prescaler | 0 |
| Counter Period | 65535 |
| Counter Mode / Clock Division | Up / DIV1 |
| IC1/IC2 Polarity | Rising |
| IC1/IC2 Selection | Direct TI |
| IC1/IC2 Prescaler | DIV1 |
| IC1/IC2 Filter | 10，来自厂商基线 |

CubeMX上的Rising设置配合TI12编码器模式，不能简单按普通输入捕获去推断最终每圈计数；CPR必须实测。滤波10是厂商起点，高速是否漏计数仍需测试。

初始化后调用HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL)和HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL)。逐只手转轮子，确认究竟是TIM4还是TIM8计数变化；电机A/B、编码器TIM4/TIM8与物理左右的对应关系全部以接线和实测记录为准。保留自由运行计数，固定周期求有符号回绕差，不混用读取后清零的另一套驱动。

#### 9.2.1 先手转计数，不启动电机

配置TIM4/TIM8后Generate Code。此实验把两项LAB_TEST_*_CMD都保持0，不调用运动程序；电机供电隔离时若编码器也掉电，要按板上电路单独保证编码器供电与共地。

**位置：USER CODE BEGIN 2，在MX_TIM4_Init/MX_TIM8_Init之后。新增：**

<!-- CODE: encoder_init -->
```c
if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK ||
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL) != HAL_OK) {
    Lab_Write("Encoder start FAIL\r\n");
    Error_Handler();
}
```

**位置：USER CODE BEGIN 0。新增：**

<!-- CODE: encoder_functions -->
```c
static int32_t Lab_Delta16(uint16_t now, uint16_t before)
{
    /* 差值折到[-32768,32767]，避免有符号强制转换的实现差异。 */
    uint32_t d = ((uint32_t)now - (uint32_t)before) & 0xFFFFU;
    return d < 32768U ? (int32_t)d : (int32_t)d - 65536;
}

static void Lab_PrintEncoders(void)
{
    static int ready = 0;
    static uint16_t prev4 = 0U, prev8 = 0U;
    uint16_t now4 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    uint16_t now8 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);
    int32_t d4 = ready ? Lab_Delta16(now4, prev4) : 0;
    int32_t d8 = ready ? Lab_Delta16(now8, prev8) : 0;
    prev4 = now4;
    prev8 = now8;
    ready = 1;
    char msg[96];
    int n = snprintf(msg, sizeof msg, "TIM4=%u d4=%ld TIM8=%u d8=%ld\r\n",
        (unsigned)now4, (long)d4, (unsigned)now8, (long)d8);
    if (n > 0 && n < (int)sizeof msg) Lab_Write(msg);
}
```

**位置：USER CODE BEGIN 3的100ms日志if。暂时把原日志函数调用替换为`Lab_PrintEncoders();`。** 保留if的时间控制，不把日志函数每次while都调用。

**为什么：** Start真正打开编码器计数；只点CubeMX不会开始采样。前一次值要跨函数调用保留；首帧没有前一次值，delta显示0。65530→4的前进跨界差值是+10；4→65530反向是-10，不能直接用now-before当距离。要求相邻采样真实计数变化绝对值小于32768，否则无法只凭16位值判断方向。

**通过标准：** 静止不持续乱计数；手转一只轮，只有对应计数器变化；反向转时delta反号；两只轮都验证。标记轮子，慢转输出轮整整一圈，多次记录计数差，得到实测CPR。跨65535回绕时，用delta累计，不只看终点减起点。此处100ms日志用于手转诊断，不能按10ms把它换算成速度。

### 9.3 独立10ms控制定时器

在本指南的新工程中可选TIM2：Clock Source=Internal Clock，PSC=71、ARR=9999、Up；NVIC勾选TIM2 global interrupt。定时器时钟72MHz，周期(71+1)×(9999+1)/72MHz=10ms。初始化后HAL_TIM_Base_Start_IT(&htim2)。

HAL_TIM_PeriodElapsedCallback只在htim->Instance==TIM2时运行对应任务。若老师/厂商工程已经使用TIM1等做10ms控制，保留其已验证节拍，不再叠加TIM2重复执行同一PI或角度积分。

中断里不做printf、HAL_Delay、长UART发送或整屏OLED刷新；软件I²C有时间开销与超时，实际测最坏耗时。若改成中断置标志、主循环运行控制，应使用计数/时间戳发现漏拍，不能一个bool标志丢了多次节拍却仍假定dt=10ms。

#### 9.3.1 先证明中断真的每10ms发生

本实验先停止点动，两项CMD保持0。以下只是节拍诊断，尚未运行PI。

**位置：USER CODE BEGIN PV。新增：**

<!-- CODE: tick_variables -->
```c
static volatile uint32_t lab_tick10_count = 0U;
```

**位置：USER CODE BEGIN 4，放在main外面。新增：**

<!-- CODE: tick_callback -->
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        ++lab_tick10_count;
    }
}
```

整个工程只能有一份这个回调；若已有，在原函数中新增TIM2分支，不能再定义一份。volatile告诉编译器此变量可能被中断修改，不能把它当作主循环不变的值缓存。

**位置：USER CODE BEGIN 2，在MX_TIM2_Init之后。新增：**

<!-- CODE: tick_init -->
```c
__HAL_TIM_SET_COUNTER(&htim2, 0U);
__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
    Lab_Write("TIM2 start FAIL\r\n");
    Error_Handler();
}
```

**再核对stm32f1xx_it.c：** CubeMX生成TIM2_IRQHandler，内部应有`HAL_TIM_IRQHandler(&htim2);`；NVIC启用TIM2中断。不要另写一个同名中断入口。硬件中断入口→HAL_TIM_IRQHandler→PeriodElapsedCallback是完整调用链，缺一环都可能没有计数。

**位置：USER CODE BEGIN 0。新增：**

<!-- CODE: tick_function -->
```c
static void Lab_PrintTickRate(void)
{
    static uint32_t before_count = 0U, before_ms = 0U;
    static int ready = 0;
    uint32_t now_ms = HAL_GetTick();
    uint32_t now_count = lab_tick10_count;
    if (!ready) {
        before_count = now_count;
        before_ms = now_ms;
        ready = 1;
        return;
    }
    uint32_t elapsed = now_ms - before_ms;
    if (elapsed < 1000U) return;
    char msg[96];
    int n = snprintf(msg, sizeof msg, "TIM2: %lu ticks in %lu ms\r\n",
        (unsigned long)(now_count - before_count), (unsigned long)elapsed);
    if (n > 0 && n < (int)sizeof msg) Lab_Write(msg);
    before_count = now_count;
    before_ms = now_ms;
}
```

**位置：USER CODE BEGIN 3。节拍实验暂时用下面这一行替换原100ms日志if：**

<!-- CODE: tick_call -->
```c
Lab_PrintTickRate();
```

**通过标准：** 多次看到约`100 ticks in 1000 ms`，判断ticks×10是否与实际elapsed接近。开始时清除更新标志，避免初始化残留标志多算一拍。该日志验证约10ms的平均节拍，不能证明每一拍的抖动与控制任务耗时；精确验证用示波器在独立测试引脚上翻转，每次翻转10ms意味着完整方波周期20ms。

**为什么只计数：** 先把“中断是否运行”独立验证；打印仍在主循环，不让UART阻塞中断。正式PI接入前要删去/替换诊断回调，在同一个已验证的10ms任务中采样和控制。

若主循环执行控制，用计数差或时间戳识别积压：差值>1时应停机记录或按真实dt重新设计。不要连续补跑几次“10ms任务”读取同一个最新计数，也不要用bool吞掉漏拍。轮速闭环未接入前，本文没有自动运动控制任务。

### 9.4 板上MPU6050与电池ADC

MPU6050已装在你板上。厂商用PB14/PB15软件I²C，首版不要启用I2C1占用PB6/PB7编码器引脚。接入驱动时，核对SDA释放、ACK、上拉和初始化方式；不是把MX点为硬件I²C就能读通。读身份→读回量程→选实际偏航轴→静止校准→手转约90°→记录30秒漂移，详见操作手册9.0—9.3。

电池PC1对应ADC1_IN11，可设置一个软件触发通道、12位右对齐；PCLK2/8得到9MHz ADC时钟，采样时间可先选239.5cycles。生成后校准ADC并读取原始值，再按板上分压及万用表标定电压。首版先记录raw，不能凭未校准ADC设一个看似真实的电压。

厂商LF04完整巡线例程还带“电压大于7V且使能ON”的条件；该阈值属于那份例程，不是所有硬件的通用规则。出现能读线但电机不转时，也要检查使能、电压条件和驱动程序。

#### 9.4.1 电池ADC：先打印raw，再谈电压

CubeMX中配置ADC1：PC1/ADC1_IN11，Scan/Continuous/Discontinuous关闭，单通道，软件触发，右对齐，239.5cycles，ADC时钟9MHz。PC1生成模式应为Analog。Generate Code后使用以下代码。

**位置：USER CODE BEGIN Includes。新增`#include "adc.h"`。**

**位置：USER CODE BEGIN 2，在MX_ADC1_Init之后。新增：**

<!-- CODE: adc_init -->
```c
if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
    Lab_Write("ADC calibration FAIL\r\n");
    Error_Handler();
}
```

**位置：USER CODE BEGIN 0。新增：**

<!-- CODE: adc_function -->
```c
static void Lab_PrintBatteryRaw(void)
{
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        Lab_Write("ADC start FAIL\r\n");
        return;
    }
    if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc1);
        Lab_Write("ADC conversion timeout\r\n");
        return;
    }
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);
    char msg[48];
    int n = snprintf(msg, sizeof msg, "BAT_RAW=%lu\r\n", (unsigned long)raw);
    if (n > 0 && n < (int)sizeof msg) Lab_Write(msg);
}
```

**位置：USER CODE BEGIN 3。恢复第6节100ms日志if，把调用改成`Lab_PrintBatteryRaw();`。** 不同时保留每秒节拍实验的主循环版本。

**为什么：** F1 ADC上电后做校准；Start启动一次采样，Poll等转换完成，GetValue取结果，Stop结束。没有确认分压比和参考电压时，打印raw比伪造一个电池电压更可靠。此轮询ADC实验也不放进10ms中断。

**通过标准：** 输出0~4095且与万用表和板上分压关系合理，不一直卡0/4095；记录至少两组实际电池电压与raw。转换关系为`Vbat = raw / 4095 × Vref × 分压倍率`，Vref和倍率必须实测或从已核对原理图取得。不能仅凭raw稳定就宣称测压准确，更不能把电池直接接ADC脚。

#### 9.4.2 MPU6050：接入驱动，先读身份和原始数据

接入包目录：`/Users/mzh/Desktop/车车/output/固件接入包/MPU6050_9.4.2接入包`。独立操作指南：[9.4.2_先读身份和原始数据](/Users/mzh/Desktop/车车/output/固件接入包/MPU6050_9.4.2接入包/9.4.2_先读身份和原始数据.md)。

本接入包适用于当前Sunward工程：STM32F103RCT6、STM32F1 HAL、PB14/PB15软件I²C。现在只做身份、配置读回和三轴原始数据检查。轮子保持停止。

文件已做ARM编译检查；通信、电平和实际读数仍需你烧录后验证。原有Sunward项目没有被自动修改。

##### 9.4.2.1 CubeMX设置两根引脚

在Pinout中分别点击PB14、PB15，选择GPIO_Output。然后在System Core → GPIO中，设置：

| 选项 | PB14 | PB15 |
|---|---|---|
| User Label | MPU_SCL | MPU_SDA |
| GPIO output level | High | High |
| GPIO mode | Output Open Drain | Output Open Drain |
| GPIO Pull-up/Pull-down | No pull-up and no pull-down | No pull-up and no pull-down |
| Maximum output speed | High | High |

保存并Generate Code。保持PB6/PB7为TIM4编码器引脚，不启用I2C1来接这块模块。

**为什么：** PB14是时钟线SCL，PB15是数据线SDA；厂商就是用这两根线连接模块。新驱动使用开漏方式：写High表示释放总线，让模块回答；实际高电平依赖模块/线路上的上拉电阻。输出速度High是GPIO边沿设置，不是I²C通信频率。代码留有微秒延时，实际频率需仪器验证。

本驱动初始化也会设置这两根GPIO；CubeMX同步配置是为了使生成代码和驱动一致。模块已安装，先检查现有供电、接线和上拉，不改变模块电源接法。如果上拉缺失或总线拉到错误电压，需要根据实物处理；不能用推挽强拉高掩盖问题。

##### 9.4.2.2 复制四个文件到Sunward

| 本接入包中的文件 | 复制到你的工程 |
|---|---|
| Core/Inc/soft_i2c.h | /Users/mzh/STM32Cube/Sunward/Core/Inc/soft_i2c.h |
| Core/Inc/mpu6050.h | /Users/mzh/STM32Cube/Sunward/Core/Inc/mpu6050.h |
| Core/Src/soft_i2c.c | /Users/mzh/STM32Cube/Sunward/Core/Src/soft_i2c.c |
| Core/Src/mpu6050.c | /Users/mzh/STM32Cube/Sunward/Core/Src/mpu6050.c |

是复制文件，不是把文件内容全部粘到main.c。复制时如果目标已有同名文件，先检查，不覆盖你自己后来加入的驱动。

**为什么：** soft_i2c负责起始/停止、发送接收、ACK、SCL等待超时和复位后的总线恢复；mpu6050负责地址探测、芯片唤醒、配置读回和14字节连续读取。

当前EIDE配置srcDirs包含Core，因此这两份.c正常应进入构建。编译后检查build/Debug/compile_commands.json或构建输出里有soft_i2c.c和mpu6050.c；若出现undefined reference，先检查源文件是否加入构建，而不是再写空函数。

##### 9.4.2.3 main.c加头文件

在`/* USER CODE BEGIN Includes */`和`/* USER CODE END Includes */`之间，增加：

```c
#include "mpu6050.h"
#include "soft_i2c.h"
```

原有stdio.h、stdint.h、string.h保留。

##### 9.4.2.4 main.c加状态变量

在`/* USER CODE BEGIN PV */`和`/* USER CODE END PV */`之间，增加一次：

```c
static int lab_mpu_ready = 0;
```

**为什么：** 初始化失败时不继续使用无效传感器数据。本实验仅记录故障，不启动电机。

##### 9.4.2.5 main.c加打印函数

在`/* USER CODE BEGIN 0 */`区域里，放在已有Lab_Write函数后面、`/* USER CODE END 0 */`前面，增加：

```c
static void Lab_PrintMPURaw(void)
{
    if (!lab_mpu_ready) return;
    MPU6050_Raw raw;
    HAL_StatusTypeDef status = MPU6050_ReadRaw(&raw);
    if (status != HAL_OK) {
        Lab_Write("MPU read FAIL\r\n");
        return;
    }
    char msg[112];
    int n = snprintf(msg, sizeof msg,
        "MPU AX=%d AY=%d AZ=%d GX=%d GY=%d GZ=%d\r\n",
        (int)raw.ax, (int)raw.ay, (int)raw.az,
        (int)raw.gx, (int)raw.gy, (int)raw.gz);
    if (n > 0 && n < (int)sizeof msg) Lab_Write(msg);
}
```

**为什么：** AX/AY/AZ是加速度计，GX/GY/GZ是陀螺仪。本阶段同时看三个轴，先找出小车平面转弯对应的轴；不预先假定一定是GZ。

##### 9.4.2.6 main.c加初始化和身份日志

在`/* USER CODE BEGIN 2 */`区域，现有ADC校准代码后面、`/* USER CODE END 2 */`前面，增加：

```c
MPU6050_Info info;
HAL_StatusTypeDef mpu_status = MPU6050_Init(&info);
char mpu_msg[128];
int mpu_n = snprintf(mpu_msg, sizeof mpu_msg,
    "MPU status=%u ADDR=0x%02X WHO=0x%02X PWR=0x%02X "
    "DIV=%u DLPF=%u GYRO=0x%02X ACCEL=0x%02X LINES=%u\r\n",
    (unsigned)mpu_status, (unsigned)info.address, (unsigned)info.who,
    (unsigned)info.power, (unsigned)info.divider, (unsigned)info.filter,
    (unsigned)info.gyro_config, (unsigned)info.accel_config,
    (unsigned)SoftI2C_Lines());
if (mpu_n > 0 && mpu_n < (int)sizeof mpu_msg) Lab_Write(mpu_msg);
lab_mpu_ready = (mpu_status == HAL_OK);
```

身份日志只在初始化时打印一次。状态0表示HAL_OK；1表示HAL_ERROR（例如NACK、身份不符或配置读回不符），2表示HAL_BUSY，3表示HAL_TIMEOUT。LINES是当时读到的总线电平，正常空闲应为3（SCL=1、SDA=1）。LINES=0/1/2可辅助检查哪根线被拉低；即使LINES=3，也不保证对方已应答。

地址自动尝试0x68和0x69，WHO_AM_I预期均为0x68。身份值和总线地址不同。初始化失败时ADDR/WHO是最后一次探测信息或初始零值，其他字段可能还没有成功读回；不要把失败日志当作有效量程。

本驱动设置并读回：陀螺仪±250°/s、加速度计±2g、DLPF=3、采样分频9（内部数据更新100Hz），不启用DMP/FIFO/外部中断。陀螺仪raw除以131得到°/s，加速度raw除以16384得到g。先读回确认配置，再使用换算。

初始化内部有约300ms等待，仅用于初始化；不要放进TIM2中断或循环重复调用。

##### 9.4.2.7 改主循环中的一行

保留现有100ms的if和全部循环花括号，把其中`Lab_PrintBatteryRaw();`暂时替换为`Lab_PrintMPURaw();`。替换后的循环部分如下：

```c
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if ((uint32_t)(HAL_GetTick() - line_log_ms) >= 100U) {
        line_log_ms = HAL_GetTick();
        Lab_PrintMPURaw();
    }
  }
  /* USER CODE END 3 */
```

这段只覆盖while循环；其后main函数原有的最后一个`}`保留。

两个LAB_TEST_CMD继续为0，`Lab_MotorTestOnce()`继续注释。TIM2回调仍只计数，本实验传感器通信与串口打印均在主循环，不在中断里做阻塞操作。

##### 9.4.2.8 编译、烧录、复位并验收

正常启动日志格式（ADDR也可能是0x69）：

```text
MPU status=0 ADDR=0x68 WHO=0x68 PWR=0x01 DIV=9 DLPF=3 GYRO=0x00 ACCEL=0x00 LINES=3
MPU AX=... AY=... AZ=... GX=... GY=... GZ=...
MPU AX=... AY=... AZ=... GX=... GY=... GZ=...
```

带省略号的第二、三行只是格式示意，不是预设实测值。

按顺序做：

1. 核对status=0、WHO=0x68，以及PWR、DIV、DLPF、GYRO、ACCEL读回值。
2. 小车放平、静止，记录至少10行。GX/GY/GZ可以有小的非零值和抖动，不要求全部等于0。
3. 保持车身水平，手动左右转动车头，记录GX/GY/GZ中哪个变化最明显，以及左转/右转的符号。一次只做一种转动，避免抬起车身造成其他轴同时转动。
4. 若身份或读回配置失败，先检查模块电源、共地、PB14/PB15接线、上拉和构建文件；暂不做角度运算。

**本轮通过标准：** 身份和配置读回正确、原始数据持续输出、静止与手动转动的读数能区分。仅身份成功还不能证明传感器测量功能和精度全部正常。

##### 9.4.2.9 下一轮才做零偏和角度

这次记录选出的轴和左右转符号后，再接入：静止平均零偏 → 角速度扣零偏 → 按实际采样时间积分 → 手动约90°检查 → 静止30秒记录漂移。不能把raw直接当角度，也不能把100ms日志读取误当成10ms实时控制采样。

本指南完成9.4.2的通信和原始数据阶段。下一步的两个新文件和详细代码位置已提供，见[零偏与角度操作指南](/Users/mzh/Desktop/车车/output/固件接入包/MPU6050_9.4.2接入包/9.4.2_下一步_零偏与角度.md)。静止校准、90°角度和30秒漂移仍需实物验收，未宣称已完成。

##### 9.4.2.10 依据与限制

- 厂商HAL工程中的IOI2C.h确认PB14=SCL、PB15=SDA。本驱动采用开漏释放和带超时的通信流程，不依赖厂商全套Header.h、任务框架或DMP代码。
- 本地MPU6050英文寄存器手册：0x75身份值、0x6B唤醒、0x19分频、0x1A滤波、0x1B/0x1C量程、0x3B起的14字节连续数据。
- 软件延时使用STM32F103 Cortex-M3的DWT周期计数器；初始化检查计数是否运行。只用于当前前台诊断，不与其他软件I²C任务并发访问，不在运行中改变HCLK。
- 编译成功证明文件和HAL接口匹配；ACK、上拉、波形和真实测量仍要在你的板子上验证。

### 9.5 速度闭环还差什么：给出计算代码和接入位置

**2026-10-08当前进度：** 单轮与双轮架空PI、自动结束/STOP/OFF已有实测记录，当前处于第一天T06的地面负载与速度闭环验收。按[T06地面验收步骤](/Users/mzh/Desktop/车车/output/固件接入包/9.5_双轮PI闭环测试包/T06_地面速度闭环验收.md)先做两次60mm/s、3秒短跑，再补不同目标响应及地面停车记录；结果待测。下方计算函数保留作原理说明，已接入工程不重复粘贴。验收通过后进入T09航向与定距直行。

**2026-10-09进展：** 已有桌面带载短跑约15.6cm重复记录、无明显偏向，首次无抖动/自动停；用户决定暂缓追加速度及瓷砖复验，继续T09。已接入并编译[航向保持3秒版本](/Users/mzh/Desktop/车车/output/固件接入包/T09_航向保持接入包/T09_第一步_航向保持3秒.md)：OFF时CAL静止校准，READY后GO保持启动时朝向；未烧录、实车效果待测。定距停车将在下一步接入。

**2026-10-10进展：** GO桌面短跑已由用户确认无偏向、正常停止，重新CAL成功。[GO20定距版本](/Users/mzh/Desktop/车车/output/固件接入包/T09_20cm定距接入包/T09_第二步_20cm定距停车.md)已有实物日志：READY N=500，末段减速，BRAKE→DONE，最终编码器估算20.1cm；用户桌面尺测约20.2cm，短距功能初步通过，保持参数。用户决定暂缓真实地面测试，宿舍坑洼瓷砖复验不再作为继续开发门槛。已接入并编译[GO100一米与完成声光](/Users/mzh/Desktop/车车/output/固件接入包/T09_1米直行接入包/T09_第三步_1米直行与声光.md)，尚未烧录/实车验证；本阶段仅OFF下载、静止CAL检查，地面路线验证留待后续。原GO20!与GO!保留；独立按键启动将在后续接入。

PWM会输出、编码器会计数，不等于已经闭环。闭环每一拍做：读计数差→换算轮速→目标减实际→PI→限幅→驱动；目标为零/使能OFF/故障时清积分并制动。

**下面是可编译的计算函数，不是自动启动小车的控制任务。** 不把示例参数当成真实电机参数。

**位置：USER CODE BEGIN 0，放在Lab_Delta16之后。新增：**

<!-- CODE: pi_functions -->
```c
typedef struct {
    float kp;
    float ki;
    float integral;
    float max_cmd; /* 按PWM计数单位，标定后允许的上限，不能超过7200。 */
} LabPI;

static float Lab_WheelSpeed(int32_t delta, int forward_sign,
                           float wheel_circumference_m,
                           float counts_per_rev, float dt_s)
{
    if ((forward_sign != 1 && forward_sign != -1) ||
        wheel_circumference_m <= 0.0f || counts_per_rev <= 0.0f || dt_s <= 0.0f) {
        return 0.0f; /* 接入层应先验证参数，错误不能当作真实静止。 */
    }
    return (float)delta * (float)forward_sign * wheel_circumference_m /
           (counts_per_rev * dt_s);
}

/* 仅前进速度内环：输出0~max_cmd；负输出不用于倒车找线。 */
static int Lab_PIForward(LabPI *pi, float target_mps, float measured_mps,
                         float dt_s, int enabled)
{
    if (!enabled || target_mps <= 0.0f || dt_s <= 0.0f ||
        pi->max_cmd <= 0.0f || pi->max_cmd > 7200.0f) {
        pi->integral = 0.0f;
        return 0;
    }
    float error = target_mps - measured_mps;
    float candidate = pi->integral + pi->ki * error * dt_s;
    float out = pi->kp * error + candidate;
    /* 饱和且误差还在把输出推向饱和方向时，暂停积分。 */
    if (!((out > pi->max_cmd && error > 0.0f) || (out < 0.0f && error < 0.0f))) {
        pi->integral = candidate;
    }
    out = pi->kp * error + pi->integral;
    if (out < 0.0f) out = 0.0f;
    if (out > pi->max_cmd) out = pi->max_cmd;
    return (int)(out + 0.5f);
}
```

**为什么需要这些量：** 轮周长用米，CPR是输出轮一圈实测计数，dt用秒，结果才是m/s。编码器前进符号与电机前进符号是两个不同参数，分别测。Kp单位是“PWM计数/(m/s)”，Ki还包含每秒积分；不能直接抄一个使用“每10ms计数”作为速度单位的厂商PI参数。

**正式代码接入位置：** 在一个已经验收的10ms控制任务里分别读TIM4、TIM8的delta并换算为物理左/右速度；给两轮各一份LabPI；每拍调用PI一次。用实测电机映射把左右命令换成A/B及前进符号，再调用Lab_MotorRawAB。目标、标定参数、使能和故障状态不齐时，一律不启动。第9.3节的计数回调必须在此时改成真正控制任务，不能以为有个同名函数就已经接上。

不要在这里附上一组猜测CPR/Kp/Ki就宣布完成闭环。本篇给出计算实现；**完整调度、参数检查、启动去抖、低压阈值与故障停车仍需工程接入**。初期依02手册第8节先Ki=0调Kp，再加Ki；用真实目标与实际速度日志确认稳定，无持续满输出/振荡，再允许巡线转向。必须验证OFF、零目标和漏拍都能制动并复位积分。

## 10 真正的巡线控制怎么调

先验证左右轮速度闭环和前进符号，再低速验证转向方向。下面是真值表起点，前提是已按物理左→右重排、0=黑：

| L1 L2 R2 R1 | 初始处理 |
| --- | --- |
| 1011 | 左内见黑，向左修正：左轮慢、右轮快 |
| 1101 | 右内见黑，向右修正：左轮快、右轮慢 |
| 0111 | 左外见黑，加大左修正但两轮目标不为负 |
| 1110 | 右外见黑，加大右修正但两轮目标不为负 |
| 1001 | 两内侧见黑，减少额外纠偏，保持该弧段名义转向 |
| 1111 | 全白：白地正常；圆弧需丢线判断，不能直接算终点 |
| 0000/其他码 | 保存实际码，不直接算路口/圈数/结束 |

1. 先手动把黑线移到左侧，确认程序请求左转；移到右侧请求右转。方向反了先查通道顺序和轮子映射。
2. 初期用已能稳定跟随的低轮速，例如0.10m/s作为待标定起点；不是保证你的电机在此速度一定可用。速度太高会掩盖阈值/方向问题。
3. 左转为vL=v-d、vR=v+d；右转相反。先用小d，只改一个参数。示例d=0.02m/s只作起点，必须以真实响应确认；目标均≥0，不一轮倒转找线。
4. 在H题圆弧中，叠加名义曲率：R=0.4m，BC顺时针、CB逆时针、DA顺时针。居中不能因此永远两轮等速；探头及车体参考轨迹还需实际调整。
5. 左右摆动：先降速度与纠偏强度，检查阈值抖动和左右映射；转不过来：先检查方向，再逐步加纠偏，别直接增大所有PWM。
6. 黑线持续丢失就停止并记录最后读数、段号和位置，不倒车搜索。全白在AB/对角线上不是错误，这些段靠编码器与MPU6050。
7. 只验证单弧BC/CB/DA后再串路线，最后做四圈。拍摄整车投影；传感器见黑不等于整车一定覆盖端点/弧线。

不要强行把1001调成“居中时必出现”：探头间距、黑线宽与安装方式不同，实际居中码可能只有一个探头见黑。先标定实际见线区域再设计可靠逻辑。

### 10.1 先让小车不动，只打印“它打算怎么转”

完成第8节映射后，可先做这个独立静态实验。两项点动CMD保持0，移走运动控制调用，PWM若已启用则只保留制动。下面不会调用电机驱动或PI。

**位置：USER CODE BEGIN 0，放在Lab_ReadMapped之后。新增：**

<!-- CODE: dryrun_function -->
```c
static void Lab_PrintTurnRequest(void)
{
    uint8_t lr[4], mask;
    if (!Lab_ReadMapped(lr, &mask)) {
        Lab_Write("MAP/CFG not ready; no turn request\r\n");
        return;
    }
    const char *action;
    int correction = 0; /* mm/s；正数表示向左的额外差速。 */
    switch (mask) {
        case 0x2U: correction = 20;  action = "LEFT_SMALL";  break;
        case 0x4U: correction = -20; action = "RIGHT_SMALL"; break;
        case 0x1U: correction = 40;  action = "LEFT_LARGE";  break;
        case 0x8U: correction = -40; action = "RIGHT_LARGE"; break;
        case 0x6U: action = "KEEP_NOMINAL_ARC"; break;
        case 0x0U: action = "ALL_WHITE: depends on route segment"; break;
        default: action = "UNCLASSIFIED: record; do not count endpoint"; break;
    }
    char msg[160];
    int n = snprintf(msg, sizeof msg,
        "raw=%u%u%u%u mask=0x%X %s corr=%d mm/s\r\n",
        (unsigned)lr[0], (unsigned)lr[1], (unsigned)lr[2],
        (unsigned)lr[3], (unsigned)mask, action, correction);
    if (n > 0 && n < (int)sizeof msg) Lab_Write(msg);
}
```

**位置：USER CODE BEGIN 3。使用第6节100ms日志if，将调用改成`Lab_PrintTurnRequest();`。** 这是另一项实验，替换ADC/编码器/映射日志调用，不把多个阻塞日志都塞进控制任务。

**为什么先打印：** 先证明左右逻辑，不让轮子动作掩盖通道排序错误。20/40mm/s是解释差速大小的起点，必须根据实际速度闭环和赛道调；这里没有实际下发目标。mask=0x6时仅额外纠偏为0，圆弧名义差速仍应存在。

**通过标准：** 最左见黑打印LEFT_LARGE；左内LEFT_SMALL；右内RIGHT_SMALL；最右RIGHT_LARGE；全白不打印“终点”或“圈数+1”。静态验证只证明请求方向，真正转向还要结合已标定的电机前进符号和左右映射。

### 10.2 接速度闭环时，目标该怎样算

以下是数学关系，**不是可以粘贴的完整主程序**。统一向左转为正，已知标定轮距B、基准前进速度v、弧段名义角速度omega：

```text
名义差速 d_nominal = omega × B / 2
附加纠偏 d_line = 上述corr / 1000          // mm/s转为m/s
左轮目标 vL = v - d_nominal - d_line
右轮目标 vR = v + d_nominal + d_line
```

R=0.4m的名义圆弧可用omega=±v/R作为起点；顺时针为负，逆时针为正。左右目标都不得为负，并按已验收最大轮速限幅；参数不足或持续丢线时发零目标，清两轮PI积分并制动。弧线进入/退出需要路段状态与里程、航向联合判断，不能仅凭一个黑白码换段。

**为什么不能全白立刻停：** AB及对角线本来就是白地；圆弧才需要结合持续时间判丢线。丢线阈值、端点覆盖、实际居中码依实测确定，本文没有用一个猜测阈值冒充完整路线逻辑。

### 10.3 这一章真正的验收边界

先过静态转向请求，再过真实两轮闭环，随后低速单弧多次重复通过，最后才能串联路线。只打印LEFT/RIGHT不算巡线成功；只走完一段弧也不算H题成功。完整路线验收看02手册第11—14节，包括起点摆放、端点覆盖、停车、声光、时间和重复成功记录。

## 11 故障时查哪一层

| 现象 | 先查什么 |
| --- | --- |
| 串口没有字 | 代码是否实际运行、115200/8N1、端口、UART引脚/终端DTR/RTS |
| LED不分白黑 | 供电、固定高度、对应电位器、实际赛道材料、环境光 |
| LED变化但GPIO不变 | O信号接线、共地、MX输入设置、PS2旧代码 |
| 只一两路变化 | 每路单独遮黑白、排针松动、通道损坏、位置映射 |
| 静止读数抖动 | 阈值边缘、高度松动、光照和反光；先排传感器再滤波 |
| 见左线却向右转 | 物理探头顺序、左右电机映射、前进符号、差速符号 |
| 能读线但电机不转 | 是否启动PWM、使能、电池/ADC条件、是否有真实驱动实现 |
| 直线全白就停 | 把巡线丢线逻辑错误用于AB/对角线白地区段 |

本指南配置依据：LF04使用手册2026.5.19版第3—4页原理/调节、第8页C10B接线、第12页环境注意；C10B厂商HAL ZIP中的miniBlance.ioc、Src/tim.c、Src/gpio.c、Src/usart.c、MPU6050 IOI2C.h及本地C10B原理图。厂商2026年H题相关例程与本目录2024年H题不是同一赛题，不直接照搬其完整赛道逻辑。


## 12 今天实际照做的顺序与代码清理

| 当前实验 | USER CODE BEGIN 2新增/保留 | USER CODE BEGIN 3用哪种版本 | 通过后保存什么 |
| --- | --- | --- | --- |
| 第4节串口与指示器 | Quiet、PrintBoot、可选IndicatorSelfTest | 1000ms的alive if | 开机与alive日志、指示器现象 |
| 第6—7节静态LF04 | 同上，SELFTEST恢复0 | 100ms if调用PrintLine | 四路黑白、KEY、EN_RAW、CFG记录 |
| 第8节左右映射 | 同上 | 100ms if调用PrintMapped | 物理顺序、raw与mask对照 |
| 第9.1节PWM与点动 | 增加MotorStartBrake；点动时可选MotorTestOnce | 保留100ms的静态日志 | 左右电机、前进符号、停止与使能验证 |
| 第9.2节编码器 | CMD归零，增加Encoder_Start | 100ms if调用PrintEncoders | TIM4/TIM8物理映射、方向与CPR |
| 第9.3节节拍 | CMD归零，增加Base_Start_IT | 替换成PrintTickRate一行 | 每段elapsed与ticks；回调只计数 |
| 第9.4节ADC | CMD归零，增加ADC校准 | 恢复100ms if调用PrintBatteryRaw | 万用表电压与raw标定 |
| 第10.1节转向请求 | CMD归零，只制动，不启动运动任务 | 100ms if调用PrintTurnRequest | 五种可识别状态与全白结果 |

辅助函数可以保留；未被当前实验调用的static函数可能产生unused-function警告，先看它是否真的该接入，不随便删除错误或把警告关闭。若工程将此警告作为错误，可移走尚未使用的实验函数，按章节需要再加回。不得重复定义同名变量/函数/回调。

**开始正式运动前清理：** 点动宏恢复0；SELFTEST恢复0；启动时的阻塞测试不再执行；正式10ms任务替换诊断计数回调；低频串口改到主循环非阻塞发送；标定参数、左右/前进映射、使能和故障停车全部明确。Lab_ReadMapped的失败必须拒绝控制，不能继续用输出数组里的旧值。

### 12.1 常见编译报错直接定位

| 报错/现象 | 该检查什么 | 为什么 |
| --- | --- | --- |
| usart.h/tim.h/adc.h: No such file | 是否选了外设分文件；外设是否启用并生成 | 头文件不是手写一行include就会存在 |
| huart1/htim4/hadc1 undeclared | 对应头文件与生成句柄声明；不要自己另造同名句柄 | 调用必须指向已初始化的真实外设 |
| implicit declaration of Lab_… | USER CODE BEGIN 0内辅助函数是否在调用之前、顺序正确 | C编译器需要先见定义或声明 |
| multiple definition / redefinition | 是否重复粘贴函数、回调或变量 | 一个名字应只有一份定义 |
| undefined reference to MPU6050_… | 是否真的加入驱动.c并纳入构建 | 有声明不等于有函数实现 |
| 编译成功但开机标识没变化 | 构建输出目录、镜像时间、下载目标、Flash校验 | 很可能运行的是旧镜像 |
| CFG=FAIL | GPIO配置与旧PS2重初始化 | 算法不能修复引脚被设成输出的问题 |
| 电机仍会自动转 | 厂商初始化内PWM启动、旧控制中断、点动宏是否归零 | 检查完整调用链，不能只看当前main循环 |

### 12.2 本版代码检查与实物边界

本次已完成：25组标记代码片段按所述用户区拼接，使用STM32CubeIDE自带ARM GCC 14.3.1、STM32F103xE、Cortex-M3及本地STM32F1 HAL头文件，通过-Wall/-Wextra/-Werror编译检查；各主循环版本也已单独检查。电脑数值用例通过：四路16种黑白组合、反向/无效映射、输入上拉与输出冲突、16位计数回绕、轮速单位、PI限幅与停止清积分、电机CCR限幅。**这些都不替代完整CubeMX工程链接、烧录、传感器供电测量或实车验收。** 本次修改手册不生成完整路线固件，也不烧录或驱动你的车。

HAL接口和F1 GPIO模式依据：[ST官方GPIO实现](https://github.com/STMicroelectronics/stm32f1xx-hal-driver/blob/master/Src/stm32f1xx_hal_gpio.c)、[ST官方UART实现](https://github.com/STMicroelectronics/stm32f1xx-hal-driver/blob/master/Src/stm32f1xx_hal_uart.c)、[ST官方TIM实现](https://github.com/STMicroelectronics/stm32f1xx-hal-driver/blob/master/Src/stm32f1xx_hal_tim.c)。板上引脚与电机语义仍以第11节所列本地C10B资料为依据，不能把另一款板的接线替换进来。
