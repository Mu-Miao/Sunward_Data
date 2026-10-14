# OLED试亮与任务状态显示

2026-10-11。OLED驱动已经编入固件，用户不用另装电脑驱动或下载库。老师给的屏幕未标显示控制芯片型号：正面六针GND/VCC/SCL/SDA/RES/DC，背面写Default:4SPI，照片中R3与C5在默认位置，R1/C7空置；据此按默认四线SPI接入。本版采用SSD1306、128×64配置试亮，不宣称已经识别或验证实际控制器。U1外观和排线标字不作为控制芯片型号证据。

已在实际Sunward工程接入并完整编译，软件验证通过；未代为烧录、上电或确认屏幕显示。用户确认之前按键版行驶102cm且全程无异常，这属于旧按键版现场记录，不能替代本版OLED或控制耗时现场验证。

## 现在只做静态试亮

1. 保持实体使能 **OFF**。现有六根线沿用，先确认屏幕VCC接H1的3.3V、GND接H1的GND。
2. 烧录[本版Sunward.hex](/Users/mzh/Desktop/车车/output/固件接入包/T11_OLED试亮接入包/Sunward.hex)。它与[当前工程HEX](/Users/mzh/STM32Cube/Sunward/build/Debug/Sunward.hex)一致；先前T10按键包仍保留旧HEX。
3. 上电或复位，松开KEY_U，约1秒内预期显示以下四行。此步保持OFF，先确认有字、无缺边或乱码，再继续校准显示检查。

```text
TASK 1 A->B
YC NONE EN OFF
STATE IDLE
S 0/1000mm
```

这是正常MPU初始化、未执行CAL时的预期；若YC FAIL，需要结合原陀螺仪日志排查。OLED是否有字不能证明陀螺仪已READY。

4. 显示正常后，保持OFF、车放稳，短按并松开KEY_U：YC应经历WARM→CAL→READY，约7～8秒；LED1沿用慢闪到常亮。现在先检查静态显示，保留ON按键一米启动功能供后续使用。

若黑屏、花屏、文字缺边或方向不合适，先保持OFF，记录显示现象/正面照片；黑屏不直接判坏屏。优先核对实际供电、六线顺序和新固件版本，再考虑控制器初始化、列偏移或扫描方向差异。背面IIC改焊说明是接口转换说明，本阶段不用改焊电阻或电容。

## 接线与显示含义

| 屏幕标字 | H1针号 | MCU/电源 |
|---|---:|---|
| GND | 1 | GND |
| VCC | 2 | 3.3V |
| SCL | 3 | PC14时钟 |
| SDA | 4 | PB5数据 |
| RES | 5 | PB4屏幕复位 |
| DC | 6 | PB3命令/数据 |

主板H1定义已核对本地C10B原理图第2页；认板上1脚/GND标记，不凭照片左右猜方向。正面SCL/SDA标字共用于不同接口，背面Default:4SPI与跳线位置才是这次选择SPI的依据。

第一行TASK 1 A->B表示当前固定一米任务；暂未加入任务2～4、按键选任务或LF04完整路线。GO20显示TEST GO20，GO显示TEST GO，RUN显示TEST RUN，手动控制显示MANUAL。第二行YC是陀螺仪校准状态，EN是实际使能ON/OFF。第三行STATE显示IDLE/RUN/BRAKE/DONE/STOP或故障名。第四行S是编码器估计里程/目标（mm），不代替尺量距离；手动或限时试跑显示左轮目标速度V。

按键、LED与声光行为继续沿用上一版：OFF按键CAL，READY+ON稳定后新按键GO100，忙时新按键STOP；GO100正常停稳后双灯与蜂鸣器提示一次500ms。PI、速度、周长、CPR和航向参数未改。屏幕显示只读取控制状态，不控制运动。

## 驱动与验证

新增[task_oled.c](/Users/mzh/STM32Cube/Sunward/Core/Src/task_oled.c)和[task_oled.h](/Users/mzh/STM32Cube/Sunward/Core/Inc/task_oled.h)，在dual_pi_test.c中接入。使用H1软件SPI；保持既有NOJTAG/SWD配置，GPIO初始化仅新增PC14/PB3/PB4/PB5。PC14使用2MHz输出驱动，串行时钟软件限定不超过1MHz，依据[ST数据手册](https://www.st.com/resource/en/datasheet/stm32f103rc.pdf)的PC13～PC15限制。

文本四行、每行最多21个ASCII字符，字体纵向加倍；后台每次最多3个定位命令加8字节显存数据，同一毫秒最多一块，1024字节帧分128块刷新。刷新途中冻结帧，避免新旧文字混合。文本每200ms尝试更新，显示可能落后控制状态约几百毫秒；停车不等显示刷新完成。运行中不调用HAL_Delay；仅启动、10ms控制尚未primed前屏幕复位各延时20ms。

新增日志`OLED TRY=SSD1306 SPI=H1 IO_US=...`，TRY说明所用配置而非检测结果，IO_US是最近一次小块发送耗时；屏幕SPI只有写出，没有型号读回或ACK，所以未接屏幕也不会阻塞控制，也不能凭这条日志确认屏幕已经亮。实际控制DT、OLED耗时和显示效果尚未采集。

主机验证采用真实驱动和模拟HAL，覆盖SPI上升沿解码、MSB顺序、DC边界、初始化参数、完整页/列覆盖、8字节分块与同毫秒抑制、清屏、字体、边界截断、无效字符、冻结帧、毫秒/DWT回绕及无运行时HAL_Delay；运行全部旧按键/GO100/GO20/航向/手动保护回归。临时目录和实际工程全量ARM编译成功，FLASH31216B、RAM5288B；新增代码无警告，仅原main unused警告。main、MPU、软件I2C、UART、gpio.c、.ioc与EIDE配置未改；builder.params增加新源文件，EIDE原Core目录扫描配置可发现它。

采用[SSD1306官方支持的SPI接口](https://www.solomon-systech.com/zh-hant/product/ssd1306)作为首个试亮方案，但屏幕实物型号和兼容性尚待显示结果确认。

## 备份与回退

已备份上一版完整源码与build/Debug：[备份目录](/Users/mzh/Desktop/车车/output/固件备份/Sunward_OLED接入前备份_2026-10-11_to31kzet)。需要回到原按键固件可烧录备份original/build/Debug/Sunward.hex，其SHA256为`c921090bcda3f98a541745def3f25d3100538d183d87388a6469f47c7c7e8e95`。恢复源码时还需恢复旧dual_pi_test.c/.h并移除新增task_oled.c/.h和builder.params中的新源文件项。

本版HEX SHA256：`2649c47370be8b2dbddff3c77009f37511a8ba23caf41c01b01fd6032d4e3c46`。

证据：[主机回归](/Users/mzh/Desktop/车车/output/固件接入包/T11_OLED试亮接入包/主机验证.txt)、[SPI验证](/Users/mzh/Desktop/车车/output/固件接入包/T11_OLED试亮接入包/SPI主机验证.txt)、[全量编译](/Users/mzh/Desktop/车车/output/固件接入包/T11_OLED试亮接入包/工程编译验证.txt)、[接入记录](/Users/mzh/Desktop/车车/output/固件接入包/T11_OLED试亮接入包/接入记录.json)、[差异](/Users/mzh/Desktop/车车/output/固件接入包/T11_OLED试亮接入包/接入差异.patch)。

## 亮屏结果与新方向版本（2026-10-11）

用户照片已确认本试亮驱动能显示四行内容，但存在左右镜像。实际工程当前更新为[OLED镜像修正版](/Users/mzh/Desktop/车车/output/固件接入包/T11_OLED镜像修正版/Sunward.hex)，仅水平重映射A1→A0，保持OFF烧录后检查文字方向；本目录保留旧试亮HEX。新方向实物效果待确认。
