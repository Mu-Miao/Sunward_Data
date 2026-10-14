# STM32F407Z + 0.96-inch SSD1306 OLED

This is a software-SPI driver for a CubeMX-generated HAL project.

## CubeMX GPIO labels

Configure all four pins as `GPIO_Output`, push-pull, no pull, high speed:

| OLED signal | MCU pin | User Label | Initial level |
| --- | --- | --- | --- |
| SCL | PA9 | `OLED_SCL` | Low |
| SDA | PA10 | `OLED_SDA` | Low |
| RES | PA11 | `OLED_RES` | High |
| DC | PA12 | `OLED_DC` | Low |

Do not enable USART1 or USB on these pins while the OLED is connected.

## Add to the CubeMX project

1. Copy `oled.c` to `Core/Src`.
2. Copy `oled.h` to `Core/Inc`.
3. Add `#include "oled.h"` inside the `USER CODE BEGIN Includes` section of `main.c`.
4. After `MX_GPIO_Init()`, call:

```c
OLED_Init();
OLED_WriteString(16, 1, "STM32F407Z");
OLED_WriteString(22, 3, "OLED TEST");
OLED_WriteString(34, 5, "HELLO!");
```

The included font supports printable ASCII, not Chinese characters.

## GIF animation

The SSD1306 cannot decode a GIF itself. Convert the GIF to monochrome C arrays
on the computer first (requires Pillow):

```sh
python3 tools/gif_to_oled.py input.gif animation.h --name demo
```

For a transparent sticker-style GIF, use its opacity as a white silhouette:

```sh
python3 tools/gif_to_oled.py input.gif animation.h --name demo --alpha
```

For a black subject on a transparent background, another option is to flatten
onto white before inverting:

```sh
python3 tools/gif_to_oled.py input.gif animation.h --name demo \
  --background white --invert
```

Copy the generated `animation.h` to `Core/Inc`, include it from `main.c`, and
draw one frame at a time:

```c
#include "animation.h"

uint16_t frame = 0;

while (1)
{
    OLED_DrawFrame(demo_frames[frame]);
    HAL_Delay(demo_delays_ms[frame]);
    frame = (frame + 1U) % DEMO_FRAME_COUNT;
}
```

Each full 128x64 monochrome frame occupies 1024 bytes of Flash. Limit or skip
frames when converting a long GIF so the generated array fits in the MCU.
