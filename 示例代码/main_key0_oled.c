/* USER CODE BEGIN Header */
/* KEY0 EXTI interrupt demonstration with SSD1306 OLED output. */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include "oled.h"
#include <stdio.h>
/* USER CODE END Includes */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void Update_Interrupt_Count(uint32_t count);
/* USER CODE END PFP */

/* USER CODE BEGIN PV */
static volatile uint32_t key0_irq_count = 0U;
static volatile uint8_t key0_event = 0U;
/* USER CODE END PV */

int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t displayed_count = 0U;
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  OLED_Init();
  OLED_Clear();
  OLED_WriteString(0U, 0U, "INTERRUPT TEST");
  OLED_WriteString(0U, 2U, "PRESS KEY0");
  Update_Interrupt_Count(0U);
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    if (key0_event != 0U)
    {
      /* Copy and clear the ISR data atomically. */
      __disable_irq();
      displayed_count = key0_irq_count;
      key0_event = 0U;
      __enable_irq();

      Update_Interrupt_Count(displayed_count);
      HAL_GPIO_TogglePin(DENG0_GPIO_Port, DENG0_Pin);
    }
    /* USER CODE END 3 */
  }
}

/* USER CODE BEGIN 0 */
static void Update_Interrupt_Count(uint32_t count)
{
  char text[22];

  OLED_WriteString(0U, 4U, "                     ");
  (void)snprintf(text, sizeof(text), "IRQ COUNT: %lu", (unsigned long)count);
  OLED_WriteString(0U, 4U, text);
}
/* USER CODE END 0 */

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t last_interrupt_tick = 0U;
  uint32_t current_tick;

  if (GPIO_Pin != KEY0_Pin)
  {
    return;
  }

  current_tick = HAL_GetTick();

  /* Ignore switch bounce for 80 ms. */
  if ((current_tick - last_interrupt_tick) >= 80U)
  {
    last_interrupt_tick = current_tick;
    key0_irq_count++;
    key0_event = 1U;
  }
}
/* USER CODE END 4 */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                               RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOF, DENG0_Pin | DENG1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA,
                    OLED_SCL_Pin | OLED_SDA_Pin | OLED_RES_Pin | OLED_DC_Pin,
                    GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = KEY0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY0_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DENG0_Pin | DENG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = XUN0_Pin | XUN1_Pin | XUN2_Pin | XUN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = OLED_SCL_Pin | OLED_SDA_Pin |
                        OLED_RES_Pin | OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
