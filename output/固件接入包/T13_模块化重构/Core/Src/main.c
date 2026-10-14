/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "adc.h"
#include "mpu6050.h"
#include "soft_i2c.h"
#include "yaw_diag.h"
#include "wheel_speed_diag.h"
#include "car_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LAB_INDICATOR_SELFTEST 0
#define LAB_ENABLE_ON_RAW 0 /* 未测填-1；实测ON=0则填0，ON=1则填1。 */
#define LAB_TEST_A_CMD     0   /* 架空验证时只改一项，例如+400或-400。 */
#define LAB_TEST_B_CMD     0   /* 与A不能同时非零。 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t line_log_ms = 0U;
static volatile uint32_t lab_tick10_count = 0U;
static int lab_mpu_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
static int32_t Lab_Delta16(uint16_t now, uint16_t before)
{
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
    int n = snprintf(msg, sizeof msg,
        "TIM4=%u d4=%ld TIM8=%u d8=%ld\r\n",
        (unsigned)now4, (long)d4,
        (unsigned)now8, (long)d8);
    if (n > 0 && n < (int)sizeof msg) Lab_Write(msg);
}
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

static void BT_TestRun(void)
{
    uint8_t ch;
    char line[64];
    char reply[96];
    uint16_t used = 0U;
    uint8_t discard = 0U;
    uint8_t boot[] =
        "\r\nBT TEST: USART3 9600; send hello + LF\r\n";

    __HAL_UART_CLEAR_OREFLAG(&huart3);
    HAL_UART_Transmit(&huart1, boot, sizeof boot - 1U, 100U);

    while (1) {
        HAL_StatusTypeDef status =
            HAL_UART_Receive(&huart3, &ch, 1U, 20U);

        if (status != HAL_OK) {
            if (status == HAL_ERROR) {
                __HAL_UART_CLEAR_OREFLAG(&huart3);
                used = 0U;
                discard = 1U;
            }
            continue;
        }

        if (ch == '\r') continue;

        if (ch == '\n') {
            int n = 0;

            if (discard) {
                n = snprintf(reply, sizeof reply,
                             "ERR: line too long or RX error\r\n");
            } else if (used > 0U) {
                line[used] = '\0';
                n = snprintf(reply, sizeof reply,
                             "ECHO: %s\r\n", line);
            }

            if (n > 0 && n < (int)sizeof reply) {
                HAL_UART_Transmit(&huart3, (uint8_t *)reply,
                                  (uint16_t)n, 200U);
                HAL_UART_Transmit(&huart1, (uint8_t *)reply,
                                  (uint16_t)n, 100U);
            }

            used = 0U;
            discard = 0U;
        } else if (!discard) {
            if (used < sizeof line - 1U) {
                line[used++] = (char)ch;
            } else {
                discard = 1U;
            }
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  Lab_Quiet();
  Lab_PrintBoot();
  Lab_IndicatorSelfTest();
  Lab_MotorStartBrake();
  // BT_TestRun();
  // Lab_MotorTestOnce(); 
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK ||
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL) != HAL_OK) {
    Lab_Write("Encoder start FAIL\r\n");
    Error_Handler();
}
__HAL_TIM_SET_COUNTER(&htim2, 0U);
__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
    Lab_Write("TIM2 start FAIL\r\n");
    Error_Handler();
}
if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
    Lab_Write("ADC calibration FAIL\r\n");
    Error_Handler();
}
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
if (CarApp_Begin(&huart1, &htim8, &htim4, &hadc1,
                     Lab_MotorRawAB) != HAL_OK) {
    Lab_Write("DPI init FAIL\r\n");
    Error_Handler();
}
if (CarApp_EnableBluetooth(&huart3) != HAL_OK) {
    Lab_MotorRawAB(0, 0);
    Error_Handler();
}
CarApp_EnableHeading(lab_mpu_ready);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
  // if ((uint32_t)(HAL_GetTick() - line_log_ms) >= 100U) {
  //   line_log_ms = HAL_GetTick();
  //   Lab_PrintLine();
  //   Lab_PrintEncoders();
  //   Lab_PrintTickRate();
  //   Lab_PrintBatteryRaw();
  //   Lab_PrintMPURaw();
  // }
  CarApp_Poll(lab_tick10_count);
  // YawDiag_Poll();
  
}
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        ++lab_tick10_count;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
