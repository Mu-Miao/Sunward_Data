
#ifndef MOCK_HAL_H
#define MOCK_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct {uint32_t count;} TIM_HandleTypeDef;
typedef struct {int x;} ADC_HandleTypeDef;
typedef struct {int x;} UART_HandleTypeDef;
typedef struct {uint32_t CYCCNT, CTRL;} DWT_Type;
typedef struct {uint32_t DEMCR;} CoreDebug_Type;
extern DWT_Type mock_dwt;
extern CoreDebug_Type mock_core;
#define DWT (&mock_dwt)
#define CoreDebug (&mock_core)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define GPIOB ((void*)3)
#define GPIO_PIN_4 16U
#define GPIO_PIN_8 256U
#define GPIO_PIN_9 512U
#define GPIOA ((void*)2)
#define GPIO_PIN_0 1U
#define GPIO_PIN_2 4U
#define GPIO_PIN_3 8U
#define GPIO_PIN_15 32768U
void HAL_GPIO_WritePin(void*,uint32_t,int);
#define GPIOC ((void*)1)
#define GPIO_PIN_13 8192U
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define __HAL_TIM_GET_COUNTER(t) ((t)->count)
uint32_t HAL_GetTick(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
void HAL_Delay(uint32_t);
int HAL_GPIO_ReadPin(void*,uint32_t);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef*);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef*,uint32_t);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef*);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef*);
#endif
