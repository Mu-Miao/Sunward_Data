#include "led.h"
uint16_t led_flash_cnt = 0;
/**************************************************************************
Function: LED interface initialization
Input   : none
Output  : none
函数功能：LED接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
void V1_0_LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOB时钟
  GPIO_InitStructure.GPIO_Pin =  LED_PIN;//LED对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
	GPIO_SetBits(GPIOA,GPIO_Pin_12);
}

void V2_0_LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // 使能GPIOE时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    // 配置GPIO_InitStructure结构体
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; // PE8对应IO口
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // 普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 不使用上下拉
    // 初始化GPIOE
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_SetBits(GPIOE, GPIO_Pin_8);
}
/**************************************************************************
Function: Buzzer interface initialized
Input   : none
Output  : none
函数功能：蜂鸣器接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
void Buzzer_Init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOB时钟
  GPIO_InitStructure.GPIO_Pin =  Buzzer_PIN;//LED对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
}

/**************************************************************************
Function: LED闪烁控制函数（执行指定次数后取反LED状态）
Input   : flash_times - 需执行的次数，达到该次数后LED状态取反
Output  : none
函数功能：
          1. 根据Board_Ver区分V1.0/V2.0版本LED
          2. 累计调用次数，达到flash_times后取反LED状态
          3. 计数达到阈值后自动清零，等待下一轮计数
**************************************************************************/
void LED_Flash_Ctrl(uint16_t flash_times)
{
    // 计数累加
    led_flash_cnt++;
    
    // 达到指定次数后执行LED状态取反
    if(led_flash_cnt >= flash_times)
    {
        // 根据硬件版本选择对应LED
        switch(Board_Ver)
        {
            case 1:  // V1.0版本（旧板C30D，PA12）
                V1_0_LED = !V1_0_LED;  // LED状态取反
                break;
                
            case 2:  // V2.0版本（新板C30D，PE8）
                V2_0_LED = !V2_0_LED;  // LED状态取反
                break;
                
            default: // 默认使用V1.0版本，防止版本号错误
                V1_0_LED = !V1_0_LED;
                break;
        }
        
        // 计数清零，准备下一轮
        led_flash_cnt = 0;
    }
}
