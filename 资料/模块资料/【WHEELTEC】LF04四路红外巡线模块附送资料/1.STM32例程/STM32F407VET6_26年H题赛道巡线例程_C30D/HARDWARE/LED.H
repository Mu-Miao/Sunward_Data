#ifndef __LED_H
#define __LED_H
#include "sys.h"



/*--------Buzzer control pin--------*/
#define Buzzer_PORT GPIOA
#define Buzzer_PIN GPIO_Pin_8
#define Buzzer PAout(8)
/*----------------------------------*/

/*--------LED control pin--------*/
#define LED_PORT GPIOA
#define LED_PIN GPIO_Pin_12
//旧板C30DLED控制口
#define V1_0_LED PAout(12) 
/*----------------------------------*/
//新板C30DLED控制口
#define V2_0_LED PEout(8) 


void V1_0_LED_Init(void);
void V2_0_LED_Init(void);  
void Buzzer_Init(void); 
void LED_Flash_Ctrl(uint16_t flash_times);
#endif
