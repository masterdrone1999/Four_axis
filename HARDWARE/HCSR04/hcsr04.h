#ifndef __HCSR04_H
#define __HCSR04_H

#include "sys.h"

#define HCSR04_PORT     GPIOA
#define HCSR04_CLK      RCC_APB2Periph_GPIOA
#define HCSR04_TRIG     GPIO_Pin_5
#define HCSR04_ECHO     GPIO_Pin_6

void hcsr04_Init(void);
void Get_Distance(void);

extern u8  TIM3CH1_CAPTURE_STA;	//输入捕获状态		    				
extern u16	TIM3CH1_CAPTURE_VAL;	//输入捕获值



#endif /*__HCSR04_H*/



