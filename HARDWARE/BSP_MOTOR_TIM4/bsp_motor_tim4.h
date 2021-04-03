#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "sys.h"

//MOT_GPIO PB6,7,8,9
#define MOT_GPIO					GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9
#define MOT_GPIO_APBxClock_FUN		RCC_APB2PeriphClockCmd
#define MOT_GPIO_CLK				RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO

#define MOT_TIM						TIM4
#define MOT_TIM_APBxClock_FUN 		RCC_APB1PeriphClockCmd
#define MOT_TIM_CLK 				RCC_APB1Periph_TIM4


void PWM_TIM_GPIO_Config(void);
void PWM_OC_Config(void);
void PWM_Init(void);
void MOT_Control(uint16_t ccr1,uint16_t ccr2,uint16_t ccr3,uint16_t ccr4);

////加速按键KEY1 		PE3
//#define MOT_PLUS_KEY_PORT			GPIOE
//#define MOT_PLUS_KEY_PIN			GPIO_Pin_3
////减速按键KEY0 		PE4
//#define MOT_SUB_KEY_PORT			GPIOE
//#define MOT_SUB_KEY_PIN				GPIO_Pin_4
////刹车按键KEY_UP 	PA0
//#define MOT_STOP_KEY_PORT			GPIOA
//#define MOT_STOP_KEY_PIN			GPIO_Pin_0

//static void NVIC_Config(void);
//static void EXTI_Config(void);
//void KEY_EXTI_Config(void);

#endif /*__BSP_MOTOR_H*/
