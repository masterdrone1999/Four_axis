/*****************************************************
* 						控制电机转速
*满油门  CCR 2000 10%
*零油门	 CCR 1000 5%	
*
2021年1月31日10:13:03
*
**************************************************************/

#include "bsp_motor_tim4.h"

/**  TIM4定时器基础配置
  *  功能：
  *  入口参数：arr；psc
  *  返回值：无
  */
void PWM_TIM_GPIO_Config()
{
	GPIO_InitTypeDef			GPIO_InitStructure;		
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	
	MOT_GPIO_APBxClock_FUN(MOT_GPIO_CLK,ENABLE);
	MOT_TIM_APBxClock_FUN(MOT_TIM_CLK,ENABLE);
		
	//PB6,7,8,9 配置
	GPIO_InitStructure.GPIO_Pin = MOT_GPIO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB,&GPIO_InitStructure);	

	//TIM5基础配置	
	//计时器时钟 = APB总线时钟/分频因子
	//							psc
	TIM_TimeBaseStructure.TIM_Period = 20000-1;//重装载值/最大计数值
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;//预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(MOT_TIM,&TIM_TimeBaseStructure);	
	
}

/**  PWM 配置
  *  功能：配置四路PWM控制电机,后期可扩展为电机解锁程序
  *  入口参数：无
  *  返回值：无
  */
void PWM_OC_Config(void)
{
	TIM_OCInitTypeDef			TIM_OCInitStructure;
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	
	//前左电机 CCR1 PB6     
	TIM_OC1Init(MOT_TIM, &TIM_OCInitStructure); 
	TIM_OC1PreloadConfig(MOT_TIM, TIM_OCPreload_Enable); //使能MOT_TIM在CCR1上的预装载寄存器
	//前右电机 CCR2 PB7 
	TIM_OC2Init(MOT_TIM, &TIM_OCInitStructure); 
	TIM_OC2PreloadConfig(MOT_TIM, TIM_OCPreload_Enable); //使能MOT_TIM在CCR2上的预装载寄存器
	//后左电机 CCR3 PB8
	TIM_OC3Init(MOT_TIM, &TIM_OCInitStructure); 
	TIM_OC3PreloadConfig(MOT_TIM, TIM_OCPreload_Enable); //使能MOT_TIM在CCR3上的预装载寄存器
	//后右电机 CCR4 PB9
	TIM_OC4Init(MOT_TIM, &TIM_OCInitStructure); 
	TIM_OC4PreloadConfig(MOT_TIM, TIM_OCPreload_Enable); //使能MOT_TIM在CCR4上的预装载寄存器

	TIM_ARRPreloadConfig(MOT_TIM, ENABLE); //使能MOT_TIM在ARR上的预装载寄存器
	TIM_Cmd(MOT_TIM, ENABLE);  //使能MOT_TIM外设
	
}

void PWM_Init(void)
{
	PWM_TIM_GPIO_Config();
	PWM_OC_Config();	
}



/**  电机控制程序
  *  功能：控制四个通道的CCR
  *  入口参数：ccr1，ccr2，ccr3，ccr4
  *  返回值：无
  *	 注：取值[0,0xFFFF]
  */
void MOT_Control(uint16_t ccr1,uint16_t ccr2,uint16_t ccr3,uint16_t ccr4)
{
	if(ccr1<1000)	ccr1=1000;			
	if(ccr1>2000)	ccr1=2000;
	if(ccr2<1000)	ccr2=1000;			
	if(ccr2>2000)	ccr2=2000;
	if(ccr3<1000)	ccr3=1000;			
	if(ccr3>2000)	ccr3=2000;		
	if(ccr4<1000)	ccr4=1000;			
	if(ccr4>2000)	ccr4=2000;
	
	TIM4->CCR1 = ccr1;
	TIM4->CCR2 = ccr2;
	TIM4->CCR3 = ccr3;
	TIM4->CCR4 = ccr4;
}
