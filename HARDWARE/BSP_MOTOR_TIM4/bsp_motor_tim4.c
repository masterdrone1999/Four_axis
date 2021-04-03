/*****************************************************
* 						通过按键中断方式控制电机转速
（优先级 KEY0和KEY1属于不同操作，优先级KEY0>KEY1（减速>加速））
*满油门  CCR 200 10%
*零油门	 CCR 100 5%	
*	KEY0	（PE4） 减速		[0,20]次
*	KEY1	（PE3） 加速		[0,20]次
*	KEY_UP	（PA0） 刹车 		CCR=5%
*现阶段控油门转速仅有五个等级,后期按比例增加，按等比例分配为 100,105,110,115,120,125,130,135,140,145,150,155,160,165,170,175,180
*其中KEY_UP仅用作电调初始化，初始CCR=200（10%），KEY_UP为CCR=100（5%），可以做紧急刹车使用
*
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

///**  按键中断配置
//  *  功能：配置三个按键的中断设置
//  *  入口参数：无
//  *  返回值：无
//  */
//static void EXTI_Config(void)
//{
//	EXTI_InitTypeDef EXTI_InitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
//	
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource3);//加速键PE3
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource4);//减速键PE4
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);//停止键PA0

//	EXTI_InitStructure.EXTI_Line = EXTI_Line3 | EXTI_Line4 | EXTI_Line0;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
//	
//}

//static void NVIC_Config(void)
//{
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	//加速键PE3
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);  
//	//减速键PE4
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);  
//	//停止键PA0 
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);  
//}

//void KEY_EXTI_Config(void)
//{
//	EXTI_Config();
//	NVIC_Config();
//	EXTI_ClearFlag(EXTI_Line0);
//	EXTI_ClearFlag(EXTI_Line3);
//	EXTI_ClearFlag(EXTI_Line4);
//}


//uint16_t ccr_val = 100;//5%的CCR，起始状态
//int add_val = 0;//每次按键CCR增加的值
//int a =0;
////实际设置写入的CCR=ccr_val+add_val

///**  按键中断服务函数
//  *  功能：根据不同的按键设置不同的CCR的值
//  *  入口参数：无
//  *  返回值：无
//  */
////加速
//void EXTI3_IRQHandler(void)
//{
//	delay_ms(10);
//	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
//	{
//		if(KEY1 == 1)
//		{
//			add_val += 5;
//			LED1 = !LED1;
//			if(add_val<81)//最高允许加速到80%
//			{	
//				MOT_Control(ccr_val+add_val,ccr_val+add_val,ccr_val+add_val,ccr_val+add_val);	
//				printf("3CCR = %d\r\n",TIM_GetCapture1(TIM4));
//			}
//			else		
//			{
//				BEEP;
//				add_val = 80;
//			}				
//		}
//		EXTI_ClearITPendingBit(EXTI_Line3);
//	}
//}
////减速
//void EXTI4_IRQHandler(void)
//{
//	delay_ms(10);
//	if(EXTI_GetITStatus(EXTI_Line4) != RESET)
//	if(KEY0 == 1)
//	{
//		add_val -=5;
//		printf("%d\r\n",add_val);
//		LED0 = !LED0;
//		if(add_val > -1)
//		{
//			MOT_Control(ccr_val+add_val,ccr_val+add_val,ccr_val+add_val,ccr_val+add_val);
//			printf("4CCR = %d\r\n",TIM_GetCapture1(TIM4));			
//		}
//		else
//		{	
//			BEEP;
//			add_val = 0;
//			printf("add_val已归零,add_val = %d\r\n",add_val);
//		}
//	}	
//	EXTI_ClearITPendingBit(EXTI_Line4);
//}
////停止
//void EXTI0_IRQHandler(void)
//{
//	delay_ms(10);
//	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
//	{	
//		if(KEY1 == 1)
//		{
//			BEEP = !BEEP;
//			MOT_Control(ccr_val,ccr_val,ccr_val,ccr_val);
//			printf("0CCR = %d\r\n",TIM_GetCapture1(TIM4));
//		}
//		printf("刹车成功\r\n");
//		EXTI_ClearITPendingBit(EXTI_Line0);
//	}
//}
