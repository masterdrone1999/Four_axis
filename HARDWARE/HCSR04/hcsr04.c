#include "hcsr04.h"
#include "delay.h"
#include "beep.h"

/**  
  *  功能：超声波使用的引脚和定时器初始化 Trig-》PA5，Echo-》PA6
  *  入口参数：无
  *  返回值：无
  */
void hcsr04_Init(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;     //生成用于定时器设置的结构体
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);   //使能对应RCC时钟
    RCC_APB2PeriphClockCmd(HCSR04_CLK, ENABLE);
     
    GPIO_InitStructure.GPIO_Pin =HCSR04_TRIG;       //发送电平引脚
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
    GPIO_Init(HCSR04_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(HCSR04_PORT,HCSR04_TRIG);
     
    GPIO_InitStructure.GPIO_Pin = HCSR04_ECHO;     //返回电平引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(HCSR04_PORT, &GPIO_InitStructure);  
	GPIO_ResetBits(HCSR04_PORT,HCSR04_ECHO);    
				 
	//定时器初始化 使用基本定时器TIM3
	//配置定时器基础结构体
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF; 
	TIM_TimeBaseStructure.TIM_Prescaler =72-1; //设置用来作为TIMx时钟频率除数的预分频值,1M的计数频率 1个=1us
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位  

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;//不分频
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//将IC1映射到IT1
	TIM_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);
								
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;             //选择TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //抢占式中断优先级设置为3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         //响应式中断优先级设置为1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;        //使能中断
	NVIC_Init(&NVIC_InitStructure);
		
	TIM_ClearFlag(TIM3, TIM_FLAG_Update|TIM_FLAG_CC1);   //清除更新中断，免得一打开中断立即产生中断
	TIM_ITConfig(TIM3, TIM_IT_CC1,ENABLE);    //打开定时器更新中断
    TIM_Cmd(TIM3,ENABLE);       
}

/**  
  *  功能： 记录输入高电平时间
  *  入口参数：
  *  返回值：
  */
u8  TIM3CH1_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM3CH1_CAPTURE_VAL;	//输入捕获值
void TIM3_IRQHandler(void)
{
	if(0 == (TIM3CH1_CAPTURE_STA & 0x80))//未捕获成功
	{
		if(TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET) //再次检测是否有中断
		{	
			if(TIM3CH1_CAPTURE_STA & 0x40)//已经捕获到高电平
			{
				if(0x3F == (TIM3CH1_CAPTURE_STA & 0x3F))//到达计时最大值，不能再进行计数；所有数据附上最大值
				{	
					TIM3CH1_CAPTURE_STA |= 0x80;//将捕获状态位（bit8）设为1，标记成功捕获
					TIM3CH1_CAPTURE_VAL = 0xFFFF;
				}
				else 
				{
					TIM3CH1_CAPTURE_STA++;
				}
			}
			
		}
		if(TIM_GetITStatus(TIM3,TIM_IT_CC1) != RESET)
		{
			if(TIM3CH1_CAPTURE_STA & 0x40)//从第一个高电平开始，捕获到第一个下降沿
			{
				TIM3CH1_CAPTURE_STA |= 0x80;//置位bit8，标记为一次捕获结束
				TIM3CH1_CAPTURE_VAL = TIM_GetCapture1(TIM3);
				TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising);	
			}				
			else//第一个高电平刚到
			{	
				TIM3CH1_CAPTURE_STA = 0;
				TIM3CH1_CAPTURE_VAL = 0;
				TIM_SetCounter(TIM3,0);	//全部清空
				TIM3CH1_CAPTURE_STA |= 0x40;//标记捕获到上升沿
				TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Falling);		
			}
		}

	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_CC1 | TIM_IT_Update);
}
/**  
  *  功能： 发送开始信号
  *  入口参数：
  *  返回值：
  */
static void Send_Trig(void)
{
	PAout(5)=1;
	delay_us(15);
	PAout(5)=0;
	delay_ms(70);
}

/**  
  *  功能： 得到距离 t=Distance*0.000 001(s),s=340*t/2(m)
  *  入口参数：
  *  返回值：
  */
void Get_Distance(void)
{
	float Distance;
	Send_Trig();
	if(TIM3CH1_CAPTURE_STA&0x80)
	{
		Distance=TIM3CH1_CAPTURE_STA&0X3F;
		Distance*=65536;         //溢出时间总和
		Distance += TIM3CH1_CAPTURE_VAL;  //得到总的高电平时间   ECHO输出的高电平的时间就是超声波从发射到返回的时间	
		if(Distance<59)//成功降落，停止 Distance=58.823529 相当于 s<10cm
		{	
			TIM_Cmd(TIM6,DISABLE);
			BEEP=0;
			TIM_SetCounter(TIM6, 0);			
		}
		else if(Distance<588)//距离小于100cm，
		{			
			TIM_SetCounter(TIM6, 0);
			TIM6->ARR=2000-1;
			TIM_Cmd(TIM6,ENABLE);
		}
		else
		{
			TIM_Cmd(TIM6,DISABLE);
			BEEP=0;
			TIM_SetCounter(TIM6, 0);	
		}
	}		
		TIM3CH1_CAPTURE_STA=0;	
}











