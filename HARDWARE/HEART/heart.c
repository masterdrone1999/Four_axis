#include "heart.h"

/**  
  *  功能：飞行器任务管理函数，根据TIM7设定不同频率的飞控任务
  *  入口参数：无
  *  返回值：无
  *  已经设定最高频率为1KHz，飞行器任务分配为1000KHz，500KHz，250KHz
  */
void HEART_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);   //使能对应RCC时钟
	//配置定时器基础结构体
	TIM_DeInit(TIM7);
	TIM_TimeBaseStructure.TIM_Period = (1000-1); //设置在下一个更新事件装入活动的自动重装载寄存器周期的值,计数到1000为1ms
	TIM_TimeBaseStructure.TIM_Prescaler =(72-1); //设置用来作为TIMx时钟频率除数的预分频值,1M的计数频率 1US计数
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位         
								
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;             //选择串口1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //抢占式中断优先级设置为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //响应式中断优先级设置为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;        //使能中断
	NVIC_Init(&NVIC_InitStructure);
    
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);   //清除更新中断，免得一打开中断立即产生中断
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);    //打开定时器更新中断
    TIM_Cmd(TIM7,ENABLE);     
}



/**  
  *  功能： TIM7中断服务函数，控制执行不同频率的任务
  *  入口参数：无
  *  返回值：无
  */
extern uint8_t Count_1ms=0;//1000Hz计时
extern uint8_t Count_2ms=0;//500Hz计时
extern uint8_t Count_4ms=0;//250Hz计时
extern uint8_t Count_25ms=0;//20Hz计时
extern uint16_t Count_LED=0;//飞行器警示灯

void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		Count_1ms++;
		Count_2ms++;
		Count_4ms++;
		Count_25ms++;
		Count_LED++;		
	}
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);  //清除TIM7更新中断标志 
}










