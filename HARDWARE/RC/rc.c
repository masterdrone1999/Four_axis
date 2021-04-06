#include "Rc.h"
#include "led.h"
#include "usart.h"
#include "Struct_all.h"

/***********************富斯 I6X 遥控器数据帧*******************
*
*	DATA 0		DATA 1		DATA 2		DATA 3		DATA 4		DATA 5		DATA 6		DATA 7
*	前后		机头方向	辅助通道2	辅助通道4	左右		油门		辅助通道1	辅助通道3
*	ch2			ch4			ch6			ch8			ch1			ch3			ch5			ch7
*
****************/

/**  
  *  功能：PPM捕获初始化 TIM2_CH2 PA1
  *  入口参数：
  *  返回值：
  */

void PPM_Init(void)//使用PA1做为输入中断源
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitType;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  //下拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 设置TIM2CLK 为 72MHZ */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
	/* 自动重装载寄存器周期的值(计数值) */
	TIM_TimeBaseStructure.TIM_Period=20000;//20ms中断一次
	/* 累计 TIM_Period个频率后产生一个更新或者中断 */
	/* 时钟预分频数为72 */
	TIM_TimeBaseStructure.TIM_Prescaler=(72-1);//1us
	/* 对外部时钟进行采样的时钟分频,这里没有用到 */
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_ICInitType.TIM_Channel=TIM_Channel_2;//PA1 通道2
	TIM_ICInitType.TIM_ICFilter=0x0;
	TIM_ICInitType.TIM_ICPolarity=TIM_ICPolarity_Rising;//上升沿
	TIM_ICInitType.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	TIM_ICInitType.TIM_ICSelection=TIM_ICSelection_DirectTI;//捕获源选择
	TIM_ICInit(TIM2,&TIM_ICInitType);        
	                                                  
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;      
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);    

	TIM_ClearFlag(TIM2, TIM_FLAG_Update|TIM_IT_CC2);    
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC2,ENABLE);
	TIM_Cmd(TIM2, ENABLE);                                                                        
}


uint16_t PPM_Sample_Cnt=0;
uint32_t PPM_Time=0;
uint16_t PPM_Okay=0;
uint16_t PPM_Databuf[8]={0};
uint8_t TIM2_CH2_CAPTURE_STA=0;
/**  
  *  功能：TIM2_ch1中断服务函数，接收PPM信号储存在PPM_Databuf中
  *  入口参数：
  *  返回值：
  */
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_CC2)==!RESET)//捕获中断
	{
		if(TIM2_CH2_CAPTURE_STA&0x01)//符合条件的话说明上次捕获了高电平，那么这次捕获的一定是低电平
		{
			PPM_Time=TIM_GetCapture2(TIM2);
			if(PPM_Time>0)
				PPM_Time++;			
			if(PPM_Okay==1)
			{
				PPM_Databuf[PPM_Sample_Cnt]=PPM_Time;
				PPM_Sample_Cnt++;
				if(PPM_Sample_Cnt>8)
					PPM_Okay=0;
			}
			if(PPM_Time>7000)//识别到帧尾
			{
				PPM_Okay=1;
				PPM_Sample_Cnt=0;
			}    

				TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Rising);
				TIM2_CH2_CAPTURE_STA=0;//清掉标志位准备开始下一次上升沿和下降沿检测
		}
			else
			{
				TIM_SetCounter(TIM2,0);//以上为清零            
				TIM2_CH2_CAPTURE_STA|=0x01;//高电平指示被赋值
				TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Falling); //当捕获上升沿后改为捕获下降沿
			}        
	}
	
	TIM_ClearITPendingBit(TIM2, TIM_IT_CC2|TIM_IT_Update); //清除中断标志位，一定不要忘，要不然下次进不了中断    
}


/**  
  *  功能：将从RC来的PPM信号数据写入对应Rc结构体
  *  入口参数：
  *  返回值：
  */
void PPM_DataArrange(uint16_t *data)
{
	Rc.ROLL = data[4];
	Rc.PITCH = data[0];
	Rc.THROTTLE = data[5];
	Rc.YAW = data[1];
	Rc.AUX1 = data[6];
	Rc.AUX2 = data[2];
	Rc.AUX3 = data[7];
	Rc.AUX4 = data[3];
}


