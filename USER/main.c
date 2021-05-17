#include "sys.h"
#include "heart.h"
#include "Struct_all.h"

#include "delay.h"
#include "led.h"
#include "usart.h"
#include "beep.h"

#include "bsp_motor_tim4.h"
#include "rc.h"
#include "control.h"

#include "iic_mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "protocol.h"

#include "hcsr04.h"

/**  
  *  功能：
  *  入口参数：
  *  返回值：
  */

int main(void)
{	
	delay_init();	    	 	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(5000000);	 
 	LED_Init();
	BEEP_Init();  
	hcsr04_Init();
	PPM_Init();
	PWM_Init();
	MPU6050_Init();
	PID_Reset();	
	HEART_Init();

 	while(1)
	{
		/*1000Hz任务*/
		if(Count_1ms>=1)
		{
			Count_1ms=0;			
			MPU_GetData();
		}
		
		/*500Hz任务*/
		if(Count_2ms>=2)
		{
			Count_2ms=0;			
			Control_Gyro(Rc_LOCK);//内环控制
		}
		
		/*250Hz任务*/
		if(Count_4ms>=4)
		{
			Count_4ms=0;			
			Control_Angle();//外环控制
		}
		
		/*20Hz任务*/	
		if(Count_50ms>=50)
		{	
			Count_50ms=0;					
			PPM_DataArrange(PPM_Databuf);//遥控器接收PPM信号，PPM数据整理，存储在 Rc结构体 中
			Lock_Rep_Ctrl();//判断Rc_LOCK和report
			if(0 == report)
				Report_FlyCtrl();//向上位机汇报			
		}
		
		/*5Hz任务*/
		if(Count_LED>=200)
		{
			Count_LED=0;
			Get_Distance();//测距模块						
			LED1=!LED1;//绿灯闪烁，程序正常运行			
		}		
	}	 
}

