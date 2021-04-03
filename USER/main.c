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
			if(report==1)//发送数据给上位机
			{
				usart1_report_imu(acc.x,acc.y,acc.z,gyro.x,gyro.y,gyro.z,(int)(out_angle.roll*100),(int)(out_angle.pitch*100),(int)(out_angle.yaw*10));
			}			
		}
		
		/*500Hz任务*/
		if(Count_2ms>=2)
		{
			Count_2ms=0;			
			Control_Gyro(&SI_gyro,&Rc,Rc_LOCK);//内环控制
		}
		
		/*250Hz任务*/
		if(Count_4ms>=4)
		{
			Count_4ms=0;			
//			Control_Angle(&out_angle,&Rc);//外环控制
		}
		
		/*20Hz遥控器接收PPM信号*/	
		if(Count_25ms>=25)
		{	
			Count_25ms=0;					
			PPM_DataArrange(PPM_Databuf);//PPM数据整理 存储在 Rc结构体 中
//			DataOutput_ToMOT(Rc_LOCK);
			if(report==1)//发送数据给上位机
			{				
//				usart1_report_pid(u16 rol_p,u16 rol_i,u16 rol_d,u16 pit_p,u16 pit_i,u16 pit_d,u16 yaw_p,u16 yaw_i,u16 yaw_d);
//				usart1_report_rc(Rc.THROTTLE,Rc.YAW,Rc.ROLL,Rc.PITCH,Rc.AUX1,Rc.AUX2,Rc.AUX3,Rc.AUX4,0,(int)((Rc.mot1-1000)/10),(int)((Rc.mot2-1000)/10),(int)((Rc.mot3-1000)/10),(int)((Rc.mot4-1000)/10),0);
			}
			
		}
		
		/*4Hz任务*/
		if(Count_LED>=250)
		{
			Count_LED=0;
//			Get_Distance();//测距模块						
			LED1=!LED1;//绿灯闪烁，程序正常运行
			
		}		
	}	 
}

