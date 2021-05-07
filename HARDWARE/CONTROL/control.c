#include "control.h"
#include "Rc.h"
#include "bsp_motor_tim4.h"
#include "Struct_all.h"
/*****************************用于处理遥控器的命令和PID输出的综合控制程序**********************************
*2021.2.17 完成没有PID干预的遥控器四通道（前后，左右，油门，转向）的综合控制,采用数据帧不排序直接处理
*
*
*
*
*1号电机（前左）= 悬停油门 + 右倾的量 - 前飞的量 - 旋转的量
*2号电机（前右）= 悬停油门 - 右倾的量 - 前飞的量 + 旋转的量
*3号电机（后左）= 悬停油门 + 右倾的量 + 前飞的量 + 旋转的量
*4号电机（后右）= 悬停油门 - 右倾的量 + 前飞的量 - 旋转的量
**************/

/**  
  *  功能：增量式离散PID
  *  入口参数：
  *  返回值：
  */
void PID_Control(struct _pid *PID,uint16_t target,float measure,float max,float Integral_max)
{
	PID->Error = target - measure;
	PID->Deriv = PID->Error - PID->PreError;
	PID->PreError = PID->Error;
	
	//PID限幅
	if(PID->Error > max)
		PID->Integral += max;
	if(PID->Error < -max)
		PID->Integral += -max;
	else
		PID->Integral += PID->Error;	
	if(PID->Integral > Integral_max)
	   PID->Integral = Integral_max;
	if(PID->Integral < -Integral_max)
	   PID->Integral = -Integral_max;
	
	if(Rc.THROTTLE <= 1300)//油门较小时积分清零
	{
		PID->Integral=0;
	}
	PID->Output = PID->kp * PID->Error + PID->ki *PID->Integral +PID->kd * PID->Deriv;

}

/**  
  *  功能：电机控制量计算
  *  入口参数：
  *  返回值：
  */
static uint16_t MOT_Compute(uint16_t throttle,int p,int r,int y)
{
	return (throttle + p*(int)(gyro_pitch.Output-1500) + r*(int)(gyro_roll.Output-1500) + y*(int)(gyro_yaw.Output-1500));
}

const float angle_max = 10;
const float angle_integral_max = 1000;
/**  
  *  功能：PID外环控制器
  *  入口参数：
  *  返回值：acc_pitch.Output,acc_roll.Output,acc_yaw.Output
  */
void Control_Angle(void)
{
	PID_Control(&acc_pitch, out_angle.pitch, ((Rc.PITCH-1500)/13.0f), angle_max, angle_integral_max);
	PID_Control(&acc_roll,  out_angle.roll,  ((Rc.ROLL-1500)/13.0f),  angle_max, angle_integral_max);
	PID_Control(&acc_yaw,   out_angle.yaw,   ((Rc.YAW-1500)/13.0f),   angle_max, angle_integral_max);
}


const float gyro_max = 50;
const float gyro_integral_max = 5000;
/**  
  *  功能：PID内环控制器
  *  入口参数：Rc_LOCK
  *  返回值：
  */
void Control_Gyro(uint8_t Lock)
{
	PID_Control(&gyro_pitch, acc_pitch.Output, (SI_gyro.x*Radian_to_Angle), gyro_max, gyro_integral_max);
	PID_Control(&gyro_roll,	 acc_roll.Output,  (SI_gyro.y*Radian_to_Angle), gyro_max, gyro_integral_max);
	PID_Control(&gyro_yaw,	 acc_yaw.Output,   (SI_gyro.z*Radian_to_Angle), gyro_max, gyro_integral_max);
	
	if(Lock==0)
	{
		Rc.mot1 = MOT_Compute(Rc.THROTTLE,1,1,-1);
		Rc.mot2 = MOT_Compute(Rc.THROTTLE,1,-1,1);
		Rc.mot3 = MOT_Compute(Rc.THROTTLE,-1,-1,-1);
		Rc.mot4 = MOT_Compute(Rc.THROTTLE,-1,1,1);
				
	}
	else
	{
		Rc.mot1=0;
		Rc.mot2=0;
		Rc.mot3=0;
		Rc.mot4=0;
	}
	MOT_Control(Rc.mot1,Rc.mot2,Rc.mot3,Rc.mot4);
}


