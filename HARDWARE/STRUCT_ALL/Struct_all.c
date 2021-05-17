#include "Struct_all.h"

uint8_t Rc_LOCK=1;//电机锁，0解锁；1上锁
uint8_t report=1;//开启上传数据到上位机,0,关闭；1开启
uint16_t MOT_Speed[4]={0};//PWM量

struct _acc  acc;			//原始数据
struct _gyro gyro;

struct _acc  filter_acc;	//滤波后数据
struct _gyro filter_gyro;

struct _acc  offset_acc;	//零偏数据
struct _gyro offset_gyro;

struct _SI_float  SI_acc;	//加速度数据（m/s2）
struct _SI_float  SI_gyro;	//角速度数据（rad）

struct _Rc Rc;				//遥控通道
struct _out_angle out_angle;//姿态解算-角度值

/* pid */
struct _pid acc_pitch;
struct _pid acc_roll;
struct _pid acc_yaw;
struct _pid gyro_pitch;
struct _pid gyro_roll;
struct _pid gyro_yaw;

void PID_Reset(void)
{
//	roll.kp  = 4.9f;
//	roll.ki  = 0.02f;
//	roll.kd  = 8.0f;
//	pitch.kp = 4.9f;
//	pitch.ki = 0.02f;
//	pitch.kd = 8.0f;
//	yaw.kp = 4.9f;
//	yaw.ki = 0.02f;
//	yaw.kd = 8.0f;
	
	gyro_roll.kp  =1.0f;
	gyro_roll.ki  =0.0f;
	gyro_roll.kd  =0.0f;
	
	gyro_pitch.kp =1.0f;
	gyro_pitch.ki =0.0f;
	gyro_pitch.kd =0.0f;
	
	gyro_yaw.kp   =1.0f;
	gyro_yaw.ki   =0.0f;
	gyro_yaw.kd   =0.0f;
}





