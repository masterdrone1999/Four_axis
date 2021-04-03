#ifndef __STRUCT_ALL_H
#define __STRUCT_ALL_H

#include "sys.h"

/******************************************************************************
							全局变量声明
*******************************************************************************/ 
extern uint8_t Rc_LOCK;//电机锁，0解锁；1上锁
extern uint8_t report;//开启上传数据到上位机,0,关闭；1开启

#define Radian_to_Angle	   57.2957795f
#define RawData_to_Angle	0.0610351f	//以下参数对应2000度每秒
#define RawData_to_Radian	0.0010653f

/******************************************************************************
							结构体声明
*******************************************************************************/ 
/* MPU6050--加速度计结构体 */
struct _acc
{
	short x;
	short y;
	short z;
};
extern struct _acc acc;
extern struct _acc filter_acc;
extern struct _acc offset_acc;

/* MPU6050--陀螺仪结构体 */
struct _gyro
{
	short x;
	short y;
	short z;
};
extern struct _gyro gyro;
extern struct _gyro filter_gyro;
extern struct _gyro offset_gyro;

/* float结构体 */
struct _SI_float
{
	float x;
	float y;
	float z;
};
extern struct _SI_float SI_acc;	
extern struct _SI_float SI_gyro;

/* 姿态解算--角度值 */
struct _out_angle
{
	float yaw;
	float roll;
	float pitch;
};
extern struct _out_angle out_angle;

/* 遥控数据 */
struct _Rc
{	
	uint16_t ROLL;		//横滚，左右
	uint16_t PITCH;		//俯仰，前后	
	uint16_t THROTTLE;	//节流阀，油门
	uint16_t YAW;		//偏航，方向
	
	uint16_t AUX1;		//辅助通道1
	uint16_t AUX2;		//辅助通道2
	uint16_t AUX3;		//辅助通道3
	uint16_t AUX4;		//辅助通道4
	
	uint16_t mot1;
	uint16_t mot2;
	uint16_t mot3;
	uint16_t mot4;
};
extern struct _Rc Rc;

/* pid变量 */
struct _pid
{
	float kp;
	float ki;
	float kd;
	float integral;
	
	float output;
};
extern struct _pid pitch;
extern struct _pid roll;
extern struct _pid yaw;
extern struct _pid gyro_pitch;
extern struct _pid gyro_roll;
extern struct _pid gyro_yaw;


void PID_Reset(void);


#endif /*__STRUCT_ALL_H*/


