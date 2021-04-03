#include "iic_mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "beep.h"
#include "usart.h"
#include "delay.h"
#include "Struct_all.h"



/**  
  *  功能：MPU初始化封装
  *  入口参数：
  *  返回值：
  */

void MPU6050_Init(void)
{
	MPU_Init();//自定义MPU初始化
	while(mpu_dmp_init())
		BEEP =! BEEP;//dmp库初始化		
}

/**  
  *  功能：MPU获取数据
  *  入口参数：结构体 &out_angle,&acc,&gyro
  *  返回值：
  */
void MPU_GetData(void)
{
	if(mpu_dmp_get_data()==0)
	{	
		Get_Radian(&gyro,&SI_gyro);
	}
}
/**  
  *  功能：MPU gyro的原始数据转化为弧度
  *  入口参数：结构体 &out_angle,&acc,&gyro
  *  返回值：
  */
void Get_Radian(struct _gyro *Gyro_in,struct _SI_float *Gyro_out)
{
	Gyro_out->x = (float)(Gyro_in->x * RawData_to_Radian);
	Gyro_out->y = (float)(Gyro_in->y * RawData_to_Radian);
	Gyro_out->z = (float)(Gyro_in->z * RawData_to_Radian);
}

/**  
  *  功能：IIC延时函数
  *  入口参数：无
  *  返回值：无
  */
void MPU_IIC_Delay(void)
{
	delay_us(2);
}
/**  
  *  功能：软模拟IIC引脚初始化 PB10.SCL  PB11.SDA
  *  入口参数：无
  *  返回值：无
  */
void MPU_IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_10 | GPIO_Pin_11);
	
}
/**  
  *  功能：IIC开始信号 SCL保持高电平，SDA从高电平跳变到低电平
  *  入口参数：无
  *  返回值：无
  */
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();
	MPU_IIC_SDA=1;
	MPU_IIC_SCL=1;
	MPU_IIC_Delay();
	MPU_IIC_SDA=0;
	MPU_IIC_Delay();
	MPU_IIC_SCL=0;//钳制IIC总线，准备发送或接受数据	
}
/**  
  *  功能：IIC结束信号 SCL保持高电平，SDA从低电平跳变到高电平
  *  入口参数：无
  *  返回值：无
  */	
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();
	MPU_IIC_SCL=0;
	MPU_IIC_SDA=0;
	MPU_IIC_Delay();
	MPU_IIC_SCL=1;
	MPU_IIC_SDA=1;
	MPU_IIC_Delay();
}
/**  
  *  功能：等待应答信号ACK
  *  入口参数：无
  *  返回值：0，接受应答成功；1，接受应答失败
  */
uint8_t MPU_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime = 0;
	MPU_SDA_IN();
	MPU_IIC_SDA=1;MPU_IIC_Delay();
	MPU_IIC_SCL=1;MPU_IIC_Delay();
	while(MPU_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_SCL=0;
	return 0;
}
/**  
  *  功能：产生ACK应答
  *  入口参数：无
  *  返回值：无
  */
void MPU_IIC_Ack(void)
{
	MPU_IIC_SCL=0;
	MPU_SDA_OUT();
	MPU_IIC_SDA=0;
	MPU_IIC_Delay();
	MPU_IIC_SCL=1;
	MPU_IIC_Delay();
	MPU_IIC_SCL=0;
}
/**  
  *  功能：产生非ACK应答
  *  入口参数：无
  *  返回值：无
  */
void MPU_IIC_NAck(void)
{
	MPU_IIC_SCL=0;
	MPU_SDA_OUT();
	MPU_IIC_SDA=1;
	MPU_IIC_Delay();
	MPU_IIC_SCL=1;
	MPU_IIC_Delay();
	MPU_IIC_SCL=0;
}
/**  
  *  功能：IIC发送一个字节（8 bit）
  *  入口参数：无
  *  返回值：返回从机有无应答。0，无应答；1，有应答
  */
void MPU_IIC_Send_Byte(uint8_t txd)
{
	uint8_t t;
	MPU_SDA_OUT();
	MPU_IIC_SCL=0;//拉低IIC总线时钟开始数据传输
	for(t=0;t<8;t++)
	{
		MPU_IIC_SDA=(txd&0x80)>>7;
		txd<<=1;
		MPU_IIC_SCL=1;
		MPU_IIC_Delay();
		MPU_IIC_SCL=0;
		MPU_IIC_Delay();
	}
}
/**  
  *  功能：IIC读取一个字节（8 bit）
  *  入口参数：ack，应答
  *  返回值：一个字节（8 bit）
  */
uint8_t MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();
	for(i=0;i<8;i++)
	{
		MPU_IIC_SCL=0;
		MPU_IIC_Delay();
		MPU_IIC_SCL=1;
		receive<<=1;
		if(MPU_READ_SDA)
			receive++;
		MPU_IIC_Delay();
	}
	if(!ack)
		MPU_IIC_NAck();//发送NACK
	else
		MPU_IIC_Ack();//发送ACK
	return receive;
}
/**  
  *  功能：IIC写一个字节
  *  入口参数：reg，寄存器地址；data，数据
  *  返回值：0，正常；1，错误
  */
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data)
{
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//ADDR左移一位+写操作
	if(MPU_IIC_Wait_Ack())//等待应答
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg);
	MPU_IIC_Wait_Ack();
	MPU_IIC_Send_Byte(data);
	if(MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Stop();
	return 0;
}
  
/**  
  *  功能：IIC读一个字节
  *  入口参数：reg，寄存器地址
  *  返回值：数据
  */
uint8_t MPU_Read_Byte(uint8_t reg)
{
	uint8_t res;
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//外设地址+写命令
	MPU_IIC_Wait_Ack();
	MPU_IIC_Send_Byte(reg);//寄存器地址
	MPU_IIC_Wait_Ack();
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);
	MPU_IIC_Wait_Ack();
	res = MPU_IIC_Read_Byte(0);
	MPU_IIC_Stop();
	return res;
}
/**  
  *  功能：IIC连续写
  *  入口参数：addr，外设地址；reg，寄存器地址；len，写入长度；buf，数据
  *  返回值：0，正常；1，错误
  */
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t i;
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|0);
	if(MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg);
	MPU_IIC_Wait_Ack();
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);
		if(MPU_IIC_Wait_Ack())
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_Stop();
	return 0;
}
/**  
  *  功能：IIC连续读
  *  入口参数：addr，外设地址；reg，寄存器地址；len，读取长度；buf，储存数据
  *  返回值：0,正常；1，错误
  */
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|0);
	if(MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg);
	MPU_IIC_Wait_Ack();
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|1);
	MPU_IIC_Wait_Ack();
	while(len)
	{
		if(len==1)
			*buf=MPU_IIC_Read_Byte(0);
		else		
			*buf=MPU_IIC_Read_Byte(1);		
		len--;
		buf++;
	}
	MPU_IIC_Stop();
	return 0;
}
/**  
  *  功能：初始化MPU6050
  *  入口参数：无
  *  返回值：0，成功；其他，错误
  */
uint8_t MPU_Init(void)
{
//	uint8_t res;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//禁止JTAG，使PA15变为普通IO
	
	MPU_AD0_CTRL = 0;//控制MPU6050的AD0为低电平，从机地址=0x68；
					 //					 高电平，		  0x69；
//	MPU_IIC_Init();
//	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0x80);//复位从机
//	delay_ms(100);
//	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0x00);//唤醒从机
//	MPU_Set_Gyro_Fsr(3);//陀螺仪传感器 +-2000dps
//	MPU_Set_Accel_Fsr(0);//加速度传感器 +-2g
//	MPU_Set_Rate(50);
//	MPU_Write_Byte(MPU_INT_EN_REG,0x00);//关闭所有中断
//	MPU_Write_Byte(MPU_USER_CTRL_REG,0x00);//IIC主模式关闭
//	MPU_Write_Byte(MPU_FIFO_EN_REG,0x00);//关闭FIFO
//	MPU_Write_Byte(MPU_INTBP_CFG_REG,0x80);//INT低电平有效
//	res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
//	if(res == MPU_ADDR)
//	{
//		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0x01);//设置CLKSEL PLL X轴为参考
//		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0x00);//加速度计和陀螺仪使能
//		MPU_Set_Rate(50);
//	}
//	else
//		return 1;
	return 0;
}
/**  
  *  功能：设置陀螺仪传感器满量程范围
  *  入口参数：fsr,0,+-250dps
  *				   1,+-500dps
  *				   2,+-1000dps
  *				   3,+-2000dps	
  *  返回值：0,成功；1，失败
  */
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);
}
/**  
  *  功能：设置加速度传感器满量程范围
  *  入口参数：fsr，0,+-2g
  *				    1,+-4g
  *				    2,+-8g
  *				    3,+-16g	
  *  返回值：0,成功；1，失败
  */
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);
}
/**  
  *  功能：设置数字低通滤波器
  *  入口参数：lpf，低通滤波器频率（Hz）
  *  返回值：0,成功；1，失败
  */
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	return MPU_Write_Byte(MPU_CFG_REG,data);
}
/**  
  *  功能：设置采样率（Fs=1KHz）
  *  入口参数：rate=[4,1000]Hz
  *  返回值：0,设置成功；1，设置失败
  */
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);
	return MPU_Set_LPF(rate/2);
}
/**  
  *  功能：获取MPU内部温度
  *  入口参数：无
  *  返回值：温度*100
  */
short MPU_Get_Temperature(void)
{
	uint8_t buf[2];
	short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf);
	raw=((uint16_t)buf[0]<<8)|buf[1];
	temp = 36.53+((double)raw)/340;
	return temp*100;
}
/**  
  *  功能：获得陀螺仪原始值
  *  入口参数：gx,gy,gz,陀螺仪XYZ轴原始数据（带符号）
  *  返回值：0，成功；1，失败
  */
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
	uint8_t buf[6],res;
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res == 0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];
		*gy=((uint16_t)buf[2]<<8)|buf[3];
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	}
	return res;
}
/**  
  *  功能：获得加速度原始值
  *  入口参数：ax,ay,az,加速度计XYZ的原始数据（）
  *  返回值：0，正常；1，错误
  */
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
	uint8_t buf[6],res;
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res ==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];
		*ay=((uint16_t)buf[2]<<8)|buf[3];
		*az=((uint16_t)buf[4]<<8)|buf[5];
	}
	return res;
}

