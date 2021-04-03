#include "protocol.h"

/**  
  *  功能：串口发送一个字符串
  *  入口参数：c，发送的字符
  *  返回值：无
  */
void usart1_send_char(uint8_t c)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
	USART_SendData(USART1,c);
}

/**  
  *  功能：发送数据给匿名上位机（V2.6）
  *  入口参数：fun,功能字，0xA0~0xAF
  *			   data，数据缓存区，最多28个字节
  *			   len，data数据长度
  *  返回值：无
  *  注：数据格式：0x88+FUN+LEN+DATA+SUM
  */
void usart1_niming_report(uint8_t fun,uint8_t *data,uint8_t len)
{
	uint8_t send_buf[32]={0x00};
	uint8_t i;
	if(len>28) return;//超过28个字节，无效
	send_buf[len+3]=0;//校验位置零
	send_buf[0]=0x88;//帧头0x88
	send_buf[1]=fun;//命令帧FUN
	send_buf[2]=len;//数据长度帧LEN
	for(i=0;i<len;i++)
		send_buf[i+3]=data[i];
	for(i=0;i<len+3;i++)
		send_buf[len+3] += send_buf[i];//计算数据校验位SUM
	for(i=0;i<len+4;i++)
		usart1_send_char(send_buf[i]);//发送数据到串口1
}	
/**  自定义帧0xA1
  *  功能：发送加速度传感器和陀螺仪传感器数据给匿名上位机（V2.6）
  *  入口参数：aacx,aacy,aacz：xyz三个方向的加速度值
  *			   gyrox,gyroy,gyroz：xyz三个方向的陀螺仪值		  
  *  返回值：无
  *  注：数据格式：0x88+FUN+LEN+DATA+SUM
  */
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	uint8_t buf[12];
	buf[0]=(aacx>>8)&0xFF;
	buf[1]=aacx&0xFF;
	buf[2]=(aacy>>8)&0xFF;
	buf[3]=aacy&0xFF;
	buf[4]=(aacz>>8)&0xFF;
	buf[5]=aacz&0xFF;
	
	buf[6]=(gyrox>>8)&0xFF;
	buf[7]=gyrox&0xFF;
	buf[8]=(gyroy>>8)&0xFF;
	buf[9]=gyroy&0xFF;
	buf[10]=(gyroz>>8)&0xFF;
	buf[11]=gyroz&0xFF;
	
	usart1_niming_report(0xA1,buf,12);
}

/**  飞控显示帧
  *  功能：上报解算后的姿态数据给上位机
  *  入口参数：aacx,aacy,aacz：xyz三个方向的加速度值
  *			   gyrox,gyroy,gyroz：xyz三个方向的陀螺仪值		
  *			   yaw,偏航角，单位为0.1度 0 -> 3600  对应 0 -> 360.0度
  *			   roll,横滚角，单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
  *			   pitch,俯仰角，单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
  *  返回值：无
  *  注：数据格式：0x88+0xAF+0x1C+ ACC DATA + GYRO DATA + MAG DATA + ANGLE DATA(roll/pitch/yaw) +0x00+0x00+0x00+0x00+SUM
  *		 ANGLE的roll和pitch数据为实际值乘以100以后得到的整数值，yaw为乘以10以后得到的整数值
  */
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	uint8_t buf[28]={0x00};
	
	buf[0]=(aacx>>8)&0xFF;
	buf[1]=aacx&0xFF;
	buf[2]=(aacy>>8)&0xFF;
	buf[3]=aacy&0xFF;
	buf[4]=(aacz>>8)&0xFF;
	buf[5]=aacz&0xFF;
	
	buf[6]=(gyrox>>8)&0xFF;
	buf[7]=gyrox&0xFF;
	buf[8]=(gyroy>>8)&0xFF;
	buf[9]=gyroy&0xFF;
	buf[10]=(gyroz>>8)&0xFF;
	buf[11]=gyroz&0xFF;
	
	//12-17为磁力计，MPU6050没有磁力计，发送0x00
	
	buf[18]=(roll>>8)&0xFF;
	buf[19]=roll&0xFF;
	buf[20]=(pitch>>8)&0xFF;
	buf[21]=pitch&0xFF;
	buf[22]=(yaw>>8)&0xFF;
	buf[23]=yaw&0xFF;
	
	//24-27为数据格式中的0x00
	
	usart1_niming_report(0xAF,buf,28);
}

/**  
  *  功能：发送PID数据给上位机
  *  入口参数：rol_p,rol_i,rol_d,pit_p，pit_i,pit_d,yaw_p,yaw_i,yaw_d
  *  返回值：无
  *	 格式为：0X88 0XAC 0X1C 0XAD + PID数据 + 无用数据 + SUM
  */
void usart1_report_pid(uint16_t rol_p,uint16_t rol_i,uint16_t rol_d,uint16_t pit_p,uint16_t pit_i,uint16_t pit_d,uint16_t yaw_p,uint16_t yaw_i,uint16_t yaw_d)
{
	uint8_t buf[28]={0x00};
	
	buf[0]=0xAD;
	
	buf[1]=(rol_p>>8)&0xFF;
	buf[2]=rol_p&0xFF;
	buf[3]=(rol_i>>8)&0xFF;
	buf[4]=rol_i&0xFF;
	buf[5]=(rol_d>>8)&0xFF;
	buf[6]=rol_d&0xFF;
	
	buf[7]=(pit_p>>8)&0xFF;
	buf[8]=pit_p&0xFF;
	buf[9]=(pit_i>>8)&0xFF;
	buf[10]=pit_i&0xFF;
	buf[11]=(pit_d>>8)&0xFF;
	buf[12]=pit_d&0xFF;
	
	buf[13]=(yaw_p>>8)&0xFF;
	buf[14]=yaw_p&0xFF;
	buf[15]=(yaw_i>>8)&0xFF;
	buf[16]=yaw_i&0xFF;
	buf[17]=(yaw_d>>8)&0xFF;
	buf[18]=yaw_d&0xFF;
	
	usart1_niming_report(0xAC,buf,28);
		
}

/**  
  *  功能：遥控,电机pwm,电压显示
  *  入口参数：throt,yaw,roll,pitch,aux1,aux2,aux3,aux4,aux5,pwm1,pwm2,pwm3,pwm4,vol(电压)
  *  返回值：无
  *  帧格式：0x88+0xAE+0x12+THROT YAW ROLL PITCH AUX1 2 3 4 5 PWM:1 2 3 4 VOLTAGE+SUM
	 遥控数据最小在1000左右，最大在2000左右。数据都为uint16格式,
	 其中pwm范围1-100,voltage为实际值*100。
  */
void usart1_report_rc(short thort,short yaw,short roll,short pitch,
						short aux1,short aux2,short aux3,short aux4,short aux5,
							short pwm1,short pwm2,short pwm3,short pwm4,
								short vol)
{
	uint8_t buf[28]={0x00};
	
	//THROT YAW ROLL PITCH	
	buf[0]=(thort>>8)&0xFF;//
	buf[1]=thort&0xFF;//
	buf[2]=(yaw>>8)&0xFF;//
	buf[3]=yaw&0xFF;//
	buf[4]=(roll>>8)&0xFF;//
	buf[5]=roll&0xFF;	//
	buf[6]=(pitch>>8)&0xFF;//
	buf[7]=pitch&0xFF;//
	
	//AUX1 2 3 4 5
	buf[8]=(aux1>>8)&0xFF;
	buf[9]=aux1&0xFF;
	buf[10]=(aux2>>8)&0xFF;
	buf[11]=aux2&0xFF;
	buf[12]=(aux3>>8)&0xFF;
	buf[13]=aux3&0xFF;
	buf[14]=(aux4>>8)&0xFF;
	buf[15]=aux4&0xFF;
	buf[16]=(aux5>>8)&0xFF;
	buf[17]=aux5&0xFF;	
	
	//PWM:1 2 3 4 
	buf[18]=(pwm1>>8)&0xFF;
	buf[19]=pwm1&0xFF;
	buf[20]=(pwm2>>8)&0xFF;
	buf[21]=pwm2&0xFF;
	buf[22]=(pwm3>>8)&0xFF;
	buf[23]=pwm3&0xFF;
	buf[24]=(pwm4>>8)&0xFF;
	buf[25]=pwm4&0xFF;
	
	//VOLTAGE
	buf[26]=(vol>>8)&0xFF;
	buf[27]=vol&0xFF;
	
	usart1_niming_report(0xAE,buf,28);
}

/**  
  *  功能：发送offset给上位机
  *  入口参数：acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z
  *  返回值：无
  *  格式为：0X88 0XAC 0X1C 0XAC + 传感器零偏数据ACC XYZ GYRO XYZ+无用数据+SUM
  */
void usart1_report_offset(short acc_x,short acc_y,short acc_z,short gyro_x,short gyro_y,short gyro_z)
{
	uint8_t buf[28]={0x00};
	
	buf[0]=0xAC;
	
	buf[1]=(acc_x>>8)&0xFF;
	buf[2]=acc_x&0xFF;
	buf[3]=(acc_y>>8)&0xFF;
	buf[4]=acc_y&0xFF;
	buf[5]=(acc_z>>8)&0xFF;
	buf[6]=acc_z&0xFF;	
	
	buf[7]=(gyro_x>>8)&0xFF;
	buf[8]=gyro_x&0xFF;
	buf[9]=(gyro_y>>8)&0xFF;
	buf[10]=gyro_y&0xFF;
	buf[11]=(gyro_z>>8)&0xFF;
	buf[12]=gyro_z&0xFF;

	usart1_niming_report(0xAC,buf,28);
}



