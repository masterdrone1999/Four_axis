#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "sys.h"


void usart1_send_char(uint8_t c);
void usart1_niming_report(uint8_t fun,uint8_t *data,uint8_t len);
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz);
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw);
//ANGLE的roll和pitch数据为实际值乘以100以后得到的整数值，yaw为乘以10以后得到的整数值
void usart1_report_pid(uint16_t rol_p,uint16_t rol_i,uint16_t rol_d,uint16_t pit_p,uint16_t pit_i,uint16_t pit_d,uint16_t yaw_p,uint16_t yaw_i,uint16_t yaw_d);
void usart1_report_rc(short thort,short yaw,short roll,short pitch,short aux1,short aux2,short aux3,short aux4,short aux5,short pwm1,short pwm2,short pwm3,short pwm4,short vol);
//遥控数据最小在1000左右，最大在2000左右,其中pwm范围1-100,voltage为实际值*100。
void usart1_report_offset(short acc_x,short acc_y,short acc_z,short gyro_x,short gyro_y,short gyro_z);

void Report_FlyCtrl(void);


#endif /*__PROTOCOL_H*/




