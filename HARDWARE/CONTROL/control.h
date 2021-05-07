#ifndef __CONTROL_H
#define __CONTROL_H

#include "sys.h"
#include "Struct_all.h"

static uint16_t MOT_Compute(uint16_t THROTTLE,int p,int r,int y);
void PID_Control(struct _pid *PID,uint16_t target,float measure,float max,float Integral_max);
void Control_Gyro(uint8_t Lock);//内环
void Control_Angle(void);//外环





#endif /*__CONTROL_H*/



