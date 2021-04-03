#ifndef __CONTROL_H
#define __CONTROL_H

#include "sys.h"
#include "Struct_all.h"


void Control_Gyro(struct _SI_float *gyro,struct _Rc *rc,uint8_t Lock);//内环
void Control_Angle(struct _out_angle *angle,struct _Rc *rc);//外环
void DataOutput_ToMOT(uint8_t rc_lock);




#endif /*__CONTROL_H*/



