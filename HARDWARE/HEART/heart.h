#ifndef __HEART_H
#define __HEART_H

#include "sys.h"

void HEART_Init(void);



extern uint8_t Count_1ms;//1000HKz计时
extern uint8_t Count_2ms;//500HKz计时
extern uint8_t Count_4ms;//250HKz计时
extern uint8_t Count_25ms;//20Hz计时
extern uint16_t Count_LED;//飞行器警示灯


#endif /*__HEART_H*/



