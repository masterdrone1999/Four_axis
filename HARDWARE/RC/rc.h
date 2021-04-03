#ifndef __RC_H
#define __RC_H

#include "sys.h"
#include "Struct_all.h"

extern uint16_t PPM_Databuf[8];

void PPM_Init(void);
void PPM_DataArrange(uint16_t *data);




#endif /*__RC_H*/

