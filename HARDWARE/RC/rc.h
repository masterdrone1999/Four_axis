#ifndef __RC_H
#define __RC_H

#include "sys.h"
#include "Struct_all.h"

extern uint16_t PPM_Databuf[8];
static void PPM_Limit(uint16_t *data);
void PPM_Init(void);
void PPM_DataArrange(uint16_t *data);
void Lock_Rep_Ctrl(void);



#endif /*__RC_H*/

