#ifndef __FLIPPERLEG_H
#define	__FLIPPERLEG_H

#include "stm32f10x.h"
#include "stm32f10x_can.h"

/****************************************
*	功能：步态封装
****************************************/

void Tripod_gait_flipperleg(u8 speed);
void Hexapod_gait_flipperleg(u8 speed);
void Swim_common_gait_flipperleg(void);

//extern __IO u8  count_flag;		  //velocityflag=1表示慢速，=2表示快速
/*****************************************/   

#endif
