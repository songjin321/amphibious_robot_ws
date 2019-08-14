#ifndef __GAIT_H
#define	__GAIT_H

#include "stm32f10x.h"
#include "stm32f10x_can.h"

/****************************************
*	功能：步态封装
****************************************/


void Tripod_gait(u8 speed);
void Hexapod_gait(u8 speed);
void Tetrapod_gait(void);
void Turning_gait(u8 direction);
void Back_gait(void);
void Climbing_gait(void);
void Climbing_gait2(void);
void stair_gait(void);
void stair_init(void);
void Swim_common_gait(u16 speed);
void Swim_backcommon_gait(void);
void Swim_turncommon_gait(void);
void Swim_ascendcommon_gait(void);
void Swim_descendcommon_gait(void);

//extern __IO u8  count_flag;		  //velocityflag=1表示慢速，=2表示快速
/*****************************************/   

#endif
