#ifndef __CPG_H
#define	__CPG_H

#include "stm32f10x.h"
#include "stm32f10x_can.h"
#define TMIDxR_TXRQ  ((uint32_t)0x00000001)
#define TMIDxR_TXOK  ((uint32_t)0x00000001)
#define TMIDxR_RQCP  ((uint32_t)0x00000001)

/****************************************
*	功能：驱动初始化
****************************************/



void Runge_Kutta(void);
void hopf_func(float u1,float v1,float u2,float v2,float fre,float fai,unsigned int A);



extern float delta_h;

extern float hopf_U1[3],hopf_V1[3],hopf_U2[3],hopf_V2[3];
extern float hopf_DU1,hopf_DU2,hopf_DV1,hopf_DV2;

extern float hopf_f,hopf_fai,hopf_A;


/*****************************************/   

#endif
