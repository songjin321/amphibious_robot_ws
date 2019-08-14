#ifndef __CAN_H
#define	__CAN_H

#include "stm32f10x.h"
#include "stm32f10x_can.h"
extern   CanTxMsg TxMessage;
extern 	__IO uint32_t rflag1;
extern 	__IO uint32_t rflag2;
extern 	__IO uint32_t rflag3;
extern 	__IO uint32_t rflag4;
extern 	__IO uint32_t rflag5;
extern 	__IO uint32_t rflag6;

extern    __IO uint32_t rCOB_ID;
extern    __IO uint8_t rCOB_TYPE;
extern    __IO uint32_t rDATA_TYPE;
extern    __IO uint8_t rNOD_ID;
extern  CanRxMsg RxMessage;

/****************************************
*	功能：CAN初始化
****************************************/
void CAN_Config(void);

/****************************************
*	功能：设置报文内容
****************************************/   
void CAN_SetMsg(u32 Data_H,u32 Data_L, u16 tCOBID);

#endif
