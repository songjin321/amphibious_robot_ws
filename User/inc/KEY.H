#ifndef _KEY_H_
#define _KEY_H_
#include "stm32f10x.h"

#define S2 GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)
#define S3 GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6)
#define ReferencePosition GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)
typedef struct{
	volatile u8 PressedDowTimes;//按键被按的次数
	volatile u8 CheckLowTimes;//检测到低电平的次数
}Switch;

extern Switch switch2,switch3;



void KeyInit(void);//按键相关IO初始化



#endif
