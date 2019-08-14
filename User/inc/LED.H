#ifndef _LED_H_
#define _LED_H_
#include "stm32f10x.h"

enum{YES,NO};
//#define IsLed1On(x) (x)?GPIO_SetBits(GPIOA, GPIO_Pin_10):GPIO_ResetBits(GPIOA, GPIO_Pin_10);
//#define IsLed2On(x) (x)?GPIO_SetBits(GPIOA, GPIO_Pin_9):GPIO_ResetBits(GPIOA, GPIO_Pin_9);	
#define IsLed3On(x) (x)?GPIO_SetBits(GPIOA, GPIO_Pin_8):GPIO_ResetBits(GPIOA, GPIO_Pin_8);
#define IsLed4On(x) (x)?GPIO_SetBits(GPIOC, GPIO_Pin_7):GPIO_ResetBits(GPIOC, GPIO_Pin_7);

void LedInit(void);//LED相关IO初始化
void delay(void);
#endif
