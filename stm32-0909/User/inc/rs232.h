#ifndef __RS485_H
#define __RS485_H			 
#include "stm32f10x.h"
#include "stdio.h"

extern u8 RS232_RX_BUF;  	//接收缓冲
extern u8 RS232_RX_CNT;
extern u8 usart2_RX_BUF[1];  	//接收缓冲

//如果想串口中断接收，请不要注释以下宏定义
#define EN_USART3_RX 	1			//0,不接收;1,接收.


/********************************************************************
*	功能：RS232初始化
*	参数：bound 波特率
********************************************************************/  
void usart1_Init(u32 BaudRate);
void usart2_Init(u32 BaudRate);

/**********************************************************
*	功能：发送数据
*	参数：buf 发送缓冲区首地址
*		  len 待发送的字节数
**********************************************************/
void usart1_Send_Data(u8 *tdata);
int fputc(int ch, FILE *f);


#endif	   
















