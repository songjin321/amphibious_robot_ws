#ifndef __RS485_H
#define __RS485_H			 
#include "stm32f10x.h"
#include "stdio.h"

extern u8 RS232_RX_BUF;  	//���ջ���
extern u8 RS232_RX_CNT;
extern u8 usart2_RX_BUF[1];  	//���ջ���

//����봮���жϽ��գ��벻Ҫע�����º궨��
#define EN_USART3_RX 	1			//0,������;1,����.


/********************************************************************
*	���ܣ�RS232��ʼ��
*	������bound ������
********************************************************************/  
void usart1_Init(u32 BaudRate);
void usart2_Init(u32 BaudRate);

/**********************************************************
*	���ܣ���������
*	������buf ���ͻ������׵�ַ
*		  len �����͵��ֽ���
**********************************************************/
void usart1_Send_Data(u8 *tdata);
int fputc(int ch, FILE *f);


#endif	   
















