#include "rs232.h"
#include <string.h>
	 
//½ÓÊÕ»º´æÇø 	
u8 RS232_RX_BUF=0xff;  	//½ÓÊÕ»º³
u8 usart2_RX_BUF[1];
u8 RS232_RX_CNT=0;
  		  
/*******************************************************************
*	¹¦ÄÜ£ºÖÐ¶Ï·½Ê½½ÓÊÕÊý¾Ý
*******************************************************************/  
void USART1_IRQHandler(void)
{
 	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //½ÓÊÕµ½Êý¾Ý	   
	{	 
	 	RS232_RX_BUF =USART_ReceiveData(USART1); 	//¶ÁÈ¡½ÓÊÕµ½µÄÊý¾Ý
		RS232_RX_CNT=1;	
	}										 
} 
void USART2_IRQHandler(void)
{
 	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //½ÓÊÕµ½Êý¾Ý	   
	{	 
	 	usart2_RX_BUF[0] =USART_ReceiveData(USART2); 	//¶ÁÈ¡½ÓÊÕµ½µÄÊý¾Ý
		RS232_RX_CNT=1;	
	}										 
} 
			
										 
/********************************************************************
*	¹¦ÄÜ£ºRS232³õÊ¼»¯
*	²ÎÊý£ºbound ²¨ÌØÂÊ
********************************************************************/  
void usart1_Init(u32 BaudRate)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	    //TX ÍÆÃâÊä³ö

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	   //RX ÉÏÀ­ÊäÈë
 

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1,ENABLE);//¸´Î»´®¿Ú1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1,DISABLE);//Í£Ö¹¸´Î»
 
	USART_InitStructure.USART_BaudRate = BaudRate;//Ò»°ãÉèÖÃÎª9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8Î»Êý¾Ý³¤¶È
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//Ò»¸öÍ£Ö¹Î»
	USART_InitStructure.USART_Parity = USART_Parity_No;///ÆæÅ¼Ð£ÑéÎ»
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ÎÞÓ²¼þÊý¾ÝÁ÷¿ØÖÆ
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//ÊÕ·¢Ä£Ê½

  USART_Init(USART1, &USART_InitStructure); //³õÊ¼»¯´®¿Ú1

  
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; //Ê¹ÄÜ´®¿Ú3ÖÐ¶Ï
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //ÏÈÕ¼ÓÅÏÈ¼¶2¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //´ÓÓÅÏÈ¼¶2¼¶
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //Ê¹ÄÜÍâ²¿ÖÐ¶ÏÍ¨µÀ
	NVIC_Init(&NVIC_InitStructure); //¸ù¾ÝNVIC_InitStructÖÐÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯ÍâÉèNVIC¼Ä´æÆ÷
 
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//¿ªÆôÖÐ¶Ï
 
	USART_Cmd(USART1, ENABLE);                    //Ê¹ÄÜ´®¿Ú 

}

void usart2_Init(u32 BaudRate)
{  
    GPIO_InitTypeDef GPIO_InitStructure;
  	USART_InitTypeDef USART_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
 

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,ENABLE);//¸´Î»´®¿Ú1
	RCC_APB2PeriphResetCmd(RCC_APB1Periph_USART2,DISABLE);//Í£Ö¹¸´Î»
 
	USART_InitStructure.USART_BaudRate = BaudRate;//Ò»°ãÉèÖÃÎª9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8Î»Êý¾Ý³¤¶È
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//Ò»¸öÍ£Ö¹Î»
	USART_InitStructure.USART_Parity = USART_Parity_No;///ÆæÅ¼Ð£ÑéÎ»
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ÎÞÓ²¼þÊý¾ÝÁ÷¿ØÖÆ
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//ÊÕ·¢Ä£Ê½

    USART_Init(USART2, &USART_InitStructure); //³õÊ¼»¯´®¿Ú1

  
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //Ê¹ÄÜ´®¿Ú3ÖÐ¶Ï
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //ÏÈÕ¼ÓÅÏÈ¼¶2¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //´ÓÓÅÏÈ¼¶2¼¶
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //Ê¹ÄÜÍâ²¿ÖÐ¶ÏÍ¨µÀ
	NVIC_Init(&NVIC_InitStructure); //¸ù¾ÝNVIC_InitStructÖÐÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯ÍâÉèNVIC¼Ä´æÆ÷
 
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//¿ªÆôÖÐ¶Ï
   
    USART_Cmd(USART2, ENABLE);                    //Ê¹ÄÜ´®¿Ú 
 
}


/**********************************************************
*	¹¦ÄÜ£º·¢ËÍÊý¾Ý
*	²ÎÊý£ºbuf ·¢ËÍ»º³åÇøÊ×µØÖ·
*		  len ´ý·¢ËÍµÄ×Ö½ÚÊý
**********************************************************/
void usart1_Send_Data(u8 *tdata)
{
	u8 t;
	u8 length;
	length=strlen(tdata);
  	for(t=0;t<length;t++)		//Ñ­»··¢ËÍÊý¾Ý
	{		   
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	//·¢ËÍÍê³É  
		USART_SendData(USART1,tdata[t]);
	}	 
 
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);			  
}

int fputc(int ch, FILE *f)
{ 
   USART_SendData(USART2, (u8) ch); 
   while(!(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == SET))
   {
   }
   return ch;
}
