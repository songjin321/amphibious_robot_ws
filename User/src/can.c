#include "can.h" 


__IO uint32_t rflag1 = 0xff;		 //用于标志是否接收到数据，在中断函数中赋值
__IO uint32_t rflag2 = 0xff;
__IO uint32_t rflag3 = 0xff;
__IO uint32_t rflag4 = 0xff;
__IO uint32_t rflag5 = 0xff;
__IO uint32_t rflag6 = 0xff;
__IO uint32_t rCOB_ID=0;
__IO uint8_t rCOB_TYPE=0;
__IO uint32_t rDATA_TYPE=0;
__IO uint8_t rNOD_ID=0;
CanTxMsg TxMessage;			     //发送缓冲区
CanRxMsg RxMessage;				 //接收缓冲区

/***************************************************
*	功能：CAN相关的IO的初始化
***************************************************/
static void CAN_GPIO_Config(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
   	
  	/*外设时钟设置*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);

  	/*IO设置*/
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
	/* Configure CAN pin: RX */									               // PB8
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	             // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* Configure CAN pin: TX */									               // PB9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		         // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
	GPIO_Init(GPIOB, &GPIO_InitStructure);

   	
  
	
}

/*******************************************************
*	功能：CAN接收中断优先级配置
*******************************************************/
static void CAN_NVIC_Config(void)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
		/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	 	/*中断设置*/
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	   //CAN1 RX0中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	 //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	//子优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


}

/*********************************************************************
*	功能：CAN控制器初始化
*********************************************************************/
static void CAN_Mode_Config(void)
{
   	CAN_InitTypeDef        CAN_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	/*CAN寄存器初始化*/
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	/*CAN单元初始化*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  关闭时间触发通信模式使能
	CAN_InitStructure.CAN_ABOM=ENABLE;			   //MCR-ABOM  自动离线管理 
	CAN_InitStructure.CAN_AWUM=ENABLE;			   //MCR-AWUM  使用自动唤醒模式
	CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
	CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
	CAN_InitStructure.CAN_TXFP=ENABLE;			   //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //正常工作模式
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW 重新同步跳跃宽度 1个时间单元
	CAN_InitStructure.CAN_BS1=CAN_BS1_5tq;		   //BTR-TS1 时间段1 占用了12个时间单元
	CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;		   //BTR-TS1 时间段2 占用了3个时间单元
	CAN_InitStructure.CAN_Prescaler =9;		   ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 36/(1+5+2)/9=0.5Mbps
	CAN_Init(CAN1, &CAN_InitStructure);
}

/********************************************************
*	功能：CAN过滤器配置
********************************************************/
static void CAN_Filter_Config(void)
{
   CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/*CAN过滤器初始化*/
	CAN_FilterInitStructure.CAN_FilterNumber=0;		//过滤器组0
   	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//工作在标识符屏蔽位模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//过滤器位宽为单个32位。
	/* 使能报文标示符过滤器按照标示符的内容进行比对过滤，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */

	CAN_FilterInitStructure.CAN_FilterIdHigh= (((u32)0x1314<<3)&0xFFFF0000)>>16;	//要过滤的ID高位 
   	CAN_FilterInitStructure.CAN_FilterIdLow= (((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //要过滤的ID低位 
   	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0000;			//过滤器高16位每位不须匹配
   	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;			//过滤器低16位每位不须匹配
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;		//过滤器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能过滤器
	CAN_FilterInit(&CAN_FilterInitStructure);
	/*CAN通信中断使能*/
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}


/****************************************
*	功能：CAN初始化
****************************************/
void CAN_Config(void)
{
  CAN_GPIO_Config();
  CAN_NVIC_Config();
  CAN_Mode_Config();
  CAN_Filter_Config();   
}


/****************************************
*	功能：设置报文内容
****************************************/  
void CAN_SetMsg(u32 Data_H, u32 Data_L, u16 tCOB_ID)
{
//	flag=0xff;	  
	TxMessage.StdId=tCOB_ID;			 //使用的标准ID
	TxMessage.IDE=CAN_ID_STD;			 //标准模式
	TxMessage.RTR=CAN_RTR_DATA;			 //发送的是数据
	TxMessage.DLC=8;				 //数据长度为8字节
	TxMessage.Data[0]=(uint8_t)0xFF &  Data_L;
	TxMessage.Data[1]=(uint8_t)0xFF & (Data_L>>8);
	TxMessage.Data[2]=(uint8_t)0xFF & (Data_L>>16);
	TxMessage.Data[3]=(uint8_t)0xFF & (Data_L>>24);
	TxMessage.Data[4]=(uint8_t)0xFF &  Data_H;
	TxMessage.Data[5]=(uint8_t)0xFF & (Data_H>>8);
	TxMessage.Data[6]=(uint8_t)0xFF & (Data_H>>16);
	TxMessage.Data[7]=(uint8_t)0xFF & (Data_H>>24);
}

/******************************************************
*	功能：数据接收中断
******************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
 
 /*从邮箱中读出报文*/
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
  
    //flag = 0;
    rCOB_ID=  RxMessage.StdId;
    rCOB_TYPE=  0x0f & (RxMessage.StdId>>7);
    rNOD_ID= (uint8_t)0x7f & rCOB_ID;		       //接收成功
    rDATA_TYPE=RxMessage.Data[0] |RxMessage.Data[1]<<8 | RxMessage.Data[2]<<16;
    if(rDATA_TYPE==0x604060) return;
    switch (rNOD_ID)
    {
    	case 1:  
		rflag1 = 0;
		break;
	case 2:
		rflag2 = 0;
		break;
	case 3:
		rflag3 = 0;
		break;
	case 4:
		rflag4 = 0;
		break;
	case 5:
		rflag5 = 0;
		break;
	case 6:
		rflag6 = 0;
		break;
	default: 
		break;
    }  
    CAN_FIFORelease(CAN1,CAN_FIFO0);
}
/**************************END OF FILE************************************/

