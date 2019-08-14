#include "can.h" 


__IO uint32_t rflag1 = 0xff;		 //���ڱ�־�Ƿ���յ����ݣ����жϺ����и�ֵ
__IO uint32_t rflag2 = 0xff;
__IO uint32_t rflag3 = 0xff;
__IO uint32_t rflag4 = 0xff;
__IO uint32_t rflag5 = 0xff;
__IO uint32_t rflag6 = 0xff;
__IO uint32_t rCOB_ID=0;
__IO uint8_t rCOB_TYPE=0;
__IO uint32_t rDATA_TYPE=0;
__IO uint8_t rNOD_ID=0;
CanTxMsg TxMessage;			     //���ͻ�����
CanRxMsg RxMessage;				 //���ջ�����

/***************************************************
*	���ܣ�CAN��ص�IO�ĳ�ʼ��
***************************************************/
static void CAN_GPIO_Config(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
   	
  	/*����ʱ������*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);

  	/*IO����*/
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
	/* Configure CAN pin: RX */									               // PB8
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	             // ��������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* Configure CAN pin: TX */									               // PB9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		         // �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
	GPIO_Init(GPIOB, &GPIO_InitStructure);

   	
  
	
}

/*******************************************************
*	���ܣ�CAN�����ж����ȼ�����
*******************************************************/
static void CAN_NVIC_Config(void)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
		/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	 	/*�ж�����*/
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	   //CAN1 RX0�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	 //��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	//�����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


}

/*********************************************************************
*	���ܣ�CAN��������ʼ��
*********************************************************************/
static void CAN_Mode_Config(void)
{
   	CAN_InitTypeDef        CAN_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	/*CAN�Ĵ�����ʼ��*/
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	/*CAN��Ԫ��ʼ��*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  �ر�ʱ�䴥��ͨ��ģʽʹ��
	CAN_InitStructure.CAN_ABOM=ENABLE;			   //MCR-ABOM  �Զ����߹��� 
	CAN_InitStructure.CAN_AWUM=ENABLE;			   //MCR-AWUM  ʹ���Զ�����ģʽ
	CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  ��ֹ�����Զ��ش�	  DISABLE-�Զ��ش�
	CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  ����FIFO ����ģʽ  DISABLE-���ʱ�±��ĻḲ��ԭ�б���  
	CAN_InitStructure.CAN_TXFP=ENABLE;			   //MCR-TXFP  ����FIFO���ȼ� DISABLE-���ȼ�ȡ���ڱ��ı�ʾ�� 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //��������ģʽ
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW ����ͬ����Ծ��� 1��ʱ�䵥Ԫ
	CAN_InitStructure.CAN_BS1=CAN_BS1_5tq;		   //BTR-TS1 ʱ���1 ռ����12��ʱ�䵥Ԫ
	CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;		   //BTR-TS1 ʱ���2 ռ����3��ʱ�䵥Ԫ
	CAN_InitStructure.CAN_Prescaler =9;		   ////BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 36/(1+5+2)/9=0.5Mbps
	CAN_Init(CAN1, &CAN_InitStructure);
}

/********************************************************
*	���ܣ�CAN����������
********************************************************/
static void CAN_Filter_Config(void)
{
   CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/*CAN��������ʼ��*/
	CAN_FilterInitStructure.CAN_FilterNumber=0;		//��������0
   	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//�����ڱ�ʶ������λģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//������λ��Ϊ����32λ��
	/* ʹ�ܱ��ı�ʾ�����������ձ�ʾ�������ݽ��бȶԹ��ˣ���չID�������µľ����������ǵĻ��������FIFO0�� */

	CAN_FilterInitStructure.CAN_FilterIdHigh= (((u32)0x1314<<3)&0xFFFF0000)>>16;	//Ҫ���˵�ID��λ 
   	CAN_FilterInitStructure.CAN_FilterIdLow= (((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //Ҫ���˵�ID��λ 
   	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0000;			//��������16λÿλ����ƥ��
   	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;			//��������16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;		//��������������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//ʹ�ܹ�����
	CAN_FilterInit(&CAN_FilterInitStructure);
	/*CANͨ���ж�ʹ��*/
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}


/****************************************
*	���ܣ�CAN��ʼ��
****************************************/
void CAN_Config(void)
{
  CAN_GPIO_Config();
  CAN_NVIC_Config();
  CAN_Mode_Config();
  CAN_Filter_Config();   
}


/****************************************
*	���ܣ����ñ�������
****************************************/  
void CAN_SetMsg(u32 Data_H, u32 Data_L, u16 tCOB_ID)
{
//	flag=0xff;	  
	TxMessage.StdId=tCOB_ID;			 //ʹ�õı�׼ID
	TxMessage.IDE=CAN_ID_STD;			 //��׼ģʽ
	TxMessage.RTR=CAN_RTR_DATA;			 //���͵�������
	TxMessage.DLC=8;				 //���ݳ���Ϊ8�ֽ�
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
*	���ܣ����ݽ����ж�
******************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
 
 /*�������ж�������*/
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
  
    //flag = 0;
    rCOB_ID=  RxMessage.StdId;
    rCOB_TYPE=  0x0f & (RxMessage.StdId>>7);
    rNOD_ID= (uint8_t)0x7f & rCOB_ID;		       //���ճɹ�
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

