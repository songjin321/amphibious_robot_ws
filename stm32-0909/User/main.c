#include"RS232.H"
#include"SysTick.H"
#include"led.h"
#include "can.h"
#include "CPG.h"
#include "Drive.h"
#include "Drive_common.h"
#include "Drive_CPG.h"
#include "stdlib.h"
#include "gait.h"
#include "Timer.h"
#include "sampling.h"
int main()
{
	usart1_Init(9600);//���ߴ���
	usart2_Init(115200);//sd��ͨѶ
	SysTick_Init(72);//ʱ��
	init_sample();//����������ʼ��
	LedInit();//LED��
	CAN_Config();//CANͨѶ
	Driveinit();//elmo������ʼ��
	delay_ms(500);
	usart1_Send_Data(&RS232_RX_BUF);	
	Timer3Init(200);//��ʱ���жϳ�ʼ��
	while(1)
	{
		//IsLed4On(YES);
		if(RS232_RX_CNT==1 )
		{
			initgait();
			RS232_RX_CNT=0;
			Gaitflag_common=2;
			delay_ms(100);
			break;
		}
		delay_ms(100);
	}	//configure the gait to hexapod gait
	while(1)
	{
		if((RS232_RX_CNT==1 )&&((RS232_RX_BUF &0x10)==0x00))	 //��ͨģʽ
		{
			while(1)
			{
				switch (RS232_RX_BUF)
				{
					case 0x00:				 //ˮ������Ѳ��
						OpenSdcardFile("\r\nlow-speed swimming gait test data:\r\n");
						sample_flag=1;
						Swim_common_gait(0);//0
						break;
					case 0x01:
					    //Swim_backcommon_gait();		//ˮ�к���
						OpenSdcardFile("\r\nmiddle-speed swimming gait test data:\r\n");
						sample_flag=1;
						Swim_common_gait(1);//ˮ������Ѳ��
						break;
					case 0x02:
					    //Swim_turncommon_gait();		//ˮ��ת��
						OpenSdcardFile("\r\nhigh-speed swimming gait test data:\r\n");
						sample_flag=1;
						Swim_common_gait(2);//ˮ�п���Ѳ��
						break;
					case 0x03:
					    Swim_descendcommon_gait();	 //ˮ���½�
						break;
					case 0x04:
					    Swim_ascendcommon_gait();	 //ˮ������
						break;
					case 0x05:				   //����
						stair_gait();
						break;
					case 0x06:				   //����1  �ı������ʱ��
						Climbing_gait();
						break;
					case 0x07:				  //����
						Back_gait();
						break;
					case 0x08:
						Climbing_gait2();	   //����2	 �ı�Ӵ���
						break;
					case 0x09:
						Turning_gait(1); //��ת
						break;
					case 0x0A:
						Turning_gait(0);//0 ��ת
						break;
					case 0x0B:
						Tetrapod_gait();		   //����
						break;
					case 0x0C:
						OpenSdcardFile("\r\nhigh-speed hexapod gait test data:\r\n");
						sample_flag=1;
						Hexapod_gait(1);		   //�����
						break;
					case 0x0D:
						OpenSdcardFile("\r\nlow-speed hexapod gait test data:\r\n");
						sample_flag=1;
						Hexapod_gait(0);		   //������
						break;
					case 0x0E:
						OpenSdcardFile("\r\nhigh-speed tripod gait test data:\r\n");
						sample_flag=1;
						Tripod_gait(1);			   //�����
						break;
					case 0x0F:
						OpenSdcardFile("\r\nlow-speed tripod gait test data:\r\n");
						sample_flag=1;
						Tripod_gait(0);			   //������
						break;
					default:
						break;	
				}
				if(((RS232_RX_BUF &0x10)==0x10) && (RS232_RX_CNT==1))                         //ֹͣ
	   			{			
					Stop_common();
					sample_flag=0;
					CloseFile();
	   			}
			}
		}														



															   //��ͨģʽ����
       /* ---------------------------------------------------------------------------------*/
		if((RS232_RX_CNT==1 )&&((RS232_RX_BUF &0x10)==0x10))		 //CPGģʽ��������ʵ��
		{
	 		while(1)
	  		{
	  			switch(RS232_RX_BUF)
				{
					case 0x10:
//						Tripod_gait_flipperleg(1);
						sample_flag=1;
						break;
					case 0x11:
//						Tripod_gait_flipperleg(2);
						sample_flag=1;
						break;
					case 0x12:
	//					Tripod_gait_flipperleg(3);
						sample_flag=1;
						break;
					case 0x13:
//						Hexapod_gait_flipperleg(1);
						sample_flag=1;
						break;
					case 0x14:
//						Hexapod_gait_flipperleg(2);
						sample_flag=1;
						break;
					case 0x15:
//						Hexapod_gait_flipperleg(3);
						sample_flag=1;
						break;
					case 0x16:
//						Swim_common_gait_flipperleg();
						break;
					case 0x17:
						break;
					case 0x18:
						break;
					case 0x19:
						break;
					case 0x1A:
						break;
					case 0x1B:
						break;
					case 0x1C:
						Tri_gait_cpg();		 //cpg����
						break;					
					case 0x1D:
					    Tri_gait_cpg_fast();	 //cpg�����
						break;
					case 0x1E:
						Hex_gait_cpg();		   //cpg����
						break;
					case 0x1F:
						Hex_gait_cpg_fast();	//cpg�����
						break;
					default:
						break;
				}
				if(((RS232_RX_BUF &0x10)==0x00) && (RS232_RX_CNT==1))                  //ֹͣ
	   			{				
					Stop_common();
					sample_flag=0;
				//	stopsample();
	   			}
	 		 }
		}
 		delay_ms(100);														  //CPGģʽ����
 	}
 }														   //��while
															  //main����

		
