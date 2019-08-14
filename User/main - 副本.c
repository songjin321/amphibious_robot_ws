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
	///**Driveinit();//elmo������ʼ��
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
	
						OpenSdcardFile("\r\nlow-speed swimming gait test data:\r\n");
						sample_flag=1;
						//**Swim_common_gait(0);//0
				sample_flag=0;
					CloseFile();
						
 }														   //��while
															  //main����

		
