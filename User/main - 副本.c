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
	usart1_Init(9600);//无线串口
	usart2_Init(115200);//sd卡通讯
	SysTick_Init(72);//时钟
	init_sample();//电流采样初始化
	LedInit();//LED灯
	CAN_Config();//CAN通讯
	///**Driveinit();//elmo驱动初始化
	delay_ms(500);
	usart1_Send_Data(&RS232_RX_BUF);	
	Timer3Init(200);//定时器中断初始化
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
						
 }														   //大while
															  //main函数

		
