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
	Driveinit();//elmo驱动初始化
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
	while(1)
	{
		if((RS232_RX_CNT==1 )&&((RS232_RX_BUF &0x10)==0x00))	 //普通模式
		{
			while(1)
			{
				switch (RS232_RX_BUF)
				{
					case 0x00:				 //水中慢速巡航
						OpenSdcardFile("\r\nlow-speed swimming gait test data:\r\n");
						sample_flag=1;
						Swim_common_gait(0);//0
						break;
					case 0x01:
					    //Swim_backcommon_gait();		//水中后退
						OpenSdcardFile("\r\nmiddle-speed swimming gait test data:\r\n");
						sample_flag=1;
						Swim_common_gait(1);//水中中速巡航
						break;
					case 0x02:
					    //Swim_turncommon_gait();		//水中转弯
						OpenSdcardFile("\r\nhigh-speed swimming gait test data:\r\n");
						sample_flag=1;
						Swim_common_gait(2);//水中快速巡航
						break;
					case 0x03:
					    Swim_descendcommon_gait();	 //水中下降
						break;
					case 0x04:
					    Swim_ascendcommon_gait();	 //水中上升
						break;
					case 0x05:				   //两足
						stair_gait();
						break;
					case 0x06:				   //爬坡1  改变腿离地时间
						Climbing_gait();
						break;
					case 0x07:				  //倒退
						Back_gait();
						break;
					case 0x08:
						Climbing_gait2();	   //爬坡2	 改变接触角
						break;
					case 0x09:
						Turning_gait(1); //右转
						break;
					case 0x0A:
						Turning_gait(0);//0 左转
						break;
					case 0x0B:
						Tetrapod_gait();		   //四足
						break;
					case 0x0C:
						OpenSdcardFile("\r\nhigh-speed hexapod gait test data:\r\n");
						sample_flag=1;
						Hexapod_gait(1);		   //六足快
						break;
					case 0x0D:
						OpenSdcardFile("\r\nlow-speed hexapod gait test data:\r\n");
						sample_flag=1;
						Hexapod_gait(0);		   //六足慢
						break;
					case 0x0E:
						OpenSdcardFile("\r\nhigh-speed tripod gait test data:\r\n");
						sample_flag=1;
						Tripod_gait(1);			   //三足快
						break;
					case 0x0F:
						OpenSdcardFile("\r\nlow-speed tripod gait test data:\r\n");
						sample_flag=1;
						Tripod_gait(0);			   //三足慢
						break;
					default:
						break;	
				}
				if(((RS232_RX_BUF &0x10)==0x10) && (RS232_RX_CNT==1))                         //停止
	   			{			
					Stop_common();
					sample_flag=0;
					CloseFile();
	   			}
			}
		}														



															   //普通模式结束
       /* ---------------------------------------------------------------------------------*/
		if((RS232_RX_CNT==1 )&&((RS232_RX_BUF &0x10)==0x10))		 //CPG模式与柔性腿实验
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
						Tri_gait_cpg();		 //cpg三足
						break;					
					case 0x1D:
					    Tri_gait_cpg_fast();	 //cpg三足快
						break;
					case 0x1E:
						Hex_gait_cpg();		   //cpg六足
						break;
					case 0x1F:
						Hex_gait_cpg_fast();	//cpg六足快
						break;
					default:
						break;
				}
				if(((RS232_RX_BUF &0x10)==0x00) && (RS232_RX_CNT==1))                  //停止
	   			{				
					Stop_common();
					sample_flag=0;
				//	stopsample();
	   			}
	 		 }
		}
 		delay_ms(100);														  //CPG模式结束
 	}
 }														   //大while
															  //main函数

		
