#include "Drive_common.h"
#include "can.h"
#include "led.h"
#include "Drive.h"
#include "systick.h"
#include "rs232.h"
#include "sampling.h"

// #define speed1_common 45.25
// #define speed2_common 135.25
 float faststand_common=40 ;
 float fastswing_common=200 ;
 float slowstand_common=20 ;
 float slowswing_common=100 ;
u8 arriveflag_common=0;
u8 Gaitflag_common=0;	//步态标志，1为三足，2为六足	  10为水中
u8 Stopflag_common=0;
u8  delay_count=0;

 

float     velocitystand,velocityswing;

void Tri_common1(float speed1,float speed2, u8 delaycount)
{
	arriveflag_common=0;
	SetPosition(90,0x601,1);
	SetVelocity(speed1,0x601); //motor 1
	SetPosition(90,0x603,1);
	SetVelocity(speed1,0x603);   //motor 3
	SetPosition(90,0x605,1);
	SetVelocity(speed1,0x605);   //motor 5

	SetPosition(270,0x602,1);
	SetVelocity(speed2,0x602);  //motor 2
	SetPosition(270,0x604,1);
	SetVelocity(speed2,0x604);  //motor 4
	SetPosition(270,0x606,1);
	SetVelocity(speed2,0x606);  //motor 6   t=2
	EnableMotor(0x601);
	EnableMotor(0x602);
	EnableMotor(0x603);
	EnableMotor(0x604);
	EnableMotor(0x605);
	EnableMotor(0x606);
	
	Motor4_Speed=speed2;
	Motor5_Speed=speed1;
	if(delaycount==1)			  //慢速，3S周期1.5S支撑相1.5秒摆动相
	{
	    delay_ms(1530);
	}
   	if(delaycount==2)			  //快速，1.5S周期0.75S支撑相0.75S摆动相
	{
	    delay_ms(770);
	}

}

void Tri_common2(float speed1,float speed2, u8 delaycount)
{
	SetPosition(90,0x602,1);
	SetVelocity(speed1,0x602); //motor 2
	SetPosition(90,0x604,1);
	SetVelocity(speed1,0x604);   //motor 4
	SetPosition(90,0x606,1);
	SetVelocity(speed1,0x606);   //motor 6
	SetPosition(270,0x601,1);
	SetVelocity(speed2,0x601);  //motor 1
	SetPosition(270,0x603,1);
	SetVelocity(speed2,0x603);  //motor 3
	SetPosition(270,0x605,1);
	SetVelocity(speed2,0x605);  //motor 5   t=2
	EnableMotor(0x601);
	EnableMotor(0x602);
	EnableMotor(0x603);
	EnableMotor(0x604);
	EnableMotor(0x605);
	EnableMotor(0x606);
	Motor4_Speed=speed1;
	Motor5_Speed=speed2;
	if(delaycount==1)			  //慢速，3S周期1.5S支撑相1.5秒摆动相
	{
	    delay_ms(1530);
	}
   	if(delaycount==2)			  //快速，1.5S周期0.75S支撑相0.75S摆动相
	{
	    delay_ms(770);
	}
	arriveflag_common=1;
}

void Climbing_common1(void)
{
	arriveflag_common=0;
	SetPosition(90,0x601,1);
	SetVelocity(45,0x601); //motor 1
	SetPosition(90,0x603,1);
	SetVelocity(45,0x603);   //motor 3
	SetPosition(90,0x605,1);
	SetVelocity(45,0x605);   //motor 5

	SetPosition(270,0x602,1);
	SetVelocity(135,0x602);  //motor 2
	SetPosition(270,0x604,1);
	SetVelocity(360,0x604);  //motor 4   
	SetPosition(270,0x606,1);
	SetVelocity(180,0x606);  //motor 6
	


	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	EnableMotor(0x602);
	delay_ms(500);
	EnableMotor(0x606);
	delay_ms(750);
	EnableMotor(0x604);	
         	delay_ms(770);

}

void Climbing_common2(void)
{
	SetPosition(90,0x602,1);
	SetVelocity(45,0x602); //motor 2
	SetPosition(90,0x604,1);
	SetVelocity(45,0x604);   //motor 4
	SetPosition(90,0x606,1);
	SetVelocity(45,0x606);   //motor 6

	SetPosition(270,0x601,1);
	SetVelocity(135,0x601);  //motor 1
	SetPosition(270,0x603,1);
	SetVelocity(270,0x603);  //motor 3
	SetPosition(270,0x605,1);
	SetVelocity(180,0x605);  //motor 5  

	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	delay_ms(500);
	EnableMotor(0x605);
	delay_ms(750);
	EnableMotor(0x603);
	delay_ms(770);
	arriveflag_common=1;
}
void climbing_init(void)
{
	SetPosition(15,0x601,1);
	SetVelocity(15,0x601); //motor 1
	SetPosition(15,0x603,1);
	SetVelocity(15,0x603);   //motor 3
	SetPosition(15,0x605,1);
	SetVelocity(15,0x605);   //motor 5

	SetPosition(15,0x602,0);
	SetVelocity(30,0x602);  //motor 2
	SetPosition(15,0x604,0);
	SetVelocity(30,0x604);  //motor 4   
	SetPosition(15,0x606,0);
	SetVelocity(30,0x606);  //motor 6
	


	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	delay_ms(500);
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	delay_ms(550);
			
}

void Climbing2_common1(void)
{
	arriveflag_common=0;
	SetPosition(60,0x601,1);
	SetVelocity(40,0x601); //motor 1
	SetPosition(60,0x603,1);
	SetVelocity(40,0x603);   //motor 3
	SetPosition(60,0x605,1);
	SetVelocity(40,0x605);   //motor 5

	SetPosition(300,0x602,1);
	SetVelocity(200,0x602);  //motor 2
	SetPosition(300,0x604,1);
	SetVelocity(200,0x604);  //motor 4   
	SetPosition(300,0x606,1);
	SetVelocity(200,0x606);  //motor 6
	


	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	EnableMotor(0x602);
	EnableMotor(0x606);
	EnableMotor(0x604);	
         	delay_ms(1520);

}


void Climbing2_common2(void)
{
	SetPosition(60,0x602,1);
	SetVelocity(40,0x602); //motor 2
	SetPosition(60,0x604,1);
	SetVelocity(40,0x604);   //motor 4
	SetPosition(60,0x606,1);
	SetVelocity(40,0x606);   //motor 6

	SetPosition(300,0x601,1);
	SetVelocity(200,0x601);  //motor 1
	SetPosition(300,0x603,1);
	SetVelocity(200,0x603);  //motor 3
	SetPosition(300,0x605,1);
	SetVelocity(200,0x605);  //motor 5  

	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x605);
	EnableMotor(0x603);
	delay_ms(1520);
	arriveflag_common=1;
}

void Climbing2_finish(void)
{
	SetPosition(15,0x601,0);
	SetVelocity(15,0x601); //motor 1
	SetPosition(15,0x603,0);
	SetVelocity(15,0x603);   //motor 3
	SetPosition(15,0x605,0);
	SetVelocity(15,0x605);   //motor 5

	SetPosition(15,0x602,1);
	SetVelocity(30,0x602);  //motor 2
	SetPosition(15,0x604,1);
	SetVelocity(30,0x604);  //motor 4   
	SetPosition(15,0x606,1);
	SetVelocity(30,0x606);  //motor 6
	


	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	delay_ms(500);
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	delay_ms(550);	
}

void stair_init(void)
{
	SetPosition(90,0x602,1);
	SetVelocity(90,0x602); //motor 2
	SetPosition(90,0x604,1);
	SetVelocity(90,0x604);   //motor 4
	SetPosition(90,0x606,1);
	SetVelocity(90,0x606);   //motor 6

	SetPosition(90,0x601,1);
	SetVelocity(90,0x601);  //motor 1
	SetPosition(90,0x603,1);
	SetVelocity(90,0x603);  //motor 3
	SetPosition(90,0x605,1);
	SetVelocity(90,0x605);  //motor 5  

	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x605);
	EnableMotor(0x603);
	delay_ms(1020);
	
}

void stair_common1(void)
{
	SetPosition(360,0x602,1);
	SetVelocity(180,0x602); //motor 2
	SetPosition(360,0x604,1);
	SetVelocity(180,0x604);   //motor 4
	SetPosition(360,0x606,1);
	SetVelocity(180,0x606);   //motor 6

	SetPosition(360,0x601,1);
	SetVelocity(180,0x601);  //motor 1
	SetPosition(360,0x603,1);
	SetVelocity(180,0x603);  //motor 3
	SetPosition(360,0x605,1);
	SetVelocity(180,0x605);  //motor 5  

	EnableMotor(0x601);
	EnableMotor(0x602);
	delay_ms(1000);
	delay_ms(1030);
	EnableMotor(0x605);
	EnableMotor(0x606);
	delay_ms(1000);
	delay_ms(1030);
	EnableMotor(0x603);
	EnableMotor(0x604);
	delay_ms(1000);
	delay_ms(1030);	
}

void stair_finish(void)
{
	SetPosition(90,0x602,0);
	SetVelocity(90,0x602); //motor 2
	SetPosition(90,0x604,0);
	SetVelocity(90,0x604);   //motor 4
	SetPosition(90,0x606,0);
	SetVelocity(90,0x606);   //motor 6

	SetPosition(90,0x601,0);
	SetVelocity(90,0x601);  //motor 1
	SetPosition(90,0x603,0);
	SetVelocity(90,0x603);  //motor 3
	SetPosition(90,0x605,0);
	SetVelocity(90,0x605);  //motor 5  

	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x605);
	EnableMotor(0x603);
	delay_ms(1020);		
}

void Tri_Hexcommon(void)					   //三足转六足切换
{
	SetPosition(270,0x602,1);
	SetVelocity(270,0x602);  //motor 2
	SetPosition(270,0x604,1);
	SetVelocity(270,0x604);  //motor 4
	SetPosition(270,0x606,1);
	SetVelocity(270,0x606);  //motor 6   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	Motor4_Speed=270;
	Motor5_Speed=0;
	delay_ms(1030);
}

void Hex_Tricommon(void)					 //六足转三足切换
{
	SetPosition(90,0x602,1);
	SetVelocity(90,0x602); //motor 2
	SetPosition(90,0x604,1);
	SetVelocity(90,0x604);   //motor 4
	SetPosition(90,0x606,1);
	SetVelocity(90,0x606);   //motor 6
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	Motor4_Speed=0;
	Motor4_Speed=90;
	delay_ms(1030);
}

void Tri_Tetrcommon(void)					   //三足转四足切换
{
	SetPosition(285,0x601,0);
	SetVelocity(285,0x601); //motor 1
	SetPosition(285,0x604,1);
	SetVelocity(285,0x604);   //motor 4
	SetPosition(285,0x606,1);
	SetVelocity(285,0x606);   //motor 6
	EnableMotor(0x601);
	EnableMotor(0x604);
	EnableMotor(0x606);
	delay_ms(1030);
}

void Hex_Tetrcommon(void)					  //六足转四足切换
{
	SetPosition(75,0x601,1);
	SetVelocity(75,0x601);   //motor 1
	SetPosition(75,0x602,1);
	SetVelocity(75,0x602);   //motor 2
	SetPosition(75,0x603,1);
	SetVelocity(75,0x603);   //motor 3
	SetPosition(75,0x604,1);
	SetVelocity(75,0x604);   //motor 4
	SetPosition(15,0x605,1);
	SetVelocity(15,0x605);   //motor 5
	SetPosition(15,0x606,1);
	SetVelocity(15,0x606);   //motor 6
	EnableMotor(0x601);
	EnableMotor(0x602);
	EnableMotor(0x603);
	EnableMotor(0x604);
	EnableMotor(0x605);
	EnableMotor(0x606);
	delay_ms(1030);
}

void Tetr_Tricommon(void)					   //四足转三足切换
{
	SetPosition(285,0x601,1);
	SetVelocity(285,0x601); //motor 1
	SetPosition(285,0x604,0);
	SetVelocity(285,0x604);   //motor 4
	SetPosition(285,0x606,0);
	SetVelocity(285,0x606);   //motor 6
	EnableMotor(0x601);
	EnableMotor(0x604);
	EnableMotor(0x606);
	delay_ms(1030);
}

void Tetr_Hexcommon(void)					  //四足转六足切换
{
	SetPosition(75,0x601,0);
	SetVelocity(75,0x601);   //motor 1
	SetPosition(75,0x602,0);
	SetVelocity(75,0x602);   //motor 2
	SetPosition(75,0x603,0);
	SetVelocity(75,0x603);   //motor 3
	SetPosition(75,0x604,0);
	SetVelocity(75,0x604);  //motor 4
	SetPosition(15,0x605,0);
	SetVelocity(15,0x605);  //motor 5
	SetPosition(15,0x606,0);
	SetVelocity(15,0x606);  //motor 6   
	EnableMotor(0x601);
	EnableMotor(0x602);
	EnableMotor(0x603);
	EnableMotor(0x604);
	EnableMotor(0x605);
	EnableMotor(0x606);
	delay_ms(1030);
}

void Swim_Tricommon(void)					   //水中转三足切换
{
	SetPosition(315,0x602,1);
	SetVelocity(315,0x602);  //motor 2
	SetPosition(315,0x604,1);
	SetVelocity(315,0x604);  //motor 4
	SetPosition(315,0x606,1);
	SetVelocity(315,0x606);  //motor 6   t=2
	SetPosition(225,0x601,1);
	SetVelocity(225,0x601);  //motor 1
	SetPosition(225,0x603,1);
	SetVelocity(225,0x603);  //motor 3
	SetPosition(225,0x605,1);
	SetVelocity(225,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	delay_ms(1030);
}

void Swim_Hexcommon(void)					   //水中转六足切换
{
	SetPosition(225,0x602,1);
	SetVelocity(225,0x602);  //motor 2
	SetPosition(225,0x604,1);
	SetVelocity(225,0x604);  //motor 4
	SetPosition(225,0x606,1);
	SetVelocity(225,0x606);  //motor 6   t=2
	SetPosition(225,0x601,1);
	SetVelocity(225,0x601);  //motor 1
	SetPosition(225,0x603,1);
	SetVelocity(225,0x603);  //motor 3
	SetPosition(225,0x605,1);
	SetVelocity(225,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	delay_ms(1030);
}

void Hex_common1(float speed1, u8 delaycount)
{
  arriveflag_common=0;
  SetPosition(180,0x602,1);
	SetVelocity(speed1,0x602); //motor 2
	SetPosition(180,0x604,1);
	SetVelocity(speed1,0x604);   //motor 4
	SetPosition(180,0x606,1);
	SetVelocity(speed1,0x606);   //motor 6
	SetPosition(180,0x601,1);
	SetVelocity(speed1,0x601);  //motor 1
	SetPosition(180,0x603,1);
	SetVelocity(speed1,0x603);  //motor 3
	SetPosition(180,0x605,1);
	SetVelocity(speed1,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	Motor4_Speed=speed1;
	Motor5_Speed=speed1;
	if(delaycount==1)			  //慢速，3S周期1.5S支撑相1.5秒摆动相
	{
	    delay_ms(1530);
	}
   	if(delaycount==2)			  //快速，1.5S周期0.75S支撑相0.75S摆动相
	{
	    delay_ms(770);
	}
}

void Hex_common2(float speed2, u8 delaycount)
{
   	SetPosition(180,0x602,1);
	SetVelocity(speed2,0x602); //motor 2
	SetPosition(180,0x604,1);
	SetVelocity(speed2,0x604);   //motor 4
	SetPosition(180,0x606,1);
	SetVelocity(speed2,0x606);   //motor 6
	SetPosition(180,0x601,1);
	SetVelocity(speed2,0x601);  //motor 1
	SetPosition(180,0x603,1);
	SetVelocity(speed2,0x603);  //motor 3
	SetPosition(180,0x605,1);
	SetVelocity(speed2,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	Motor4_Speed=speed2;
	Motor5_Speed=speed2;
	if(delaycount==1)			  //慢速，3S周期1.5S支撑相1.5秒摆动相
	{
	    delay_ms(1530);
	}
   	if(delaycount==2)			  //快速，1.5S周期0.75S支撑相0.75S摆动相
	{
	    delay_ms(770);
	}
	arriveflag_common=1;
}

void Tetr_common1(void)
{
	arriveflag_common=0;
   	SetPosition(300,0x601,1);
	SetVelocity(300,0x601);   //motor 1
	SetPosition(300,0x602,1);
	SetVelocity(300,0x602);   //motor 2
	SetPosition(0,0x603,1);
	SetVelocity(0,0x603);   //motor 3
	SetPosition(0,0x604,1);
	SetVelocity(0,0x604);  //motor 4
	SetPosition(60,0x605,1);
	SetVelocity(60,0x605);  //motor 5
	SetPosition(60,0x606,1);
	SetVelocity(60,0x606);  //motor 6   
	EnableMotor(0x601);
	EnableMotor(0x602);
	EnableMotor(0x603);
	EnableMotor(0x604);
	EnableMotor(0x605);
	EnableMotor(0x606);
	delay_ms(1030);
}

void Tetr_common2(void)
{
   	SetPosition(60,0x601,1);
	SetVelocity(60,0x601);   //motor 1
	SetPosition(60,0x602,1);
	SetVelocity(60,0x602);   //motor 2
	SetPosition(300,0x603,1);
	SetVelocity(300,0x603);   //motor 3
	SetPosition(300,0x604,1);
	SetVelocity(300,0x604);  //motor 4
	SetPosition(0,0x605,1);
	SetVelocity(0,0x605);  //motor 5
	SetPosition(0,0x606,1);
	SetVelocity(0,0x606);  //motor 6   
	EnableMotor(0x601);
	EnableMotor(0x602);
	EnableMotor(0x603);
	EnableMotor(0x604);
	EnableMotor(0x605);
	EnableMotor(0x606);
	delay_ms(1030);
}

void Tetr_common3(void)
{
   	SetPosition(0,0x601,1);
	SetVelocity(0,0x601);   //motor 1
	SetPosition(0,0x602,1);
	SetVelocity(0,0x602);   //motor 2
	SetPosition(60,0x603,1);
	SetVelocity(60,0x603);   //motor 3
	SetPosition(60,0x604,1);
	SetVelocity(60,0x604);  //motor 4
	SetPosition(300,0x605,1);
	SetVelocity(300,0x605);  //motor 5
	SetPosition(300,0x606,1);
	SetVelocity(300,0x606);  //motor 6   
	EnableMotor(0x601);
	EnableMotor(0x602);
	EnableMotor(0x603);
	EnableMotor(0x604);
	EnableMotor(0x605);
	EnableMotor(0x606);
	delay_ms(1030);
	arriveflag_common=1;
}

void Stop_common(void)
{
    RS232_RX_CNT=0;
	Halt(0x601);
	Halt(0x602);
	Halt(0x603);
	Halt(0x604);
	Halt(0x605);
	Halt(0x606);
	Motor4_Speed=0;
	Motor5_Speed=0;
}

void Turn_start(void)
{
	SetPosition(30,0x602,1);
	SetVelocity(60,0x602); //motor 2	
	SetPosition(30,0x604,1);
	SetVelocity(60,0x604); //motor 4	
	SetPosition(300,0x606,0);
	SetVelocity(200,0x606); //motor 6	
	SetPosition(30,0x601,1);
	SetVelocity(60,0x601); //motor 1
	SetPosition(30,0x603,1);
	SetVelocity(60,0x603); //motor 3
   	SetPosition(300,0x605,0);
	SetVelocity(200,0x605); //motor 5

	EnableMotor(0x601);
	EnableMotor(0x602);
	EnableMotor(0x603);
	EnableMotor(0x604);
	delay_ms(500);
	EnableMotor(0x605);
	EnableMotor(0x606);
	delay_ms(1520);
}

void Turnleft_common1(void)
{
	arriveflag_common=0;
   	SetPosition(30,0x602,1);
	SetVelocity(30,0x602); //motor 2	
	SetPosition(30,0x604,1);
	SetVelocity(30,0x604); //motor 4	
	SetPosition(30,0x606,0);
	SetVelocity(30,0x606); //motor 6	
	SetPosition(330,0x601,0);
	SetVelocity(330,0x601); //motor 1
	SetPosition(330,0x603,0);
	SetVelocity(330,0x603); //motor 3
	SetPosition(330,0x605,1);
	SetVelocity(330,0x605); //motor 5
	EnableMotor(0x601);
	EnableMotor(0x602);
	EnableMotor(0x603);
	EnableMotor(0x604);
	EnableMotor(0x605);
	EnableMotor(0x606);
	delay_ms(1030);
}

void Turnleft_common2(void)
{
   	SetPosition(330,0x602,1);
	SetVelocity(330,0x602); //motor 2	
	SetPosition(330,0x604,1);
	SetVelocity(330,0x604); //motor 4
	SetPosition(330,0x606,0);
	SetVelocity(330,0x606); //motor 6
	SetPosition(30,0x601,0);
	SetVelocity(30,0x601); //motor 1
	SetPosition(30,0x603,0);
	SetVelocity(30,0x603); //motor 3
	SetPosition(30,0x605,1);
	SetVelocity(30,0x605); //motor 5
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	delay_ms(1030);
	arriveflag_common=1;
}


void Turnright_common1(void)
{
	arriveflag_common=0;
   	SetPosition(30,0x601,1);
	SetVelocity(30,0x601); //motor 1	
	SetPosition(30,0x603,1);
	SetVelocity(30,0x603); //motor 3	
	SetPosition(30,0x605,0);
	SetVelocity(30,0x605); //motor 5	
	SetPosition(330,0x602,0);
	SetVelocity(330,0x602); //motor 2
	SetPosition(330,0x604,0);
	SetVelocity(330,0x604); //motor 4
	SetPosition(330,0x606,1);
	SetVelocity(330,0x606); //motor 6
	EnableMotor(0x601);
	EnableMotor(0x602);
	EnableMotor(0x603);
	EnableMotor(0x604);
	EnableMotor(0x605);
	EnableMotor(0x606);
	delay_ms(1030);
}

void Turnright_common2(void)
{
   	SetPosition(330,0x601,1);
	SetVelocity(330,0x601); //motor 1	
	SetPosition(330,0x603,1);
	SetVelocity(330,0x603); //motor 3
	SetPosition(330,0x605,0);
	SetVelocity(330,0x605); //motor 5
	SetPosition(30,0x602,0);
	SetVelocity(30,0x602); //motor 2
	SetPosition(30,0x604,0);
	SetVelocity(30,0x604); //motor 4
	SetPosition(30,0x606,1);
	SetVelocity(30,0x606); //motor 6
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	delay_ms(1030);
	arriveflag_common=1;
}

void Turn_finish(void)
{
   	SetPosition(30,0x602,0);
	SetVelocity(30,0x602); //motor 2	
	SetPosition(30,0x604,0);
	SetVelocity(30,0x604); //motor 4	
	SetPosition(300,0x606,1);
	SetVelocity(200,0x606); //motor 6	
	SetPosition(30,0x601,0);
	SetVelocity(30,0x601); //motor 1
	SetPosition(30,0x603,0);
	SetVelocity(30,0x603); //motor 3
   	SetPosition(300,0x605,1);
	SetVelocity(200,0x605); //motor 5

	EnableMotor(0x605);
	EnableMotor(0x606);
	delay_ms(500);
	EnableMotor(0x601);
	EnableMotor(0x602);
	EnableMotor(0x603);
	EnableMotor(0x604);
	delay_ms(1020);
}

void Back_tri_common1(void)
{
	arriveflag_common=0;
	SetPosition(270,0x601,0);
	SetVelocity(270,0x601);   //motor 1
	SetPosition(270,0x603,0);
	SetVelocity(270,0x603);   //motor 3
	SetPosition(270,0x605,0);
	SetVelocity(270,0x605);   //motor 5

	SetPosition(90,0x602,0);
	SetVelocity(90,0x602);  //motor 2
	SetPosition(90,0x604,0);
	SetVelocity(90,0x604);  //motor 4
	SetPosition(90,0x606,0);
	SetVelocity(90,0x606);  //motor 6   t=2
	EnableMotor(0x601);
	EnableMotor(0x602);
	EnableMotor(0x603);
	EnableMotor(0x604);
	EnableMotor(0x605);
	EnableMotor(0x606);
	delay_ms(1030);
}

void Back_tri_common2(void)
{	
	SetPosition(90,0x601,0);
	SetVelocity(90,0x601); //motor 1
	SetPosition(90,0x603,0);
	SetVelocity(90,0x603);   //motor 3
	SetPosition(90,0x605,0);
	SetVelocity(90,0x605);   //motor 5

	SetPosition(270,0x602,0);
	SetVelocity(270,0x602);  //motor 2
	SetPosition(270,0x604,0);
	SetVelocity(270,0x604);  //motor 4
	SetPosition(270,0x606,0);
	SetVelocity(270,0x606);  //motor 6   t=2
	EnableMotor(0x601);
	EnableMotor(0x602);
	EnableMotor(0x603);
	EnableMotor(0x604);
	EnableMotor(0x605);
	EnableMotor(0x606);
	delay_ms(1030);
	arriveflag_common=1;
}

void Back_hex_common1(void)
{
	arriveflag_common=0;	
	SetPosition(180,0x601,0);
	SetVelocity(180,0x601); //motor 1
	SetPosition(180,0x603,0);
	SetVelocity(180,0x603);   //motor 3
	SetPosition(180,0x605,0);
	SetVelocity(180,0x605);   //motor 5

	SetPosition(180,0x602,0);
	SetVelocity(180,0x602);  //motor 2
	SetPosition(180,0x604,0);
	SetVelocity(180,0x604);  //motor 4
	SetPosition(180,0x606,0);
	SetVelocity(180,0x606);  //motor 6   
	EnableMotor(0x601);
	EnableMotor(0x602);
	EnableMotor(0x603);
	EnableMotor(0x604);
	EnableMotor(0x605);
	EnableMotor(0x606);
	delay_ms(1030);
}

void Back_hex_common2(void)
{	
	SetPosition(180,0x601,0);
	SetVelocity(180,0x601); //motor 1
	SetPosition(180,0x603,0);
	SetVelocity(180,0x603);   //motor 3
	SetPosition(180,0x605,0);
	SetVelocity(180,0x605);   //motor 5

	SetPosition(180,0x602,0);
	SetVelocity(180,0x602);  //motor 2
	SetPosition(180,0x604,0);
	SetVelocity(180,0x604);  //motor 4
	SetPosition(180,0x606,0);
	SetVelocity(180,0x606);  //motor 6   
	EnableMotor(0x601);
	EnableMotor(0x602);
	EnableMotor(0x603);
	EnableMotor(0x604);
	EnableMotor(0x605);
	EnableMotor(0x606);
	delay_ms(1030);
	arriveflag_common=1;
}

void Tri_Swimcommon(void)
{
	SetPosition(45,0x602,1);
	SetVelocity(45,0x602);  //motor 2
	SetPosition(45,0x604,1);
	SetVelocity(45,0x604);  //motor 4
	SetPosition(45,0x606,1);
	SetVelocity(45,0x606);  //motor 6   t=2
	SetPosition(135,0x601,1);
	SetVelocity(135,0x601);  //motor 1
	SetPosition(135,0x603,1);
	SetVelocity(135,0x603);  //motor 3
	SetPosition(135,0x605,1);
	SetVelocity(135,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	Motor4_Speed=45;
	Motor5_Speed=135;
	delay_ms(1030);
}

void Hex_Swimcommon(void)
{
	SetPosition(135,0x602,1);
	SetVelocity(135,0x602);  //motor 2
	SetPosition(135,0x604,1);
	SetVelocity(135,0x604);  //motor 4
	SetPosition(135,0x606,1);
	SetVelocity(135,0x606);  //motor 6   t=2
	SetPosition(135,0x601,1);
	SetVelocity(135,0x601);  //motor 1
	SetPosition(135,0x603,1);
	SetVelocity(135,0x603);  //motor 3
	SetPosition(135,0x605,1);
	SetVelocity(135,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	Motor4_Speed=135;
	Motor5_Speed=135;
	delay_ms(1030);
}





void Swim_start(void)
{
    arriveflag_common=0;
 	SetPosition(45,0x602,1);
	SetVelocity(45,0x602);  //motor 2
	SetPosition(45,0x604,1);
	SetVelocity(45,0x604);  //motor 4
	SetPosition(45,0x606,0);
	SetVelocity(45,0x606);  //motor 6   t=2
	SetPosition(45,0x601,1);
	SetVelocity(45,0x601);  //motor 1
	SetPosition(45,0x603,1);
	SetVelocity(45,0x603);  //motor 3
	SetPosition(45,0x605,0);
	SetVelocity(45,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	Motor4_Speed=45;
	Motor5_Speed=45;
	delay_ms(1050);
}

void Swim_common1(u16 speed)
{

	u16 ve;
	u16 delaytime;
	float amp;
	amp=90;
	switch(speed){
		case 0:
			ve=150;
			delaytime=650;
			break;
		case 1:
			ve=200;
			delaytime=500;
			break;
		case 2:
			ve=300;
			delaytime=350;
			break;
		default:
			break;
	}
	arriveflag_common=0;
 	SetPosition(amp,0x602,0);
	SetVelocity(ve,0x602);  //motor 2
	SetPosition(amp,0x604,0);
	SetVelocity(ve,0x604);  //motor 4
	SetPosition(amp,0x606,1);
	SetVelocity(ve,0x606);  //motor 6   t=2
	SetPosition(amp,0x601,0);
	SetVelocity(ve,0x601);  //motor 1
	SetPosition(amp,0x603,0);
	SetVelocity(ve,0x603);  //motor 3
	SetPosition(amp,0x605,1);
	SetVelocity(ve,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	arriveflag_common=1;
	Motor4_Speed=ve;
	Motor5_Speed=ve;
	delay_ms(delaytime);
}

void Swim_common2(u16 speed)
{
	u16 ve;
	u16 delaytime;
  float amp;
	amp=90;
	switch(speed){
		case 0:
			ve=150;
			delaytime=650;
			break;
		case 1:
			ve=200;
			delaytime=500;
			break;
		case 2:
			ve=300;
			delaytime=350;
			break;
		default:
			break;
	}
 	SetPosition(amp,0x602,1);
	SetVelocity(ve,0x602);  //motor 2
	SetPosition(amp,0x604,1);
	SetVelocity(ve,0x604);  //motor 4
	SetPosition(amp,0x606,0);
	SetVelocity(ve,0x606);  //motor 6   t=2
	SetPosition(amp,0x601,1);
	SetVelocity(ve,0x601);  //motor 1
	SetPosition(amp,0x603,1);
	SetVelocity(ve,0x603);  //motor 3
	SetPosition(amp,0x605,0);
	SetVelocity(ve,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	arriveflag_common=1;
	Motor4_Speed=ve;
	Motor5_Speed=ve;
	delay_ms(delaytime);
}

void Swim_finish(void)
{
 	SetPosition(45,0x602,0);
	SetVelocity(45,0x602);  //motor 2
	SetPosition(45,0x604,0);
	SetVelocity(45,0x604);  //motor 4
	SetPosition(45,0x606,1);
	SetVelocity(45,0x606);  //motor 6   t=2
	SetPosition(45,0x601,0);
	SetVelocity(45,0x601);  //motor 1
	SetPosition(45,0x603,0);
	SetVelocity(45,0x603);  //motor 3
	SetPosition(45,0x605,1);
	SetVelocity(45,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	delay_ms(1050);
	Motor4_Speed=45;
	Motor5_Speed=45;
	arriveflag_common=1;
}

void Swim_backstart(void)
{
    arriveflag_common=0;
 	SetPosition(150,0x602,1);
	SetVelocity(150,0x602);  //motor 2
	SetPosition(150,0x604,1);
	SetVelocity(150,0x604);  //motor 4
	SetPosition(150,0x606,1);
	SetVelocity(150,0x606);  //motor 6   t=2
	SetPosition(150,0x601,1);
	SetVelocity(150,0x601);  //motor 1
	SetPosition(150,0x603,1);
	SetVelocity(150,0x603);  //motor 3
	SetPosition(150,0x605,1);
	SetVelocity(150,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	delay_ms(1030);
}

void Swim_backfinish(void)
{
 	SetPosition(150,0x602,0);
	SetVelocity(150,0x602);  //motor 2
	SetPosition(150,0x604,0);
	SetVelocity(150,0x604);  //motor 4
	SetPosition(150,0x606,0);
	SetVelocity(150,0x606);  //motor 6   t=2
	SetPosition(150,0x601,0);
	SetVelocity(150,0x601);  //motor 1
	SetPosition(150,0x603,0);
	SetVelocity(150,0x603);  //motor 3
	SetPosition(150,0x605,0);
	SetVelocity(150,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	delay_ms(1030);
	arriveflag_common=1;
}

void Swim_turnstart(void)
{
    arriveflag_common=0;
 	SetPosition(150,0x602,1);
	SetVelocity(150,0x602);  //motor 2
	SetPosition(150,0x604,1);
	SetVelocity(150,0x604);  //motor 4
	SetPosition(30,0x606,1);
	SetVelocity(30,0x606);  //motor 6   t=2
	SetPosition(30,0x601,1);
	SetVelocity(30,0x601);  //motor 1
	SetPosition(30,0x603,1);
	SetVelocity(30,0x603);  //motor 3
	SetPosition(150,0x605,1);
	SetVelocity(150,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	delay_ms(1030);
}

void Swim_turnfinish(void)
{
 	SetPosition(150,0x602,0);
	SetVelocity(150,0x602);  //motor 2
	SetPosition(150,0x604,0);
	SetVelocity(150,0x604);  //motor 4
	SetPosition(30,0x606,0);
	SetVelocity(30,0x606);  //motor 6   t=2
	SetPosition(30,0x601,0);
	SetVelocity(30,0x601);  //motor 1
	SetPosition(30,0x603,0);
	SetVelocity(30,0x603);  //motor 3
	SetPosition(150,0x605,0);
	SetVelocity(150,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	delay_ms(1030);
	arriveflag_common=1;
}

void Swim_ascendstart(void)
{
    arriveflag_common=0;
 	SetPosition(90,0x602,0);
	SetVelocity(90,0x602);  //motor 2
	SetPosition(90,0x604,0);
	SetVelocity(90,0x604);  //motor 4
	SetPosition(90,0x606,0);
	SetVelocity(90,0x606);  //motor 6   t=2
	SetPosition(90,0x601,0);
	SetVelocity(90,0x601);  //motor 1
	SetPosition(90,0x603,0);
	SetVelocity(90,0x603);  //motor 3
	SetPosition(90,0x605,0);
	SetVelocity(90,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	delay_ms(1030);
}

void Swim_ascendfinish(void)
{   
 	SetPosition(90,0x602,1);
	SetVelocity(90,0x602);  //motor 2
	SetPosition(90,0x604,1);
	SetVelocity(90,0x604);  //motor 4
	SetPosition(90,0x606,1);
	SetVelocity(90,0x606);  //motor 6   t=2
	SetPosition(90,0x601,1);
	SetVelocity(90,0x601);  //motor 1
	SetPosition(90,0x603,1);
	SetVelocity(90,0x603);  //motor 3
	SetPosition(90,0x605,1);
	SetVelocity(90,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	delay_ms(1030);
	arriveflag_common=1;
}

void Swim_descendstart(void)
{
    arriveflag_common=0;
 	SetPosition(90,0x602,1);
	SetVelocity(90,0x602);  //motor 2
	SetPosition(90,0x604,1);
	SetVelocity(90,0x604);  //motor 4
	SetPosition(90,0x606,1);
	SetVelocity(90,0x606);  //motor 6   t=2
	SetPosition(90,0x601,1);
	SetVelocity(90,0x601);  //motor 1
	SetPosition(90,0x603,1);
	SetVelocity(90,0x603);  //motor 3
	SetPosition(90,0x605,1);
	SetVelocity(90,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	delay_ms(1030);
}

void Swim_descendfinish(void)
{   
 	SetPosition(90,0x602,0);
	SetVelocity(90,0x602);  //motor 2
	SetPosition(90,0x604,0);
	SetVelocity(90,0x604);  //motor 4
	SetPosition(90,0x606,0);
	SetVelocity(90,0x606);  //motor 6   t=2
	SetPosition(90,0x601,0);
	SetVelocity(90,0x601);  //motor 1
	SetPosition(90,0x603,0);
	SetVelocity(90,0x603);  //motor 3
	SetPosition(90,0x605,0);
	SetVelocity(90,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	delay_ms(1030);
	arriveflag_common=1;
}
	
void Swim_turn_common1(void)
{
	u8 ve;
	ve=240;
	arriveflag_common=0;
 	SetPosition(60,0x602,1);
	SetVelocity(ve,0x602);  //motor 2
	SetPosition(60,0x604,1);
	SetVelocity(ve,0x604);  //motor 4
	SetPosition(60,0x606,0);
	SetVelocity(ve,0x606);  //motor 6   t=2
	SetPosition(60,0x601,0);
	SetVelocity(ve,0x601);  //motor 1
	SetPosition(60,0x603,0);
	SetVelocity(ve,0x603);  //motor 3
	SetPosition(60,0x605,1);
	SetVelocity(ve,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	arriveflag_common=1;
	delay_ms(270);
}

void Swim_turn_common2(void)
{
	u8 ve;
	ve=240;
	arriveflag_common=0;
 	SetPosition(60,0x602,0);
	SetVelocity(ve,0x602);  //motor 2
	SetPosition(60,0x604,0);
	SetVelocity(ve,0x604);  //motor 4
	SetPosition(60,0x606,1);
	SetVelocity(ve,0x606);  //motor 6   t=2
	SetPosition(60,0x601,1);
	SetVelocity(ve,0x601);  //motor 1
	SetPosition(60,0x603,1);
	SetVelocity(ve,0x603);  //motor 3
	SetPosition(60,0x605,0);
	SetVelocity(ve,0x605);  //motor 5   t=2
	EnableMotor(0x602);
	EnableMotor(0x604);
	EnableMotor(0x606);
	EnableMotor(0x601);
	EnableMotor(0x603);
	EnableMotor(0x605);
	arriveflag_common=1;
	delay_ms(270);
}
