#include "Drive_common.h"
#include "can.h"
#include "led.h"
#include "Drive.h"
#include "systick.h"
#include "rs232.h"
#include "drive_common.h"
#include "gait.h"
#include "flipperleg.h"
#include "sampling.h"

void Tripod_gait_flipperleg(u8 speed)
{
	float land_tri_speed1=0,land_tri_speed2=0;
	switch(speed){
		case 1:
			land_tri_speed1=60;
			land_tri_speed2=180;
			delay_count=1;
			break;
		case 2:
			land_tri_speed1=90;
			land_tri_speed2=270;
			delay_count=2;
			break;
		case 3:
			land_tri_speed1=120;
			land_tri_speed2=360;
			delay_count=3;
			break;
		default:
			break;			
	}
	if(Gaitflag_common==10)
	{
		Swim_Tricommon();
		delay_ms(50);
		Gaitflag_common=1;
	}
	if(Gaitflag_common==2)
	{
		Hex_Tricommon();
		delay_ms(50);
		Gaitflag_common=1;
	}
	RS232_RX_CNT=0;					
	while(1)
	{
	//	SampleCurrent(10);
		Tri_common1(land_tri_speed1,land_tri_speed2,delay_count);
		Tri_common2(land_tri_speed1,land_tri_speed2,delay_count);
		if((RS232_RX_CNT==1 )&&(arriveflag_common==1))	break;
	}
}


void Hexapod_gait_flipperleg(u8 speed)
{
	float land_hex_speed1=0,land_hex_speed2=0;
		switch(speed){
		case 1:
			land_hex_speed1=120;
			land_hex_speed2=120;
			delay_count=1;
			break;
		case 2:
			land_hex_speed1=240;
			land_hex_speed2=240;
			delay_count=2;
			break;
		case 3:
			land_hex_speed1=360;
			land_hex_speed2=360;
			delay_count=3;
			break;
		default:
			break;			
	}
	if(Gaitflag_common==10)
	{
		Swim_Hexcommon();
		delay_ms(50);
		Gaitflag_common=2;
	}
	if(Gaitflag_common==1)
	{
		Tri_Hexcommon();
		delay_ms(50);
		Gaitflag_common=2;
	}
	RS232_RX_CNT=0; 				
	while(1)
	{
		Hex_common1(land_hex_speed1,delay_count);
		Hex_common2(land_hex_speed2,delay_count);
		if((RS232_RX_CNT==1 )&&(arriveflag_common==1))	break;
	}  
}



void Swim_common_gait_flipperleg()
{
   	if(Gaitflag_common==1)
	{
	 Tri_Swimcommon();
	 delay_ms(50);
	 Gaitflag_common=10;
	}
   	if(Gaitflag_common==2)
	{
	 Hex_Swimcommon();
	 delay_ms(50);
	 Gaitflag_common=10;
	}
	 Swim_start();
	 RS232_RX_CNT=0;	
	while(1)
	{
		Swim_common1();
		Swim_common2();
		if((RS232_RX_CNT==1 )&&(arriveflag_common==1))break;
	}
	Swim_finish();
}

