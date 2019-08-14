#include "Drive_common.h"
#include "can.h"
#include "led.h"
#include "Drive.h"
#include "systick.h"
#include "rs232.h"
#include "drive_common.h"
#include "gait.h"
#include "sampling.h"

void Tripod_gait(u8 speed)
{
	float land_tri_speed1=0,land_tri_speed2=0;
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
	if(speed==0 ) //Èý×ãÂý
	{	    	    
		land_tri_speed1=60;
		land_tri_speed2=180;
		delay_count=1;					
		while(1)
		{
		Tri_common1(land_tri_speed1,land_tri_speed2,delay_count);
		Tri_common2(land_tri_speed1,land_tri_speed2,delay_count);
		if((RS232_RX_CNT==1 )&&(arriveflag_common==1))
		 break;
		}
	  }														//Èý×ãÂý½áÊø
	  //---------------------------
	  if(speed==1)  //Èý×ã¿ì
	  {
		land_tri_speed1=120;
		land_tri_speed2=360;
		delay_count=2;	
		while(1)
		{
		Tri_common1(land_tri_speed1,land_tri_speed2,delay_count);
		Tri_common2(land_tri_speed1,land_tri_speed2,delay_count);
		if((RS232_RX_CNT==1 )&&(arriveflag_common==1))
		 break;
		}
	  }
}
void Hexapod_gait(u8 speed)
{
	float land_hex_speed1=0,land_hex_speed2=0;
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
	if(speed==0)//Áù×ãÂý
	{		
			
		land_hex_speed1=120;
		land_hex_speed2=120;	
		delay_count=1;
		while(1)
		{
			Hex_common1(land_hex_speed1,delay_count);
			Hex_common2(land_hex_speed2,delay_count);
		if((RS232_RX_CNT==1 )&&(arriveflag_common==1))
		 break;
		}
	}   //Áù×ãÂý½áÊø
	   //--------------------------------
	if(speed==1)//Áù×ã¿ì
	{	
		land_hex_speed1=240;
		land_hex_speed2=240;
		delay_count=2;
		while(1)
		{
			Hex_common1(land_hex_speed1,delay_count);
			Hex_common2(land_hex_speed2,delay_count);
	   	 	if((RS232_RX_CNT==1 )&&(arriveflag_common==1))
		 	break;
		}
	} //Áù×ã¿ì½áÊø
}
void Tetrapod_gait(void)
{//ËÄ×ã
	   
   if((Gaitflag_common==1)||(Gaitflag_common==2))
   {	    
	if(Gaitflag_common==1)
	{
		Tri_Hexcommon();
	}
	    
	Hex_Tetrcommon();
	RS232_RX_CNT=0;			
	while(1)
	{
		Tetr_common1();
    Tetr_common2();
		Tetr_common3();
		if((RS232_RX_CNT==1 )&&(arriveflag_common==1))
		 break;
		}
	    if(Gaitflag_common==1)
	    {
	    Tetr_Hexcommon();
	    Hex_Tricommon();
	    delay_ms(50);
	    }
	    if(Gaitflag_common==2)
	    {
	    Tetr_Hexcommon();
	    delay_ms(50);
		}	
	} //ËÄ×ã½áÊø
}
void Turning_gait(u8 direction)
{                      
   if((Gaitflag_common==1)||(Gaitflag_common==2))
   {
	if(Gaitflag_common==1)
	{
		Tri_Hexcommon();
	}
	Turn_start();
	RS232_RX_CNT=0;
	if(direction==0) 
	{    //×ó×ªÍä
		while(1)
		{
			Turnleft_common1();
			Turnleft_common2();
			if((RS232_RX_CNT==1 )&&(arriveflag_common==1))break;
		}
	}													//×ó×ªÍä½áÊø

	if(direction==1)//ÓÒ×ªÍä
	{		
		 while(1)
		 {
			Turnright_common1();
			Turnright_common2();
			if((RS232_RX_CNT==1 )&&(arriveflag_common==1))break;
		 }
	}
	Turn_finish();
	if(Gaitflag_common==1)
	{
		Hex_Tricommon();
	}
   }
}
void Back_gait(void)
{                        //µ¹ÍË
   if((Gaitflag_common==1)||(Gaitflag_common==2))
   {
	delay_ms(500);
	RS232_RX_CNT=0;
	if(Gaitflag_common==1)
	{
		while(1)
		{
		Back_tri_common1();
		Back_tri_common2();
		if((RS232_RX_CNT==1 )&&(arriveflag_common==1))
		 break;
		}
	    }
	    if(Gaitflag_common==2)
	    {
		while(1)
		{
		Back_hex_common1();
		Back_hex_common2();
		if((RS232_RX_CNT==1 )&&(arriveflag_common==1))
		 break;
		}
	    }
	}		
}
void Climbing_gait(void)
{              //ÅÀÆÂ
   if((Gaitflag_common==1)||(Gaitflag_common==2))
   {
	if(Gaitflag_common==2)
	{
		Hex_Tricommon();
	   	delay_ms(50);
	    	Gaitflag_common=1;
	}
	RS232_RX_CNT=0;			
	while(1)
	{
		Climbing_common1();
		Climbing_common2();
		if((RS232_RX_CNT==1 )&&(arriveflag_common==1))break;
	}													 
   }
}

void stair_gait(void)
{
   if((Gaitflag_common==1)||(Gaitflag_common==2))
   {
	if(Gaitflag_common==1)
	{
		Tri_Hexcommon();
	   	delay_ms(50);
	    	Gaitflag_common=1;
	}
	stair_init();
	RS232_RX_CNT=0;			
	while(1)
	{
		stair_common1();
		if(RS232_RX_CNT==1 )break;
	}
	stair_finish();
	Gaitflag_common=2;
   }
}
void Climbing_gait2(void)
{	
   if((Gaitflag_common==1)||(Gaitflag_common==2))
   {
	if(Gaitflag_common==2)
	{
		Hex_Tricommon();
	   	delay_ms(50);
	    	Gaitflag_common=1;
	}
	climbing_init();
	RS232_RX_CNT=0;			
	while(1)
	{
		Climbing2_common1();
		Climbing2_common2();
		if((RS232_RX_CNT==1 )&&(arriveflag_common==1))break;
	}
	Climbing2_finish();	
   }
}

void Swim_common_gait(u16 speed)
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
		Swim_common1(speed);
		Swim_common2(speed);
		if((RS232_RX_CNT==1 )&&(arriveflag_common==1))break;
	}
	Swim_finish();
}

void Swim_backcommon_gait()
{
   	if(Gaitflag_common==10)
	{
	 Swim_backstart();
	 RS232_RX_CNT=0;
	while(1)
	{
		Swim_common2(0);
		Swim_common1(0);
		if((RS232_RX_CNT==1 )&&(arriveflag_common==1))break;
	}
	Swim_backfinish();
	}
}

void Swim_turncommon_gait()
{
   	if(Gaitflag_common==10)
	{
	 Swim_turnstart();
	 RS232_RX_CNT=0;
	while(1)
	{
		Swim_turn_common1();
		Swim_turn_common2();
		if((RS232_RX_CNT==1 )&&(arriveflag_common==1))break;
	}
	Swim_turnfinish();
	}
}

void Swim_descendcommon_gait()
{
   	if(Gaitflag_common==10)
	{
	 Swim_descendstart();
	 RS232_RX_CNT=0;
	while(1)
	{
		Swim_common1(0);
		Swim_common2(0);
		if((RS232_RX_CNT==1 )&&(arriveflag_common==1))break;
	}
	Swim_ascendfinish();
	}
}

void Swim_ascendcommon_gait()
{
   	if(Gaitflag_common==10)
	{
	 Swim_ascendstart();
	 RS232_RX_CNT=0;
	while(1)
	{
		Swim_common2(0);
		Swim_common1(0);
		if((RS232_RX_CNT==1 )&&(arriveflag_common==1))break;
	}
	Swim_descendfinish();
	}
}




