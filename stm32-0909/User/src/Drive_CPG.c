#include "Drive.h"
#include "can.h"
#include "led.h"
#include "CPG.h"
#include "Drive_CPG.h"
#include "systick.h"
#include "rs232.h"


uint32_t  velocityflag=1;

float degree1_land_cpg=0,degree2_land_cpg=0,speed1_land_cpg=0,speed2_land_cpg=0;
__IO float degree1_DC=0;
__IO float degree1_cpg=0;
__IO float speed1_cpg=0,degree2_DC=0,degree2_cpg=0,speed2_cpg=0;
 float time_DH=0.02;
 u8 count_flag=0;
 u8 statusflag_cpg=0;
 float mapdegreefast=0,mapdegreeslow=0,mapdegree_hex=45,mapdegree_trifast=135,mapdegree_trislow=45,mapdegree_hexfast=135;

void Drive_cpg(void)
{

	Runge_Kutta();
	statusflag_cpg=0;
    if ((hopf_U1[0]-hopf_U1[1])>=0)
	{

      if ((hopf_U1[1]<=hopf_U1[0])&&(hopf_U1[1]<=hopf_U1[2]))
	  {
	  degree1_cpg=0;
	  count_flag=count_flag+1;
	  statusflag_cpg=1;
	  }
    degree1_DC=(hopf_U1[0]-hopf_U1[1])*mapdegreeslow;
	degree1_cpg=degree1_cpg+degree1_DC;
    speed1_cpg=degree1_DC/time_DH;

	}
	else
	{
	  if ((hopf_U1[1]>=hopf_U1[0])&&(hopf_U1[1]>=hopf_U1[2]))
	  degree1_cpg=90;
    degree1_DC=(hopf_U1[1]-hopf_U1[0])*mapdegreefast;
	degree1_cpg=degree1_cpg+degree1_DC;
    speed1_cpg=degree1_DC/time_DH;
	}

	if ((hopf_U2[0]-hopf_U2[1])>=0)
	{
	  if ((hopf_U2[1]<=hopf_U2[0])&&(hopf_U2[1]<=hopf_U2[2]))
	  degree2_cpg=0;
	degree2_DC=(hopf_U2[0]-hopf_U2[1])*mapdegreeslow;
	degree2_cpg=degree2_cpg+degree2_DC;
    speed2_cpg=degree2_DC/time_DH;
	}
	else
   	{
	  if ((hopf_U2[1]>=hopf_U2[0])&&(hopf_U2[1]>=hopf_U2[2]))
	  degree2_cpg=90;
    degree2_DC=(hopf_U2[1]-hopf_U2[0])*mapdegreefast;
	degree2_cpg=degree2_cpg+degree2_DC;
    speed2_cpg=degree2_DC/time_DH;
	}

}

void landmotion_cpg(void)
{
	Drive_cpg();
	degree1_land_cpg=degree1_DC;
	degree2_land_cpg=degree2_DC;
	speed1_land_cpg=speed1_cpg;
	speed2_land_cpg=speed2_cpg;
	SetPosition(degree1_land_cpg,0x601,1);
	SetVelocity(speed1_land_cpg,0x601); //motor 1
	SetPosition(degree1_land_cpg,0x603,1);
	SetVelocity(speed1_land_cpg,0x603);   //motor 3
	SetPosition(degree1_land_cpg,0x605,1);
	SetVelocity(speed1_land_cpg,0x605);   //motor 5

	SetPosition(degree2_land_cpg,0x602,1);
	SetVelocity(speed2_land_cpg,0x602);  //motor 2
	SetPosition(degree2_land_cpg,0x604,1);
	SetVelocity(speed2_land_cpg,0x604);  //motor 4
	SetPosition(degree2_land_cpg,0x606,1);
	SetVelocity(speed2_land_cpg,0x606);  //motor 6   
	EnableMotor(0x601);
	EnableMotor(0x602);
	EnableMotor(0x603);
	EnableMotor(0x604);
	EnableMotor(0x605);
	EnableMotor(0x606);
	delay_ms(19);
}

void Tri_gait_cpg(void)
{			//Èý×ãÂýCPG	   
	delay_ms(50);
	statusflag_cpg=0;
	hopf_fai=-3.14;
	hopf_f=0.25;
	mapdegreefast=mapdegree_trifast;
	mapdegreeslow=mapdegree_trislow;
	RS232_RX_CNT=0;
	while(1)
	{
	 landmotion_cpg();
	 if((RS232_RX_CNT==1 )&&(statusflag_cpg==1))
      break;
	}	
}
void Hex_gait_cpg(void)
{ 					  //Áù×ãÂýCPG
	delay_ms(50);
	statusflag_cpg=0;
	hopf_fai=0;
	hopf_f=0.25;
	mapdegreefast=mapdegree_hexfast;
	mapdegreeslow=mapdegree_hex;
	RS232_RX_CNT=0;
	while(1)
	{
	 landmotion_cpg();
	 if((RS232_RX_CNT==1 )&&(statusflag_cpg==1))
      break;
	}
}
void Tri_gait_cpg_fast(void)
{
	delay_ms(50);
	statusflag_cpg=0;
	hopf_fai=-3.14;
	hopf_f=0.35;
	mapdegreefast=mapdegree_trifast;
	mapdegreeslow=mapdegree_trislow;
	RS232_RX_CNT=0;
	while(1)
	{
	 	landmotion_cpg();
	 	if((RS232_RX_CNT==1 )&&(statusflag_cpg==1))
      	break;
	}
}
void Hex_gait_cpg_fast(void)
{
	delay_ms(50);
	statusflag_cpg=0;
	hopf_fai=0;
	hopf_f=0.35;
	mapdegreefast=mapdegree_hexfast;
	mapdegreeslow=mapdegree_hex;
	RS232_RX_CNT=0;
	while(1)
	{
	 	landmotion_cpg();
	 	if((RS232_RX_CNT==1 )&&(statusflag_cpg==1))
      	break;
	}
}
void Halt_cpg(void)
{                        //Í£Ö¹

	RS232_RX_CNT=0;
	statusflag_cpg=0;
	mapdegreefast=0;
	mapdegreeslow=0;
	landmotion_cpg();
	delay_ms(20);
}
