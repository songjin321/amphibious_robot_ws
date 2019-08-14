#include "Drive.h"
#include "can.h"
#include "led.h"
#include "systick.h"
#include "CPG.h"
#include "MATH.h"

 float delta_h=0.02;

 float hopf_U1[3]={-1,0,0};
 float hopf_V1[3]={-0.1,0,0};
 float hopf_U2[3]={-1,0,0};
 float hopf_V2[3]={-0.1,0,0};
 float hopf_DU1=0,hopf_DU2=0,hopf_DV1=0,hopf_DV2=0;
 float hopf_f=0.25,hopf_fai=0,hopf_A=1;






void hopf_func(float u1,float v1,float u2,float v2,float fre,float fai,unsigned int A)
{
unsigned int  k=10;


float ee=1;
float pi=3.14;

hopf_DU1=k*(A*A-u1*u1-v1*v1)*u1-2*pi*fre*v1;
hopf_DV1=k*(A*A-u1*u1-v1*v1)*v1+2*pi*fre*u1+ee*(v2*cos(fai)-u2*sin(fai));
hopf_DU2=k*(A*A-u2*u2-v2*v2)*u2-2*pi*fre*v2;
hopf_DV2=k*(A*A-u2*u2-v2*v2)*v2+2*pi*fre*u2+ee*(u1*sin(fai)+v1*cos(fai));

}

void Runge_Kutta(void)
{
hopf_U1[2]=hopf_U1[1];
hopf_U1[1]=hopf_U1[0];
hopf_U2[2]=hopf_U2[1];
hopf_U2[1]=hopf_U2[0];
hopf_func(hopf_U1[0],hopf_V1[0],hopf_U2[0],hopf_V2[0],hopf_f,hopf_fai,hopf_A);
hopf_U1[0]=hopf_U1[0]+hopf_DU1*delta_h;
hopf_V1[0]=hopf_V1[0]+hopf_DV1*delta_h;
hopf_U2[0]=hopf_U2[0]+hopf_DU2*delta_h;
hopf_V2[0]=hopf_V2[0]+hopf_DV2*delta_h;
}


