#include "Drive.h"
#include "can.h"
#include "led.h"
#include "systick.h"
#include"RS232.H"

__IO uint8_t MotorOn=0;

//__IO u8 count_flag=0;
 
void Driveinit(void )
{
	u8 initflag1,initflag2,initflag3,initflag4,initflag5,initflag6;
	initflag1=0;
	initflag2=0;
	initflag3=0;
	initflag4=0;
	initflag5=0;
	initflag6=0;
	while(1)
	{
		if(rflag1==0 & initflag1==0)     //0x0E表示error700
	  	{
		CAN_SetMsg(0x00000006,0x0060402b,0x601);
		CAN_Transmit(CAN1,&TxMessage);
		delay_ms(DT);
	           //switch on
	 	CAN_SetMsg(0x00000007,0x0060402b,0x601);
		CAN_Transmit(CAN1,&TxMessage);
		 //switch on
		delay_ms(DT);
		
	 	CAN_SetMsg(0x0000000F,0x0060402b,0x601);
		CAN_Transmit(CAN1,&TxMessage);
		delay_ms(DT);
//		IsLed1On(YES);	
		initflag1=1;
		rflag1=0xff;
		}

		if(rflag2==0 & initflag2==0)     //0x0E表示error700
	  	{
		CAN_SetMsg(0x00000006,0x0060402b,0x602);
		CAN_Transmit(CAN1,&TxMessage);
		delay_ms(DT);
	           //switch on
	 	CAN_SetMsg(0x00000007,0x0060402b,0x602);
		CAN_Transmit(CAN1,&TxMessage);
		 //switch on
		delay_ms(DT);
		
	 	CAN_SetMsg(0x0000000F,0x0060402b,0x602);
		CAN_Transmit(CAN1,&TxMessage);
		delay_ms(DT);
//		IsLed2On(YES);	
		initflag2=1;
		rflag2=0xff;
		}

		if(rflag3==0  & initflag3==0)     //0x0E表示error700
	  	{
		CAN_SetMsg(0x00000006,0x0060402b,0x603);
		CAN_Transmit(CAN1,&TxMessage);
		delay_ms(DT);
	           //switch on
	 	CAN_SetMsg(0x00000007,0x0060402b,0x603);
		CAN_Transmit(CAN1,&TxMessage);
		 //switch on
		delay_ms(DT);
		
	 	CAN_SetMsg(0x0000000F,0x0060402b,0x603);
		CAN_Transmit(CAN1,&TxMessage);
		delay_ms(DT);
//		IsLed3On(YES);	
		initflag3=1;
		rflag3=0xff;
		}

		if(rflag4==0 & initflag4==0)     //0x0E表示error700
	  	{
		CAN_SetMsg(0x00000006,0x0060402b,0x604);
		CAN_Transmit(CAN1,&TxMessage);
		delay_ms(DT);
	           //switch on
	 	CAN_SetMsg(0x00000007,0x0060402b,0x604);
		CAN_Transmit(CAN1,&TxMessage);
		 //switch on
		delay_ms(DT);
		
	 	CAN_SetMsg(0x0000000F,0x0060402b,0x604);
		CAN_Transmit(CAN1,&TxMessage);
		delay_ms(DT);
//		IsLed4On(YES);	
		initflag4=1;
		rflag4=0xff;
		}
		if(rflag5==0 & initflag5==0)     //0x0E表示error700
	  	{
		CAN_SetMsg(0x00000006,0x0060402b,0x605);
		CAN_Transmit(CAN1,&TxMessage);
		delay_ms(DT);
	           //switch on
	 	CAN_SetMsg(0x00000007,0x0060402b,0x605);
		CAN_Transmit(CAN1,&TxMessage);
		 //switch on
		delay_ms(DT);
		
	 	CAN_SetMsg(0x0000000F,0x0060402b,0x605);
		CAN_Transmit(CAN1,&TxMessage);
		delay_ms(DT);
//		IsLed4On(YES);	
		initflag5=1;
		rflag5=0xff;
		}
		if(rflag6==0 & initflag6==0)     //0x0E表示error700
	  	{
		CAN_SetMsg(0x00000006,0x0060402b,0x606);
		CAN_Transmit(CAN1,&TxMessage);
		delay_ms(DT);
	           //switch on
	 	CAN_SetMsg(0x00000007,0x0060402b,0x606);
		CAN_Transmit(CAN1,&TxMessage);
		 //switch on
		delay_ms(DT);
		
	 	CAN_SetMsg(0x0000000F,0x0060402b,0x606);
		CAN_Transmit(CAN1,&TxMessage);
		delay_ms(DT);
//		IsLed4On(YES);	
		initflag6=1;
		rflag6=0xff;
		}
		if(initflag1 & initflag2 & initflag3 & initflag4 & initflag5 & initflag6)
		{

		usart1_Send_Data("初始化完成：");
		IsLed3On(YES);
		break;
		}
	}
}


void SetControlMode(u8 ControlMode, u16 tCOB_ID)
{
	
	CAN_SetMsg((uint32_t)ControlMode,0x0060602F,tCOB_ID);
	CAN_Transmit(CAN1,&TxMessage);

	/*
	-128…-2 Reserved
	-1 No mode
	0 Reserved
	1 Profile position mode
	2 Velocity (not supported)
	3 Profiled velocity mode
	4 Torque profiled mode
	5 Reserved
	6 Homing mode
	7 Interpolated position mode
	8…127 Reserved
	*/	
}
void SetVelocity(float Velocity, u16 tCOB_ID)
{
//    CAN_SetMsg(0x00000000,0x00606140,tCOB_ID); //modes of operation display
//    CAN_Transmit(CAN1,&TxMessage);

//    while(1)
//    {
//    	if(flag==0 && rCOB_TYPE==0x0B & rNOD_ID==1 ) //0x0B表示580
//	{
//		switch (RxMessage.Data[4])  
//		{
//		case  1:
//			CAN_SetMsg(Velocity*factor,0x00608123,tCOB_ID); 
//    			CAN_Transmit(CAN1,&TxMessage);      //当前为位置模式
//			break;
//		case  3:
//			CAN_SetMsg(Velocity*factor,0x0060ff23,tCOB_ID); 
//    			CAN_Transmit(CAN1,&TxMessage);      //当前为速度模式
//			break;
//		default:
//			break;
//		}
//		break;
//	}
//    }
	
	CAN_SetMsg(Velocity*factor,0x00608123,tCOB_ID); 
    	CAN_Transmit(CAN1,&TxMessage);      //当前为位置模式
}
void SetPosition(float Position, u16 tCOB_ID,  u8 Direction)  //deirection=0 反转
{
	switch( Direction )
	{
	case 0:
		CAN_SetMsg(1-(uint32_t)(Position*factor ),0x00607A23,tCOB_ID); 
    	CAN_Transmit(CAN1,&TxMessage);      //位置都为相对位置
		break;
	case 1:
		CAN_SetMsg((uint32_t)(Position*factor ),0x00607A23,tCOB_ID); 
    	CAN_Transmit(CAN1,&TxMessage);      //位置都为相对位置
		break;
	default:
	break;
	}
//	IsLed2On(YES);
	
}
void EnableMotor(u16 tCOB_ID)
{
	CAN_SetMsg(0x0000025F,0x0060402b,tCOB_ID);
	CAN_Transmit(CAN1,&TxMessage);	//使能
//	IsLed3On(YES);
	CAN_SetMsg(0x0000000F,0x0060402b,tCOB_ID);
	CAN_Transmit(CAN1,&TxMessage);     //clear speed
	MotorOn=1;
}
void Halt(u16 tCOB_ID)
{
	CAN_SetMsg(0x00000002,0x00605D2b,tCOB_ID);
	CAN_Transmit(CAN1,&TxMessage);
	MotorOn=0;
}				        
void SetAcceleration(u32 Acceleration,u16 tCOB_ID)
{
	CAN_SetMsg(Acceleration,0x00608323,tCOB_ID); 
    	CAN_Transmit(CAN1,&TxMessage);      		
}
void SetDeceleration(u32 Deceleration,u16 tCOB_ID)
{
	CAN_SetMsg(Deceleration,0x00608423,tCOB_ID); 
    	CAN_Transmit(CAN1,&TxMessage);
}
u32 ReadPosition(u16 tCOB_ID)
{
	u32 ActualPosition;
	rflag1=0xff;
	CAN_SetMsg(0x00000000,0x00606440,tCOB_ID); //read actual position
	CAN_Transmit(CAN1,&TxMessage);
	while(1)
	{
		if(rflag1==0 && rCOB_TYPE==0x0B & rNOD_ID==1)
		{
			ActualPosition=RxMessage.Data[4] | RxMessage.Data[5]<<8 |
				RxMessage.Data[6]<<16 | RxMessage.Data[5]<<24;
			return ActualPosition;	
		}
	}

}
void ContinueMotor(u16 tCOB_ID)
{
	CAN_SetMsg(0x0000000F,0x0060402b,0x601);
	CAN_Transmit(CAN1,&TxMessage);
}


void initgait(void)						//初始位置在顶端中间，支撑相90度，摆动相270度
{
	u32 ac;
	if((RS232_RX_BUF&0x10)==0x10)
	{
	ac=1e6;	   //cpg mode
	}
	else ac=1e7;  // normal mode
	SetControlMode(1,0x601);
	SetControlMode(1,0x602);
	SetControlMode(1,0x603);
	SetControlMode(1,0x604);
	SetControlMode(1,0x605);
	SetControlMode(1,0x606);
	SetAcceleration(ac,0x601);
	SetDeceleration(ac,0x601);
	SetAcceleration(ac,0x602);
	SetDeceleration(ac,0x602);
	SetAcceleration(ac,0x603);
	SetDeceleration(ac,0x603);
	SetAcceleration(ac,0x604);
	SetDeceleration(ac,0x604);
	SetAcceleration(ac,0x605);
	SetDeceleration(ac,0x605);
	SetAcceleration(ac,0x606);
	SetDeceleration(ac,0x606);

	SetPosition(135,0x601,1);
	SetVelocity(45,0x601);
	SetPosition(135,0x603,1);
	SetVelocity(45,0x603);
	SetPosition(135,0x605,1);
	SetVelocity(45,0x605);

	SetPosition(135,0x602,1);
	SetVelocity(45,0x602);
	SetPosition(135,0x604,1);
	SetVelocity(45,0x604);
	SetPosition(135,0x606,1); //这个电机的方向理解应当和其余电机相反
	SetVelocity(45,0x606);
	EnableMotor(0x601);
	EnableMotor(0x602);
	EnableMotor(0x603);
	EnableMotor(0x604);
	EnableMotor(0x605);
	EnableMotor(0x606);
	delay_ms(100);
}


