#include "sampling.h"
#include "string.h"


int res;  
FIL fdst1,fdst2,fdst3,fdst4,fdst5,fdst6,fdst7,fdst8,fdst9;
FATFS fs;
UINT br, bw;
BYTE CurrentTemp[Length];
BYTE uTimeTemp[Length];
BYTE uSpeedTemp[Length];
u8 sample_flag=0;
u16 Motor2_Speed=0;
u16 Motor4_Speed=0;
u16 Motor5_Speed=0;

void init_sample(void)
{
	res=f_mount(&fs,"0:",1);
	for(res=0;res<Length;res++)
	{
	CurrentTemp[res]='\0';
	uTimeTemp[res]='\0';
	uSpeedTemp[res]='\0';
	}

	OpenSdcardFile("\r\n please save the data below as soon as possible. \r\n");
	CloseFile();
}

void OpenSdcardFile(char *tip)
{
	f_open(&fdst1,"0:/current4.TXT",FA_OPEN_ALWAYS | FA_WRITE);//打开文件，如果不存在就新建
	//motor4 current
  f_open(&fdst2,"0:/uTime4.TXT",FA_OPEN_ALWAYS | FA_WRITE);
	//motor4 time
	f_open(&fdst3,"0:/speed4.TXT",FA_OPEN_ALWAYS | FA_WRITE);
	//motor4 speed
  f_open(&fdst4,"0:/current5.TXT",FA_OPEN_ALWAYS | FA_WRITE);
	//motor5 current
	f_open(&fdst5,"0:/utime5.TXT",FA_OPEN_ALWAYS | FA_WRITE);
	//motor5 time
  f_open(&fdst6,"0:/speed5.TXT",FA_OPEN_ALWAYS | FA_WRITE);
	//motor5 speed
	f_open(&fdst7,"0:/current2.TXT",FA_OPEN_ALWAYS | FA_WRITE);
	//motor2 current
	f_open(&fdst8,"0:/utime2.TXT",FA_OPEN_ALWAYS | FA_WRITE);
	//motor2 time
  f_open(&fdst9,"0:/speed2.TXT",FA_OPEN_ALWAYS | FA_WRITE);
	//motor2 speed
		f_lseek(&fdst1, f_size(&fdst1));
	f_write(&fdst1, tip, strlen(tip), &bw);
		f_lseek(&fdst2, f_size(&fdst2));
	f_write(&fdst2, tip, strlen(tip), &bw);
		f_lseek(&fdst3, f_size(&fdst3));
	f_write(&fdst3, tip, strlen(tip), &bw);
	
		f_lseek(&fdst4, f_size(&fdst4));
	f_write(&fdst4, tip, strlen(tip), &bw);
		f_lseek(&fdst5, f_size(&fdst5));
	f_write(&fdst5, tip, strlen(tip), &bw);
		f_lseek(&fdst6, f_size(&fdst6));
	f_write(&fdst6, tip, strlen(tip), &bw);
	
			f_lseek(&fdst7, f_size(&fdst7));
	f_write(&fdst7, tip, strlen(tip), &bw);
		f_lseek(&fdst8, f_size(&fdst8));
	f_write(&fdst8, tip, strlen(tip), &bw);
		f_lseek(&fdst9, f_size(&fdst9));
	f_write(&fdst9, tip, strlen(tip), &bw);
}
u16 ReadCurrent(u16 tCOB_ID)
{
	u16 ActualCurrent;
	u32 cnt;
	u16 rID;
	cnt=0;
	rID=tCOB_ID & 0x000f;
	rNOD_ID=0;
	//rflag5=0xff;
	CAN_SetMsg(0x00000000,0x00607840,tCOB_ID); //read actual current
	CAN_Transmit(CAN1,&TxMessage);
	while(1)
	{
		if(rCOB_TYPE==0x0B & rNOD_ID==rID)
		{
			ActualCurrent=RxMessage.Data[4] | RxMessage.Data[5]<<8;
		//	rflag5=0xff;
			rNOD_ID=0;
			return 	ActualCurrent;	
		}
		cnt+=1;
		if(cnt==60000)   return 0x7fff;
	}

}
u32 ReadTime(u16 tCOB_ID)
{
	u32 uTime;
	u32 cnt;
	u16 rID;
	cnt=0;
//	rflag5=0xff;
		rID=tCOB_ID & 0x000f;
	rNOD_ID=0;
	CAN_SetMsg(0x00000000,0x00204140,tCOB_ID); 	
	CAN_Transmit(CAN1,&TxMessage);
	while(1)
	{
		if(rCOB_TYPE==0x0B & rNOD_ID==rID)
		{
			uTime=RxMessage.Data[4] | RxMessage.Data[5]<<8 |
			RxMessage.Data[6]<<16 | RxMessage.Data[7]<<24;
	//		rflag5=0xff;
			rNOD_ID=0;
			return uTime;
		}
		cnt+=1;
		if(cnt==60000)
		return 0xffffffff;
	}
}

u16 ReadVelocity(u16 tCOB_ID)
{
	u16 ActualCurrent;
	u32 cnt;
	u16 rID;
	cnt=0;
	rID=tCOB_ID & 0x000f;
	rNOD_ID=0;
	//rflag5=0xff;
	CAN_SetMsg(0x00000000,0x00607840,tCOB_ID); //read actual current
	CAN_Transmit(CAN1,&TxMessage);
	while(1)
	{
		if(rCOB_TYPE==0x0B & rNOD_ID==rID)
		{
			ActualCurrent=RxMessage.Data[4] | RxMessage.Data[5]<<8;
		//	rflag5=0xff;
			rNOD_ID=0;
			return 	ActualCurrent;	
		}
		cnt+=1;
		if(cnt==60000)   return 0x7fff;
	}
}
void SampleCurrent(void)
{
		u32 uTime;
		u16 ActualCurrent;
		ActualCurrent=ReadCurrent(0x604);
		uTime=ReadTime(0x604);
		if(ActualCurrent>32767)
		{
			ActualCurrent=(~ActualCurrent)+1;
			sprintf(CurrentTemp,"\r\n-%d",ActualCurrent);		
		}
		else{
			sprintf(CurrentTemp,"\r\n%d",ActualCurrent);
		}
		sprintf(uTimeTemp,"\r\n%d",uTime);
		sprintf(uSpeedTemp,"\r\n%d",Motor4_Speed);		
		f_lseek(&fdst1, f_size(&fdst1));
		f_write(&fdst1, CurrentTemp, sizeof(CurrentTemp), &bw); //将缓冲区的数据写到文件中
		f_lseek(&fdst2, f_size(&fdst2));
		f_write(&fdst2, uTimeTemp, sizeof(uTimeTemp), &bw);
		f_lseek(&fdst3, f_size(&fdst3));
		f_write(&fdst3, uSpeedTemp, sizeof(uSpeedTemp), &bw);
		init_buffer();
		
		ActualCurrent=ReadCurrent(0x605);
		uTime=ReadTime(0x605);
		if(ActualCurrent>32767)
		{
			ActualCurrent=(~ActualCurrent)+1;
			sprintf(CurrentTemp,"\r\n-%d",ActualCurrent);		
		}
		else{
			sprintf(CurrentTemp,"\r\n%d",ActualCurrent);
		}
		sprintf(uTimeTemp,"\r\n%d",uTime);
		sprintf(uSpeedTemp,"\r\n%d",Motor5_Speed);		
		f_lseek(&fdst4, f_size(&fdst4));
		f_write(&fdst4, CurrentTemp, sizeof(CurrentTemp), &bw); //将缓冲区的数据写到文件中
		f_lseek(&fdst5, f_size(&fdst5));
		f_write(&fdst5, uTimeTemp, sizeof(uTimeTemp), &bw);
		f_lseek(&fdst6, f_size(&fdst6));
		f_write(&fdst6, uSpeedTemp, sizeof(uSpeedTemp), &bw);
		init_buffer();
		
		ActualCurrent=ReadCurrent(0x602);
		uTime=ReadTime(0x602);
		if(ActualCurrent>32767)
		{
			ActualCurrent=(~ActualCurrent)+1;
			sprintf(CurrentTemp,"\r\n-%d",ActualCurrent);		
		}
		else{
			sprintf(CurrentTemp,"\r\n%d",ActualCurrent);
		}
		sprintf(uTimeTemp,"\r\n%d",uTime);
		sprintf(uSpeedTemp,"\r\n%d",Motor4_Speed);		
		f_lseek(&fdst7, f_size(&fdst7));
		f_write(&fdst7, CurrentTemp, sizeof(CurrentTemp), &bw); //将缓冲区的数据写到文件中
		f_lseek(&fdst8, f_size(&fdst8));
		f_write(&fdst8, uTimeTemp, sizeof(uTimeTemp), &bw);
		f_lseek(&fdst9, f_size(&fdst9));
		f_write(&fdst9, uSpeedTemp, sizeof(uSpeedTemp), &bw);
		init_buffer();
}

void CloseFile(void)
{
		f_close(&fdst1);//关闭文件 
		f_close(&fdst2);
	f_close(&fdst3);//关闭文件 
		f_close(&fdst4);
	f_close(&fdst5);//关闭文件 
		f_close(&fdst6);
	f_close(&fdst7);//关闭文件 
		f_close(&fdst8);
	f_close(&fdst9);
}

void init_buffer(void)
{
for(res=0;res<Length;res++)
	{
	CurrentTemp[res]='\0';
	uTimeTemp[res]='\0';
	uSpeedTemp[res]='\0';
	}
}
