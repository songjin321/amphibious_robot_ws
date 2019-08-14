#ifndef __SAMPLING_H
#define	__SAMPLING_H




#include "stm32f10x.h"
#include "stm32f10x_can.h"

#include "can.h"
#include "stdio.h"
#include "stdlib.h"
#include "Timer.h"
#include "systick.h"
#include "sdio_sdcard.h"
#include "ff.h"


#define Length 12
/****************************************
*	功能：采集电流
****************************************/


extern BYTE CurrentTemp[Length];
extern BYTE uTimeTemp[Length];
extern BYTE uSpeedTemp[Length];
extern u8 sample_flag;
extern FIL fdst1;
extern FIL fdst2;
extern FATFS fs;
extern u16 Motor2_Speed;
extern u16 Motor4_Speed;
extern u16 Motor5_Speed;
u16 ReadCurrent(u16 tCOB_ID);
u32 ReadTime(u16 tCOB_ID);
void SampleCurrent(void);
void init_sample(void);
void CloseFile(void);
void OpenSdcardFile(char *tip);
void init_buffer(void);
//extern __IO u8  count_flag;		  //velocityflag=1表示慢速，=2表示快速
/*****************************************/   

#endif
