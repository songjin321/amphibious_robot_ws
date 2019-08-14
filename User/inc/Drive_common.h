#ifndef __DRIVE_COMMON_H
#define	__DRIVE_COMMON_H

#include "stm32f10x.h"
#include "stm32f10x_can.h"
#define TMIDxR_TXRQ  ((uint32_t)0x00000001)
#define TMIDxR_TXOK  ((uint32_t)0x00000001)
#define TMIDxR_RQCP  ((uint32_t)0x00000001)

/****************************************
*	���ܣ���̬��װ
****************************************/

void Tri_common1(float speed1,float speed2, u8 delaycount);  //����1Ϊ��׼��motion1��ʾ֧�����˶���motion2��ʾ�ڶ����˶�
void Tri_common2(float speed1,float speed2, u8 delaycount);
void Hex_common1(float speed1, u8 delaycount);
void Hex_common2(float speed2, u8 delaycount);
void Hex_Tricommon(void);
void Tri_Hexcommon(void);
void Tri_Tetrcommon(void);	
void Hex_Tetrcommon(void);	
void Tetr_Tricommon(void);	
void Tetr_Hexcommon(void);	
void Stop_common(void);
void Tetr_common1(void);
void Tetr_common2(void);
void Tetr_common3(void);
void Turn_start(void);
void Turnleft_common1(void);
void Turnleft_common2(void);
void Turnright_common1(void);
void Turnright_common2(void);
void Turn_finish(void);
void Back_tri_common1(void);
void Back_tri_common2(void);
void Back_hex_common1(void);
void Back_hex_common2(void);
void Climbing_common1(void);
void Climbing_common2(void);
void Climbing2_common1(void);
void Climbing2_common2(void);
void Climbing2_finish(void);	
void climbing_init(void);
void stair_common1(void);
void stair_finish(void);	
void Tri_Swimcommon(void);
void Hex_Swimcommon(void);
void Swim_Tricommon(void);
void Swim_Hexcommon(void);
void Swim_common1(u16 speed);
void Swim_common2(u16 speed);
void Swim_start(void);
void Swim_finish(void);
void Swim_backstart(void);
void Swim_backfinish(void);
void Swim_turnstart(void);
void Swim_turnfinish(void);
void Swim_ascendstart(void);
void Swim_ascendfinish(void);
void Swim_descendstart(void);
void Swim_descendfinish(void);
void Swim_turn_common1(void);
void Swim_turn_common2(void);

extern float faststand_common ;
extern float fastswing_common ;
extern float slowstand_common ;
extern float slowswing_common ;
extern u8 arriveflag_common;
extern u8 Gaitflag_common;	//��̬��־��1Ϊ���㣬2Ϊ���㣬3Ϊ����
extern u8 Stopflag_common;
extern u8  delay_count;			  //ֹͣ��־
//extern __IO u8  count_flag;		  //velocityflag=1��ʾ���٣�=2��ʾ����
/*****************************************/   

#endif
