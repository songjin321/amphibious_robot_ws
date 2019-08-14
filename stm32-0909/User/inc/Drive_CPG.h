#ifndef __DRIVE_CPG_H
#define	__DRIVE_CPG_H

#include "stm32f10x.h"
#include "stm32f10x_can.h"
#define TMIDxR_TXRQ  ((uint32_t)0x00000001)
#define TMIDxR_TXOK  ((uint32_t)0x00000001)
#define TMIDxR_RQCP  ((uint32_t)0x00000001)

/****************************************
*	���ܣ�������ʼ��
****************************************/

void landmotion_cpg(void);		  //����1Ϊ��׼��
void Drive_cpg(void );
void Tri_gait_cpg_fast(void);
void Hex_gait_cpg_fast(void);






extern float degree1_land_cpg,degree2_land_cpg,speed1_land_cpg,speed2_land_cpg;
extern uint32_t  velocityflag;		  //velocityflag=1��ʾ���٣�=2��ʾ����
extern __IO float degree1_DC;
extern __IO float degree1_cpg;
extern __IO float speed1_cpg,degree2_DC,degree2_cpg,speed2_cpg;

extern float time_DH;
extern u8 count_flag;
extern u8 statusflag_cpg;
extern float mapdegreefast,mapdegreeslow,mapdegree_hex,mapdegree_trifast,mapdegree_trislow;
/*****************************************/   

#endif
