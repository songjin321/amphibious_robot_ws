#ifndef __DRIVE_H
#define	__DRIVE_H

#include "stm32f10x.h"
#include "stm32f10x_can.h"
#define TMIDxR_TXRQ  ((uint32_t)0x00000001)
#define TMIDxR_TXOK  ((uint32_t)0x00000001)
#define TMIDxR_RQCP  ((uint32_t)0x00000001)

/****************************************
*	功能：can指令封装
****************************************/
void Driveinit(void);
void SetControlMode(u8 ControlMode, u16 tCOB_ID);
void SetAcceleration(u32 Acceleration,u16 tCOB_ID);
void SetDeceleration(u32 Deceleration,u16 tCOB_ID);
void SetVelocity(float Velocity, u16 tCOB_ID);
void SetPosition(float Position, u16 tCOB_ID,u8 Direction);
void EnableMotor(u16 tCOB_ID);
void Halt(u16 tCOB_ID);
u32 ReadPosition(u16 tCOB_ID);
void ContinueMotor(u16 tCOB_ID);

void Tri_gait_cpg(void);
void Hex_gait_cpg(void);
void Halt_cpg(void);

void initgait(void);



#define factor 1470.75
#define DT 150
#define DT1 10

extern    __IO uint8_t MotorOn;


//extern __IO u8  count_flag;		  //velocityflag=1表示慢速，=2表示快速
/*****************************************/   

#endif
