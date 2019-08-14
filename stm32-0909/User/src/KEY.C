#include "KEY.H"
Switch switch2,switch3;
void KeyInit(void)//按键相关IO初始化
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOD, ENABLE);//使能GPIOD,GPIOE时钟
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;		
	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;		
	GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化



	switch2.PressedDowTimes=0;
	switch2.CheckLowTimes  =0;
	switch3.PressedDowTimes=0;
	switch3.CheckLowTimes  =0;
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE);//使能GPIOC时钟
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD ;//下拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;		
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC5	
}


