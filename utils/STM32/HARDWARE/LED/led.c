#include "led.h"

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	//´ò¿ªGPIOCÊ±ÖÓ
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOC,&GPIO_InitStruct);
}

