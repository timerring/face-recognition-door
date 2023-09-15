#ifndef __LED_H_
#define __LED_H_
#include "stm32f10x.h"

#define     LED_On()     GPIO_ResetBits(GPIOC, GPIO_Pin_13)
#define     LED_Off()    GPIO_SetBits(GPIOC, GPIO_Pin_13)

void LED_Init(void);


#endif
